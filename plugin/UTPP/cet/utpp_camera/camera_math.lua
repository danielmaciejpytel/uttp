-- UTPP utpp_camera - pure camera math. No game API here: tested offline.
-- Own design. Frame-rate independent filters (1 - exp(-k dt)).
--
-- Conventions (REDengine world): Z up, yaw 0 looks along +Y, positive yaw turns left (counter-clockwise from above):
--   forward(yaw, pitch) = (-sin yaw * cos pitch, cos yaw * cos pitch, sin pitch); quaternions are {i, j, k, r}.
-- Camera component space = entity (player root) space: no parent binding (design 2.1 binding (a)).
-- Pivot height from the capsule (player position + stance height,
-- crouch from the PSM, blended) instead of the animated FPP camera (pivotSource "capsule" | "fpp"); follow-mode yaw
-- limiter (mouse turns pass at once, the rest of the body turn is followed at <= followMaxRate deg/s with ease-in) and a
-- world-yaw hold after a body flip (> flipDeg within flipWindow s, held flipHold s); per-transition enter time
-- (T.enterOverride: return from aiming); CM.lookReset (orbit yaw back behind the back).
-- (over-the-shoulder aim in third person): CM.aimWanted (enter / exit decision), CM.newAim /
-- CM.aimStep (aim weight tween: blend in aimBlendIn s, out aimBlendOut s, smoothstep like CM.blendTo), CM.aimProfile /
-- CM.aimCompose (aim pose over the live profile), CM.newAimSwap / CM.aimSwapStep (shoulder auto-swap with hysteresis);
-- lookStep env.aim / env.aimW (forced follow: no free look / idle orbit, yaw offset and pitch offset from the start of
-- the aim aligned with the aim weight tween, no limiter lag, wider pitch range), solve inp.aim / inp.aimW / inp.fovDirect (FOV follows the tween, no extra lag).
-- Starting values: 0.8 m, side 0.45 m, FOV 42, 0.12 / 0.20 s.
-- aim occlusion, level A (detection only, the shot is not changed): CM.aimOccT0 / aimOccRay1 /
-- aimOccPoint / aimOccDecide / aimOccSample (camera ray from the muzzle plane t0 -> aim point P, muzzle -> P ray, blocked
-- = hit clearly before P), CM.newAimOcc / aimOccDue (throttle: >= aimOccMinFrames frames and <= aimOccHz) /
-- aimOccStep (hysteresis aimOccHold s on both edges) / aimOccReset.
-- unarmed zoom: RMB with EMPTY hands (PSM UpperBody Aim, nothing in WeaponRight) in third person.
-- CM.zoomUnarmedSteps / zoomUnarmedPose (step k -> distance / FOV), CM.zoomUnarmedWanted (enter / stay decision),
-- CM.newZoomUnarmed / zoomUnarmedStep (weight tween like the aim, wheel steps, "fpp handoff" past the last step) /
-- zoomUnarmedCancelHandoff / zoomUnarmedActive / zoomUnarmedCompose (pose over the live profile).
-- (camera jerks, aim twists): lookStep aim no longer snaps the yaw onto the body yaw
-- within the 0.12 s pose tween; the shown yaw / pitch at the aim start is kept and its offset to the aim reference
-- (env.refYaw = the game's FPP look yaw, fallback body yaw; FPP pitch) shrinks at aimAlignGain 1/s, >= aimAlignMin and
-- <= aimAlignRate deg/s (CM.alignStep). CM.calibrate: outlier ratios rejected once calibrated (band), a streak re-seeds.
-- CM.predictTurn: entity yaw step of the coming game tick predicted from the last body step (free camera only).
-- (game look sync): lookStep env.lookYaw = the game's look yaw (FPP camera world yaw; its world heading
-- is kept while V turns by the FPP camera parameter set cfg.lookSyncSet, fppCameraParamSets.tweak VehicleCombat:
-- headingLocked + maintainWorldSpaceHeading) -> the free / own (aim) yaw and pitch ARE the game's look (the mouse scale,
-- the ADS sensitivity and the pad are the game's); an offset from the switch moment shrinks with CM.alignStep. nil = own
-- mouse integration (fallback). CM.newLookSync / CM.lookSyncSample: evidence for the sync (look follows the mouse while V
-- stands, or stays in the world while V turns) and its failure (look carried by V's turn -> resend, then broken).
-- aim point feed: CM.AIM_POINT (internal constants), CM.newAimPoint / aimPointReset / aimPointStep (1/e
-- filter of the crosshair point distance), CM.aimPointRay / aimTargetRay (native target org / fwd; P sent as P - fwd * 60).

local CM = { VERSION = "0.1.6" }

-- ================================================================ scalars
local abs, sqrt, sin, cos, exp, rad, deg, atan2, asin, floor =
  math.abs, math.sqrt, math.sin, math.cos, math.exp, math.rad, math.deg, math.atan2, math.asin, math.floor

function CM.clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local clamp = CM.clamp
function CM.lerp(a, b, t) return a + (b - a) * t end
function CM.smoothstep(t) t = clamp(t, 0, 1); return t * t * (3 - 2 * t) end
-- wrap to (-180, 180]
function CM.wrapDeg(a)
  a = a % 360
  if a > 180 then a = a - 360 end
  return a
end
-- fraction of the way to the target after dt with rate k (1/s), independent of the frame rate
function CM.expAlpha(k, dt) if k <= 0 then return 0 end return 1 - exp(-k * math.max(dt, 0)) end
function CM.approach(cur, target, k, dt) return cur + (target - cur) * CM.expAlpha(k, dt) end
-- critically damped spring (smooth time ~ time to reach the target), returns value, velocity
function CM.smoothDamp(cur, target, vel, smoothTime, dt)
  smoothTime = math.max(smoothTime, 1e-4)
  if dt <= 0 then return cur, vel end
  local omega = 2 / smoothTime
  local x = omega * dt
  local e = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x)
  local change = cur - target
  local temp = (vel + omega * change) * dt
  local newVel = (vel - omega * temp) * e
  local out = target + (change + temp) * e
  -- no overshoot
  if (target - cur > 0) == (out > target) then out, newVel = target, 0 end
  return out, newVel
end

-- ================================================================ vectors and quaternions
local function v3(x, y, z) return { x = x or 0, y = y or 0, z = z or 0 } end
CM.v3 = v3
function CM.add(a, b) return v3(a.x + b.x, a.y + b.y, a.z + b.z) end
function CM.sub(a, b) return v3(a.x - b.x, a.y - b.y, a.z - b.z) end
function CM.scale(a, s) return v3(a.x * s, a.y * s, a.z * s) end
function CM.dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
function CM.len(a) return sqrt(a.x * a.x + a.y * a.y + a.z * a.z) end
function CM.dist(a, b) return CM.len(CM.sub(a, b)) end
function CM.norm(a)
  local l = CM.len(a)
  if l < 1e-9 then return v3(0, 0, 0), 0 end
  return v3(a.x / l, a.y / l, a.z / l), l
end
function CM.vlerp(a, b, t) return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t) end

local function q(i, j, k, r) return { i = i, j = j, k = k, r = r } end
CM.q = q
CM.QID = q(0, 0, 0, 1)
function CM.qmul(a, b)
  return q(a.r * b.i + a.i * b.r + a.j * b.k - a.k * b.j,
           a.r * b.j - a.i * b.k + a.j * b.r + a.k * b.i,
           a.r * b.k + a.i * b.j - a.j * b.i + a.k * b.r,
           a.r * b.r - a.i * b.i - a.j * b.j - a.k * b.k)
end
function CM.qconj(a) return q(-a.i, -a.j, -a.k, a.r) end
function CM.qnorm(a)
  local l = sqrt(a.i * a.i + a.j * a.j + a.k * a.k + a.r * a.r)
  if l < 1e-9 then return q(0, 0, 0, 1) end
  return q(a.i / l, a.j / l, a.k / l, a.r / l)
end
function CM.qrot(a, v)
  local p = CM.qmul(CM.qmul(a, q(v.x, v.y, v.z, 0)), CM.qconj(a))
  return v3(p.i, p.j, p.k)
end
-- yaw about +Z, then pitch about the camera right axis (+X), angles in degrees
function CM.qFromYawPitch(yawDeg, pitchDeg)
  local hy, hp = rad(yawDeg) * 0.5, rad(pitchDeg) * 0.5
  return CM.qmul(q(0, 0, sin(hy), cos(hy)), q(sin(hp), 0, 0, cos(hp)))
end
function CM.forward(yawDeg, pitchDeg)
  local y, p = rad(yawDeg), rad(pitchDeg)
  return v3(-sin(y) * cos(p), cos(y) * cos(p), sin(p))
end
-- forward vector -> yaw, pitch (degrees); nil for a degenerate vector
function CM.yawPitchFromForward(f)
  local n, l = CM.norm(f)
  if l < 1e-6 then return nil end
  return deg(atan2(-n.x, n.y)), deg(asin(clamp(n.z, -1, 1)))
end
-- look from the eye to a world point -> yaw, pitch (degrees, conventions of CM.forward); nil when the point
-- is the eye (the game's look aimed at the crosshair point P, not parallel to the camera forward: parallax)
function CM.anglesEyeToPoint(eye, point) return CM.yawPitchFromForward(CM.sub(point, eye)) end
-- one step of the parallax correction of the written game look. st = { dy, dp } (correction now,
-- deg), camYaw / camPitch = the camera, yaw / pitch = eye -> P (nil = no correction: back to the camera). The correction
-- toward eye -> P is limited per axis to cfg.aimLookMaxDeg (the yaw of a near-vertical direction swings by 45-130 deg for
-- a small angle, so the axes are limited, not the angle) and moves at <= cfg.aimLookRate deg/s (0 = at once; exact once
-- reached). Returns the yaw / pitch to write (pitch within aimPitchMin..aimPitchMax).
function CM.lookParallaxStep(st, cfg, dt, camYaw, camPitch, yaw, pitch)
  local mx = math.max(tonumber(cfg.aimLookMaxDeg) or 10, 0)
  local ty, tp = 0, 0
  if yaw ~= nil and pitch ~= nil then
    ty = clamp(CM.wrapDeg(yaw - camYaw), -mx, mx)
    tp = clamp(pitch - camPitch, -mx, mx)
  end
  local s = (tonumber(cfg.aimLookRate) or 0) * math.max(dt or 0, 0)
  local function mv(cur, tgt)
    cur = cur or 0
    if s <= 0 or abs(tgt - cur) <= s then return tgt end
    return tgt > cur and cur + s or cur - s
  end
  st.dy, st.dp = mv(st.dy, ty), mv(st.dp, tp)
  return CM.wrapDeg(camYaw + st.dy), clamp(camPitch + st.dp, cfg.aimPitchMin or -89, cfg.aimPitchMax or 89)
end
-- aim point feed (barrel pitch = bullet line): the shot is a hitscan along the camera ray, so the native aim /
-- camera targets get P = the first hit of the camera forward ray (camera + fwd * d), not a point 60 m along the camera /
-- eye ray. No native signature change: the DLL builds Q = org + norm(fwd) * 60 m (aimofs::Params::targetDistance), so
-- org = P - fwd * targetDist gives Q = P. Internal constants, no setting. enabled = false (or no P) = the targets exactly
-- as before. d is filtered as 1/e, e = d - (eye ahead of the camera along fwd): angles seen from V's body change about
-- linearly in 1/e, so the rate limit bounds the per-frame aim step at a near-wall edge (60 m <-> the clamp).
-- Shipped OFF with P alone the open-loop motion aim offset aims from the fixed chest at a near P (large
-- spine yaw / pitch at a wall). Enable only in the milestone that wires the DLL barrel loop (MotionPose barrelLoop), the
-- aimPointVisual seam in MotionDriver and the step 0 muzzle axis data.
CM.AIM_POINT = {
  enabled = false,
  targetDist = 60,   -- m, must equal aimofs::Params::targetDistance (src/mm/AimOffset.hpp; no native setter)
  minDist = 2.0,     -- m along the camera ray; P is also >= cfg aimLookMinAhead m in front of the eye
  -- m past the eye plane: the near threshold shared with the DLL loop (src/mm/BarrelPitchLoop.hpp kBarrelMinTargetDist 1.0 m
  -- muzzle -> P, the muzzle sits <= ~1 m past the eye plane): P never enters the band where the loop fades to the open loop
  -- (MotionPoseTest 10 mirrors this clamp and checks muzzle -> P >= 1.2 * kBarrelMinTargetDist)
  minPastEye = 2.5,
  smoothHz = 12,     -- 1/e low-pass
  invRate = 1.0,     -- 1/m per s: max change of 1/e (60 m -> 2 m edge in ~0.5 s, <= 1 deg per frame at 60 fps)
  rayHz = 30,        -- own camera ray at most this often (a fresh parallax look ray past the start is reused)
  rayPastEye = 0.5,  -- m: the own ray starts this far past the eye plane (never on V's own body)
  hold = 0.5,        -- s: the last ray result is reused this long without a new one, then no P (the old targets)
}
-- st.p = world P of the last step with a result (set by the caller), st.d = its distance along the camera ray
function CM.newAimPoint() return { x = nil, raw = nil, rawAt = nil, rayAt = nil, lookAt = nil, ahead = nil, d = nil, p = nil } end
function CM.aimPointReset(st)
  st.x, st.raw, st.rawAt, st.rayAt, st.lookAt, st.ahead, st.d, st.p = nil, nil, nil, nil, nil, nil, nil, nil
end
-- one filter step. raw = distance of P along the camera ray measured this frame (nil = none), ahead = eye ahead of the
-- camera along fwd (m, >= 0), minAhead = cfg aimLookMinAhead, farD = the far end (a miss). Returns d (m along the camera
-- ray) or nil (no P: never measured, or the last result older than ap.hold and glided out). A new filter starts at the far
-- end (= the old target) and glides in; settled on a miss it returns farD exactly. A result older than ap.hold counts as a
-- miss: P glides to farD at the same rate limit and only then is the state cleared (nil), no one-frame target jump.
function CM.aimPointStep(st, ap, dt, clock, raw, ahead, minAhead, farD)
  local expired = false
  if raw ~= nil then st.raw, st.rawAt = raw, clock
  elseif st.raw == nil or st.x == nil then
    CM.aimPointReset(st)
    return nil
  elseif clock - (st.rawAt or -1e9) > ap.hold + 1e-6 then
    st.raw, expired = farD, true
  end
  local hiE = math.max(farD - ahead, 1e-3)
  local loE = math.min(math.max(ap.minDist - ahead, minAhead or 0, ap.minPastEye or 0, 1e-3), hiE)
  local xt = 1 / clamp(st.raw - ahead, loE, hiE)
  if st.x == nil then st.x = 1 / hiE end
  if abs(xt - st.x) < 1e-9 then st.x = xt
  else
    local lim = ap.invRate * math.max(dt or 0, 0)
    st.x = st.x + clamp((xt - st.x) * CM.expAlpha(ap.smoothHz, dt or 0), -lim, lim)
  end
  st.x = clamp(st.x, 1 / hiE, 1 / loE)
  if st.x == xt and st.raw >= farD then
    if expired then -- glided out to the far end (= the old target): no P from now on
      CM.aimPointReset(st)
      return nil
    end
    st.d = farD
  else st.d = ahead + 1 / st.x end
  return st.d
end
-- native target ray: d = nil -> (world, fwd) unchanged; d -> org = P - fwd * targetDist with P = world + fwd * d
function CM.aimPointRay(world, fwd, d, ap)
  if d == nil then return world, fwd end
  return CM.add(world, CM.scale(fwd, d - (ap or CM.AIM_POINT).targetDist)), fwd
end
-- ADS target (UTPPSetAimTarget) org / fwd: P (d) when known; else as before - the camera ray, or with a
-- parallax corrected game look written last frame (lwPrev) 60 m along that corrected direction from the eye
function CM.aimTargetRay(res, lwPrev, eye, d, ap)
  local fwd = CM.forward(res.yaw, res.pitch)
  local org = res.world
  if d ~= nil then return CM.aimPointRay(org, fwd, d, ap) end
  if lwPrev and eye then
    local dy = CM.wrapDeg(lwPrev.yaw - (lwPrev.camYaw or lwPrev.yaw))
    local dp = lwPrev.pitch - (lwPrev.camPitch or lwPrev.pitch)
    if math.abs(dy) > 1e-6 or math.abs(dp) > 1e-6 then
      fwd = CM.forward(res.yaw + dy, CM.clamp(res.pitch + dp, -89, 89))
      org = eye
    end
  end
  return org, fwd
end
-- world camera transform -> entity space (player root): pos/rot relative to the player position/orientation
function CM.toLocal(playerPos, playerQ, worldPos, worldQ)
  local inv = CM.qconj(CM.qnorm(playerQ))
  return CM.qrot(inv, CM.sub(worldPos, playerPos)), CM.qnorm(CM.qmul(inv, worldQ))
end
-- Matrix with X/Y/Z/W rows (RED4 layout: basis rows, translation in W) or columns -> forward (Y axis), position
function CM.matrixForward(m, layout)
  if layout == "cols" then return v3(m.X.y, m.Y.y, m.Z.y) end
  return v3(m.Y.x, m.Y.y, m.Y.z)
end
function CM.matrixPosition(m, layout)
  if layout == "cols" then return v3(m.X.w, m.Y.w, m.Z.w) end
  return v3(m.W.x, m.W.y, m.W.z)
end

-- ================================================================ defaults (own values, tuned in game)
CM.DEFAULTS = {
  -- input
  degPerMouseUnit = 0.1,   -- until calibrated from the game's own body rotation (see Calibrate)
  sens = 1.0, invertX = false, invertY = false,
  padDegPerSec = 170, padPitchScale = 0.7, padDeadzone = 0.12,
  pitchMin = -65, pitchMax = 55,
  -- look modes
  pitchSource = "fpp",     -- "fpp": pitch of the FPP camera (aim stays aligned); "input": own integration
  idleOrbit = true, idleDelay = 0.35, idleSpeed = 0.25, -- s without move input and m/s: orbit without turning V
  recenterRate = 5.0,      -- 1/s: free-look yaw offset back to 0 when following the body
  pitchRecoverRate = 10.0, pitchSnapDeg = 0.5,
  -- follow-mode yaw limiter: the mouse part of V's turn is applied at once (aim stays aligned), the rest (body
  -- turned by the game / motion matching, recentering after free look) at <= followMaxRate deg/s, speed rising over
  -- followEase s (ease-in), approach gain followGain 1/s (ease-out); 0 = off (old rigid follow)
  followMaxRate = 240, followEase = 0.25, followGain = 12,
  -- body flip: > flipDeg of body turn without the mouse within flipWindow s -> world yaw held flipHold s
  followHoldOnFlip = true, flipDeg = 120, flipWindow = 0.3, flipHold = 0.6,
  teleportDist = 3.0,      -- m per frame: player moved by the game (teleport) -> follow yaw snaps
  -- pivot
  -- "capsule" = player position + stance height (the FPP camera / head follows the FPP animation set, which looks
  -- like a crouch in melee / block / vehicle exits); "fpp" = eye height of the FPP camera
  pivotSource = "capsule", standHeight = 1.55, crouchHeight = 1.05, stanceBlend = 0.25,
  -- the body's Stand_to_Crouch clip (motion set) drops the hips 5 cm at 0.2 s, 50 % at 0.43 s, 90 % at 0.57 s
  -- (Crouch_to_Stand: 50 % at 0.24 s), so the 0.25 s pivot blend from the PSM edge put the camera at the crouch height 0.4 s before the
  -- body (measured, 21 in-place crouches). A CLEAN in-place edge (CM.stanceFilterStep bypass at standstill, speed < stanceFastSpeed) uses
  -- a delay + blend that matches the clip: 50 % of the pivot move at crouchDelay + crouchBlend / 2 = 0.40 s (stand: 0.225 s). Every other
  -- change (walking, filtered, slide exit, CrouchSprint 30) keeps stanceBlend.
  crouchDelay = 0.20, crouchBlend = 0.40, standDelay = 0.05, standBlend = 0.35, stanceFastSpeed = 0.5,
  -- the camera stance goes through the port of the DLL stance filter (adapter::FilteredStance,
  -- MotionAdapter.hpp): a change is accepted after stanceFilter s (a 0.017 s CrouchSprint 30 <-> Crouch 3 flicker of the PSM must not pump the
  -- pivot: 37 switches in 4 s without it), except a CLEAN edge (details Stand 1 / AimWalk 2 / Crouch 3 on both frames, the old value held
  -- stanceCleanHold s, no slide / slide-exit window of stanceExitMap s, a crouch ENTRY slower than stanceCleanMaxSpeed m/s) which passes at
  -- once. Same values as adapter::kStanceFilterSeconds / kStanceCleanHoldSeconds / kStanceCleanMaxSpeedMs / MapConfig::slideExitMapS.
  -- stanceFilter = 0 switches the filter off (the pre-fix camera, also for CrouchSprint 30 = stand).
  stanceFilter = 0.15, stanceCleanEdge = true, stanceCleanHold = 0.15, stanceCleanMaxSpeed = 3.0, stanceExitMap = 0.6,
  -- slide stance (PSM Locomotion Slide + LocomotionDetailed Slide 5). FK of the final pose (root-
  -- relative z, seated slide clips 456 / 457): hips 0.17, neck 0.52-0.57, head 0.62-0.66 and the head 0.6 m BEHIND the root
  -- (standing: hips 1.00, neck 1.50, head 1.64; crouch clip 173: hips 0.47, neck 0.94, head 1.06). The crouch pivot 1.05 m left
  -- the sliding V's head ~29 deg below the aim camera's view centre (FOV 55, 1.58 m behind) = out of the frame; the pivot sits
  -- ~0.08 m above the neck in the other stances. slideHeight >= crouchHeight = the old behaviour (slide = crouch pivot).
  -- slideHeight applies in proportion to the aim weight (CM.pivotStep aimW): the free camera (2.8 m boom) frames the slide
  -- well at the crouch pivot and at 0.62 m it sits 0.35 m over the floor - the floor ray pushes it
  -- in to 0.85 m from V's head when looking up 10 deg.
  -- slideBlend: pivot blend time into / out of the slide - 0.93 m stand -> slide at ~2.8 m/s peak (the crouch's is 3 m/s). Leaving
  -- the slide is one monotonic blend of slideBlend s from the pivot of the moment to the stance height, also after a short slide.
  -- slideArmLimit: the aimed slide's boom pitch is limited so the boom end stays over the floor (CM.solve; the 0.62 m pivot looking up)
  slideHeight = 0.62, slideBlend = 0.5, slideArmLimit = true,
  pivotSmooth = 0.12,      -- s: vertical pivot spring (stairs, crouch)
  -- a jump out of a slide dropped the pivot to 0.73 .. 0.85 m over the capsule (below the
  -- crouch pivot 1.05 m) for 0.4 s while the body stood up and rose: the 0.12 s spring of the base lags the take-off (~0.30 m at 5.5 m/s) and
  -- the 0.5 s exit blend of the slide pivot starts from the slide height. The exit blend INTO THE AIR (CM.pivotStep air) takes
  -- slideAirExitBlend s and the base spring slideAirExitSmooth s while it runs; a jump from the ground and the exit blend on the ground are untouched.
  slideAirExitBlend = 0.15, slideAirExitSmooth = 0.03,
  -- the base spring goes back from slideAirExitSmooth to pivotSmooth over slideAirExitRamp s after that exit blend (a step back at once let the lag
  -- of the still rising capsule grow again: the pivot squatted 0.10 - 0.11 m 0.1 s after the take-off with the body standing)
  slideAirExitRamp = 0.50,
  eyeFallback = 1.6,       -- m above the player origin when the FPP camera cannot be read
  eyeMin = 0.25, eyeMax = 2.3, pivotSnap = 1.0,
  -- fast fall (car jump-outs): the base spring lags V by about v * pivotSmooth (1 m at 8.3 m/s), and the
  -- old snap (lag over pivotSnap -> base = V, vb = 0) made a 1 m camera step on the way up and 1 - 3 times in the last 0.25 s before
  -- the touchdown, then 0.5 - 0.8 m of extra sink after it. Now (capsule pivot only):
  --  - a teleport is V's OWN one-frame z step over pivotSnap (not the accumulated lag), and either over teleportDist or faster than
  --    pivotSnapSpeed m/s (a 30 m/s fall at 30 fps is 1 m per frame, not a teleport): base = V, vb = 0;
  --  - the lag is bounded continuously: over pivotLagMax the base is clamped to V +- pivotLagMax with vb = V's vertical velocity;
  --  - fast mode (pivotFast): while |V's vertical speed| is over pivotFastVz m/s for pivotFastArm s (at least 2 frames; a one-frame
  --    stair / mount step never arms it) the base moves with V (velocity feed-forward) and only the offset to V decays, with the
  --    pivotFastSmooth s spring - no lag builds up, so there is nothing to catch up at the touchdown. After the speed fell below
  --    pivotFastVz for pivotFastArm s the spring goes back to pivotSmooth over pivotFastRamp s. pivotFast = false: the old rule exactly (snap to V when the base is over pivotSnap away, plain spring).
  pivotLagMax = 1.0, pivotSnapSpeed = 50, pivotFast = true, pivotFastVz = 3.0, pivotFastArm = 0.03, pivotFastSmooth = 0.08, pivotFastRamp = 0.2,
  -- jump-out landing (car jump-out: the rigid fast-fall follow stopped the camera dead in one frame at the
  -- touchdown, 250 - 1400 m/s^2). CM.jumpOutStep, only while the vehicle exit hold is on (pivotStep jo.on) and until it settled: a separate
  -- latched mode that replaces the fast-fall mode and the lag clamp. The base is an acceleration limited follower of V: speed command
  -- vd = V's speed + (V - base) / jumpOutTp, acceleration command (vd - u) / max(jumpOutTa, dt) limited to jumpOutAmax (jumpOutAup on the way up),
  -- smoothed over jumpOutTa s. A ground estimate (down ray from V's feet, only while V falls faster than jumpOutVth m/s, hit not more than
  -- jumpOutEps m above the feet, at most jumpOutRayLen m down) limits the fall speed to the one the base can still stop with jumpOutAbrake
  -- m/s^2; the stopping guard brakes directly (jumpOutGuard * jumpOutAbrake and over) when the smoothed acceleration lags. No ray or no hit:
  -- a critically damped spring of jumpOutTfb s on the descent. The base may be up to jumpOutLagMax m from V (the pivotLagMax clamp is not
  -- used meanwhile). The mode ends when the input is off and the base is within jumpOutEndDist m of V with a speed under jumpOutEndSpeed
  -- m/s. Failsafe: jumpOutHoldMax s after the input went off it ends anyway, but only with V's vertical speed under jumpOutVth and the
  -- base within pivotLagMax of V (the normal path takes over with the speed carried and nothing to clamp); a fall that lasts longer keeps the
  -- mode, and jumpOutHoldHard s after the input went off it hands over with the lag clamped to pivotLagMax. The follower is integrated in
  -- sub-steps of at most jumpOutSubStep s (a long frame is stable), at most jumpOutDtMax s of a frame are integrated.
  -- The price: after the touchdown the base sinks on for about 0.3 - 0.5 m (0.3 s).
  -- jumpOutGov = false: the previous behaviour exactly.
  jumpOutGov = true, jumpOutTp = 0.10, jumpOutTa = 0.035, jumpOutAmax = 60, jumpOutAup = 80, jumpOutAbrake = 35, jumpOutGuard = 0.8,
  jumpOutTfb = 0.11, jumpOutVth = 3.0, jumpOutEps = 0.10, jumpOutRayLen = 8, jumpOutLagMax = 2.0, jumpOutEndDist = 0.01, jumpOutEndSpeed = 0.05, jumpOutHoldMax = 3.0,
  jumpOutHoldHard = 10.0, jumpOutSubStep = 1 / 30, jumpOutDtMax = 0.5,
  -- launch (both jump-outs): V leaves at 9.7 m/s within one frame, the base starts 0.85 m behind it and the
  -- catch-up command (V - base) / jumpOutTp asked for V's speed + 8.5 m/s. The acceleration limits could not brake that in time: the pivot
  -- rose at 13.5 m/s while V rose at 6 m/s, overshot V by 0.6 m (pivot offset +0.70 -> +2.18 -> +1.58) and braked at -50 m/s^2.
  -- jumpOutCatch (m/s, 0 = off, the previous command exactly): while V rises faster than jumpOutVth the closing speed of the base
  -- towards V (V above the base) is limited to it, so the catch-up decays without overshoot.
  jumpOutCatch = 3.0,
  -- landing brake (both jump-outs; offline replay of the same jumps): the base fell with V (-16 m/s2) and the stopping
  -- guard set its acceleration to the needed 31 m/s2 in ONE frame (+47 m/s2 step, 2800 m/s3 at 60 fps, 6400 at 144 fps), held 30 - 35 m/s2 for 0.3 s.
  -- jumpOutJerk (m/s3, 0 = off, everything below off and the previous guard exactly): once a ground estimate exists and a stop on it with a
  -- deceleration plateau of jumpOutDecel m/s2 needs the whole remaining distance (CM.stopDist), the base brakes along the jerk limited profile that
  -- stops it exactly on the ground (CM.stopPeak, solved again every sub-step); the acceleration changes by at most jumpOutJerk * dt per step. The
  -- stop starts earlier and is spread over a longer time with a lower peak (about jumpOutDecel instead of 33 - 35 m/s2); the price is a higher
  -- pivot over V at the touchdown (0.6 - 0.9 m instead of 0.4 m) that sinks in about 0.3 s. The 35 m/s2 envelope and the stopping guard stay the outer
  -- safety bound. jumpOutDrop (m): the ground estimate fell by more than that (the first hit was a car roof) or V fell through it: the brake switches
  -- to jumpOutRecJerk / jumpOutRecDecel for the rest of the fall (0 = the guard of the previous behaviour). Without a ground estimate (no ray) nothing changes.
  jumpOutJerk = 300, jumpOutDecel = 18, jumpOutDrop = 0.3, jumpOutRecJerk = 900, jumpOutRecDecel = 26,
  -- (review): the latched landing brake respects V. V calm (vertical speed over -jumpOutCalmV m/s) for jumpOutCalmT s and more than jumpOutLandMargin m above
  -- the ground estimate: V rising (over jumpOutCalmV) = a new jump, the estimate is dropped and the follower takes over, V standing on a higher surface (a curb / sill / roof
  -- edge the centre ray missed) = the estimate becomes V's height and the recovery brake stops the base there. After either the acceleration changes at jumpOutRetJerk m/s3
  -- at most. jumpOutRetarget = false: the previous behaviour exactly.
  jumpOutRetarget = true, jumpOutCalmV = 1.5, jumpOutCalmT = 0.04, jumpOutLandMargin = 0.08, jumpOutRetJerk = 900,
  -- (the moving jump-out): jumpOutDropLatched = true: a fall of the ground estimate by more than jumpOutDrop m switches to the recovery brake only when
  -- a landing brake is latched for the old estimate. The first hit under V was the car she had just left (0.58 m higher than the ground, no brake latched yet) and the whole landing ran on
  -- the recovery brake (jerk 900, deceleration 26; measured 27 - 34 m/s2). false = the previous behaviour exactly.
  jumpOutDropLatched = true,
  -- jump-out pivot xy (the stationary jump-out: V was pushed 1.7 m sideways at up to 6.6 m/s right after a false
  -- ground contact and the pivot copied her xy one to one, a sideways camera kick; the moving one stopped from 3.8 m/s in 0.1 s the same way).
  -- jumpOutXY (CM.jumpOutXYStep, while the jump-out mode of the pivot runs): the pivot xy is a critically damped follower of V (jumpOutXYTime s) with a
  -- low passed copy of V's velocity (jumpOutXYVel s) as feed forward - no lag at a steady speed, a velocity step is spread over about jumpOutXYTime s.
  -- The offset to V is meant to stay inside jumpOutXYLag m; when a stop at jumpOutXYAmax m/s^2 would end beyond it, its growth is braked at that
  -- deceleration (it may pass jumpOutXYLag by the braking distance, at most to jumpOutXYLagHard m, where a clamp removes the part of the relative speed
  -- that would grow it). When the mode ends the follower keeps running (the same equations, no change of dynamics) until it sat on V (0.1 mm, 1 cm/s).
  -- A step of V larger than teleportDist (or faster than pivotSnapSpeed) is a teleport: the state is dropped.
  -- Physical floor: a stop from a relative speed v inside a lag L needs v^2 / (2 L) (21 m/s^2 from 3.8 m/s with 0.35 m, 62 from 6.6): faster stops
  -- run past the lag instead (8 m/s in 0.06 s: the pivot runs 0.55 m past V at 65 m/s^2; the old copy stopped at 133 m/s^2).
  -- jumpOutXY = false: the pivot xy is V's xy, exactly as before.
  jumpOutXY = true, jumpOutXYTime = 0.25, jumpOutXYVel = 0.05, jumpOutXYLag = 0.35, jumpOutXYAmax = 40, jumpOutXYLagHard = 1.0,
  -- jump-out landing framing: the landing brake has to stop the pivot base
  -- on the ground without passing V's stance, so it decelerates 0.2 - 0.3 s before the touchdown; V falls on at 10 - 14 m/s and stops dead. At the touchdown the
  -- base was 0.70 m above V (0.43 m) with the look pitch unchanged: V's chest dropped 13.6 deg (9.2) below its place in the frame within about 0.15 s and
  -- came back as the base sank (0.3 s) - a bounce of V in the frame. A smaller gap needs a harder stop (about v^2 / 8a: 0.25 m only at 70 m/s^2).
  -- jumpOutFrame (CM.jumpOutFrameStep, CM.solve): while the jump-out mode of the pivot runs and V fell faster than jumpOutVth m/s, the shown orientation (not the
  -- boom, not the look pitch) tilts by the angle between V's chest (V + jumpOutFrameHeight m) and the same point over the pivot base (where it sits when the
  -- base is on V), seen from the camera. The target looks jumpOutFrameLead s ahead with the ground estimate (V's fall - low passed velocity over
  -- jumpOutFrameVelT s, gravity jumpOutFrameGrav m/s^2 - and the base's own speed and braking, both floored at the ground), so the tilt peaks at the touchdown instead of
  -- lagging behind it. The tilt follows the target at <= jumpOutFrameRate deg/s away from 0 (jumpOutFrameRateBack towards 0: after the touchdown),
  -- jumpOutFrameAccel deg/s^2 and a gap closing gain of jumpOutFrameGain 1/s (sub-steps of 1/120 s); after the mode it eases back to 0. Scaled out by the aim
  -- weight and the FPP transition. No ground estimate (no ray hit): the target is the present angle (no look ahead). jumpOutFrame = false: the previous behaviour exactly.
  jumpOutFrame = true, jumpOutFrameHeight = 1.3, jumpOutFrameLead = 0.05, jumpOutFrameVelT = 0.03, jumpOutFrameGrav = 15, jumpOutFrameRate = 90,
  jumpOutFrameRateBack = 55, jumpOutFrameAccel = 2000, jumpOutFrameGain = 20,
  -- mount pivot continuity (bike mount): the
  -- game's mount puts V on the seat slot in one frame (about 1 m and 75 deg, +0.68 m z) and the pivot xy is the raw player
  -- position, so the camera cut with it. Inside mountBlendWindow s after the camera first sees V mounted, a planar step of
  -- more than mountBlendMinStep m (and at most teleportDist) in one frame starts an xy offset from the last shown pivot to
  -- the live one that fades with smoothstep over mountBlendTime s (CM.mountShiftStep). z keeps its own spring; the camera
  -- yaw is not touched (it keeps the world yaw through the mount). mountBlend = false = the previous behaviour.
  mountBlend = true, mountBlendTime = 0.38, mountBlendMinStep = 0.30, mountBlendWindow = 0.5,
  -- the mount camera watch of init.lua M.MountShiftGate (one line per mount: shortest boom, nearest camera distance to V, largest camera step)
  mountCamWindow = 1.5, mountCamNear = 1.0, mountCamSpan = 0.1,
  vehHoldColRate = 12.0, -- m/s: pull-in rate limit of the collision boom while a vehicle mount hold still shows the orbit (0 = instant)
  -- ('behind' approach): the front-left door swinging open crossed the boom for about 0.25 s
  -- with the vehicle standing; the boom (inRate path above) went 2.93 -> 2.16 -> 1.27 m within 0.15 s and back (blocked=0, hits=1). While that path is
  -- active a pull-in starts only after the boom was pressed for vehHoldColDelay s (a door passing through is not an occluder; a leaky timer, it
  -- empties at twice the rate it fills, and only once the hit is gone: while the occluder is still there the timer is kept) and ramps up with
  -- vehHoldColAccel m/s^2 up to the rate above. 0 = the previous behaviour exactly.
  -- init.lua passes vehHoldColDelay as colInDelay together with colInRate, only while the pivot has been below vehHoldColQuietSpeed m/s AND the camera
  -- (yaw / pitch of the solve) below vehHoldColQuietRot deg/s for vehHoldColQuiet s (a hit that comes with a moving pivot or a turning camera is an
  -- occluder: V sliding in next to a body, the 1.15 m case, or the mouse swinging the boom into the car body: both keep the immediate pull-in;
  -- the door swings while V and the camera are still). vehHoldColQuietRot <= 0: the camera rotation is not measured (the first version).
  vehHoldColDelay = 0.35, vehHoldColAccel = 30, vehHoldColQuiet = 0.3, vehHoldColQuietSpeed = 0.7, vehHoldColQuietRot = 6,
  -- ('behind' mount): the standing car's body between V and the held orbit (the camera on the far side of the cab)
  -- pulled the boom in to 0.88 m from V's head within 0.2 s (hits=1, 2.93 -> 1.68 -> 0.89 m) and held it there for 0.9 s (V's face filled the frame; the
  -- pull-out then started from that pose and crossed V). vehHoldColMin (m from the pivot, 0 = off, the previous behaviour exactly): while the rate limited
  -- pull-in path of the mount hold is active (the same gate as vehHoldColRate) the boom is not pulled in below this length (a boom that is already shorter
  -- stays as it is); the camera looks through the car body for the rest of the hold instead of coming up to V's head.
  vehHoldColMin = 1.6,
  -- when the gate of the rate limited pull-in drops while the orbit is still shown and the occluder remains (an aborted / cancelled mount,
  -- HoldForVehicle(false)), the boom used to take the instant branch and snapped from the floor up to ~0.9 m in one frame. vehHoldColCarry (false = that
  -- behaviour): the pull-in goes on at the last rate (no delay, no floor) until the boom has reached the allowed length, then the instant pull-in is back.
  vehHoldColCarry = true,
  -- (car mounts): the rate limited pull-in of the mount hold ran at the full
  -- vehHoldColRate (12 m/s = 1.2 m per 0.1 s) whenever the quiet gate was not open (V had walked up at 0.8 m/s a moment before, or slid into the seat: the
  -- delayed, ramped path above is only taken at rest). The boom came in 2.93 -> 1.60 m within 0.3 s (0.69 / 0.80 / 1.19 m per 0.1 s relative to V). Both
  -- gate paths are now capped at vehHoldColMountRate m/s (0 = the previous rate, vehHoldColRate only); 2.4 m/s = 0.24 m per 0.1 s, under mountBlendMinStep.
  -- vehHoldColLift (m/s, 0 = the previous behaviour): on the same gate the boom also LENGTHENS at that rate at most (a returning door, a lifted occluder), and a boom
  -- that is shorter than the floor vehHoldColMin when the hold starts (V had walked up to the car: 0.73 m) rises to the floor at that rate
  -- instead of keeping its length - the camera is not closer than the floor to V from the first frames of the hold on (it looks through the car body, as at the floor).
  vehHoldColMountRate = 2.4, vehHoldColLift = 2.4,
  -- the mount camera watch goes on through the pull-out override (shown pose against V): WARNING for a camera closer than mountCamNear to V at any
  -- time, for a jump at the hand-over from the orbit to the override above mountBlendMinStep, and for a move above mountPullStep m per mountCamSpan s inside the
  -- pull-out (its designed peak is about 7 m/s = 0.7 m per 0.1 s). mountPullWatch = false: the watch ends when the override takes the view.
  mountPullWatch = true, mountPullStep = 1.0,
  -- collision
  -- (camera inside V's mesh: against a wall the boom went down to hardMin 0.08 m, inside the head; minArm was
  -- unused): minArm = the body clearance, m from the pivot (capsule + standHeight, ~8 cm over the neck; CM.clearance =
  -- max(hardMin, minArm)). CM.collisionLength floors the boom at the clearance, but never behind a surface: with a raw hit
  -- nearer than clearance + clearanceWallGap the floor is that hit - clearanceWallGap (at least hardMin, the old floor).
  -- res.inside = the placed boom is shorter than the clearance; res.insideWall = because a surface shortened it (not only the
  -- blend); utpp_camera: vanilla near-camera head hide while inside (CM.solve, init.lua M.InsideHideStep).
  -- minArm <= hardMin = the previous floor. Hits farther than clearance + margin: unchanged.
  -- clearanceBlend: the enter / exit blend scales the boom from the clearance (w = 0: pivot + boom direction * clearance)
  -- instead of starting at the FPP eye point inside the TPP head (CM.solve); orientation and FOV still blend from the FPP
  -- view. false = the previous blend from the eye point. w = 1: identical either way. Default false (opt-in): on,
  -- the camera position jumps ~0.35 m at both blend ends (eye <-> clearance point) - every scoped-ADS return goes through them.
  collision = true, radius = 0.18, margin = 0.12, minArm = 0.3, hardMin = 0.08,
  clearanceWallGap = 0.06, clearanceBlend = false,
  returnTime = 0.3, deadzone = 0.015, fullEvery = 3, fullHold = 0.6,
  -- transitions
  enterTime = 0.45, exitTime = 0.35, repWait = 0.15, repWaitMax = 1.0,
  shoulderTime = 0.25, fovRate = 8.0, profileBlend = 0.5,
  zoomStep = 0.25, zoomMin = -1.0, zoomMax = 2.0, distMin = 0.6, distMax = 6.0,
  -- aim (over the shoulder while PSM UpperBody = Aim with a ranged weapon, perspective.aiming = tpp)
  aimEnabled = true, aimDistance = 1.1, -- (a bit further): was 0.8
  aimSide = 0.45, aimFov = 42, aimHeight = 0.05, aimAutoSwap = true,
  aimBlendIn = 0.12, aimBlendOut = 0.20,
  aimSwapWall = 0.5, aimSwapHold = 0.3, -- m / s: shoulder-side wall within aimSwapWall for aimSwapHold s -> swap
  crouchPitchMax = 30, -- orbit look-up cap while crouched (camera under V otherwise)
  aimOwnYaw = true,     -- aim / unarmed zoom yaw = own camera yaw (V is turned onto it), not the game's look
  aimZoomFov = false,   -- no FOV zoom when aiming without a scope (the profile FOV stays)
  aimPitchMin = -85, aimPitchMax = 85, -- was -70/70 (camera and shot disagreed at the extremes; the partial boom below keeps the camera sane)  -- wider than the orbit range: the camera forward is the aim direction
  aimBoomPitchScale = 0.35, -- (aiming up / down a drama): weapon aim boom position orbits with this fraction of the pitch (camera.aim.boomPitchScale; 1 = full orbit, old)
  -- framing limit (camera.aim.boomFrame): with the partial boom the pivot (V's shoulder line) stays within this
  -- fraction of half the vertical FOV from the screen centre; 0 = no limit (pure partial orbit). (at 0.8 the
  -- boom followed the look 1:1 past about -35 / +30 - camera 0.98 m over / under the pivot at +-85, on the floor crouched
  -- looking up): default 0 (opt-in); V's shoulder may leave the picture at extreme pitch
  aimBoomFrame = 0,
  -- (review: the framing limit at the narrow zoom FOV fell back to ~the full orbit - camera over V's
  -- head looking down): the unarmed zoom (inp.zoomOnly) has its own scale and limit, default no limit
  zoomBoomPitchScale = 0.25, zoomBoomFrame = 0, -- camera.zoomUnarmed.boomPitchScale / .boomFrame
  -- parallax-correct game look while aiming / zooming (init.lua, CM.anglesEyeToPoint): the game's look is set
  -- from the eye to the crosshair point P (camera forward ray from aimLookNear m to aimRange m, the first hit or the far
  -- end; P at least aimLookMinAhead m in front of the eye along the camera forward); the last P distance is reused for
  -- aimLookHold s when no ray is possible, then the camera yaw / pitch; aimLookParallax false = the camera yaw / pitch
  aimLookParallax = true, aimLookNear = 0.3, aimLookMinAhead = 1.0, aimLookHold = 0.5,
  -- (review: unbounded at steep pitch - a floor / ceiling hit put P beside V's eye and swung the look
  -- yaw 45-130 deg; single-frame jumps across near / far edges; every RMB release turned V by the correction): the
  -- correction (eye -> P minus the camera yaw / pitch) is limited per axis to aimLookMaxDeg and moves at <= aimLookRate
  -- deg/s (CM.lookParallaxStep); when the lock ends after a corrected write, the lock is held and the plain camera yaw /
  -- pitch written until the read-back matches it, at most aimLookFlush frames (init.lua release flush)
  aimLookMaxDeg = 10, aimLookRate = 100, aimLookFlush = 3,
  aimFovMin = 20, aimFovMax = 90,       -- accepted weapon AimFOV stat range (else aimFov)
  -- aim occlusion (level A): camera.aim.occlusion / camera.aim.range; blocked = muzzle ray hit more than aimOccTol m
  -- before P; P closer than aimOccMinAhead m in front of the muzzle = not blocked; sampled while the aim weight >
  -- aimOccMinW, at most every aimOccMinFrames frames and aimOccHz per second; edges after aimOccHold s of the same result
  aimOcclusion = true, aimRange = 60, aimOccTol = 0.15, aimOccMinAhead = 0.1, aimOccMinW = 0.5,
  aimOccHz = 30, aimOccMinFrames = 2, aimOccHold = 0.1,
  -- aim alignment onto the aim reference (game look yaw / FPP pitch): offset * aimAlignGain 1/s, at
  -- least aimAlignMin, at most aimAlignRate deg/s (CM.alignStep); the 0.12 s pose tween no longer moves the yaw
  aimAlignGain = 10, aimAlignRate = 300, aimAlignMin = 20,
  -- free / decoupled camera: the entity yaw step of the coming game tick (body turn executor) predicted from the last
  -- step (<= predictMaxStep deg per frame, CM.predictTurn) and taken out of the local transform; Locomotion inactive for less
  -- than decoupleHold s keeps the camera decoupled (flicker); mouse calibration: ratios outside calibBand x the value are
  -- rejected once calibrated (calibStreak rejections in a row re-seed), none for calibUnlockHold s after the heading lock
  -- ends (the game turns V onto its camera heading then, not the mouse)
  predictBodyTurn = true, predictMaxStep = 60, -- was 15; at 30 fps V turns 17-19 deg per frame (uncompensated = camera carried)
  decoupleHold = 0.35, calibBand = 2.0, calibStreak = 20, calibUnlockHold = 0.6,
  -- game look sync (camera.lookSync): while the heading lock is wanted the FPP camera gets the param
  -- set lookSyncSet (headingLocked + maintainWorldSpaceHeading, yaw +-180) and the camera shows the game's look; ok after
  -- lookSyncConfirm samples (look moved by the mouse while V stood / stayed while V turned), lookSyncCarried carried
  -- samples in a row -> the set is sent again (<= lookSyncResends times per lock), then broken (own yaw, logged)
  lookSync = false, lookSyncSet = "VehicleCombat", lookSyncConfirm = 2, lookSyncCarried = 3, lookSyncResends = 2,
  -- game look write (camera.lookWrite, CM.lookWriteWant): every frame while the world camera
  -- is shown in third person with the heading lock (free / aiming), the game's look (aim, shots, crosshair, Locomotion
  -- heading, AimOffset) is set to the shown camera yaw / pitch (Game.UTPPSetGameLook)
  lookWrite = true,
  -- (jump-out landing): the first game look write after a vehicle hold took the game's look pitch from 0.2 to -6.0 deg
  -- in 34 ms and to -14 deg in 0.13 s (feedback k 0.25) while the torso dropped into a deep crouch in one frame (the aim offset / spine layer reads the look
  -- pitch). lookWriteRamp (s, 0 = off, the write starts at the camera pitch exactly as before): the pitch SENT on that first write goes from the game's
  -- current look pitch to the camera pitch on a smoothstep over that time (frame time aware; the camera itself and the yaw are not touched).
  lookWriteRamp = 0.25,
  -- heading lock re-assert (init.lua lockVerify; the native game look found headingLocked false in 78 % of
  -- its ticks and wrote nothing for minutes): the PSM applies a camera parameter set with headingLocked = false at the state
  -- changes out of the slide / jump / vault (vanilla fppCameraParamSets.tweak Default / Vault / HoverJump; Slide is locked) and
  -- the cached lock was never read back. While the lock is wanted and cached the component is read every frame and set again
  -- when false - but not in vehicles / ladders / climbs / vault / hover jump / workspots / scenes / swimming / takedown /
  -- knockdowns / carrying / device control, where utpp_bodyturn.reds leaves the game's own set (init.lua lockGameOwned). At most lockReassertMax writes per lockReassertWindow s, then
  -- a pause without any (a state that resets it every frame must not make the game rotate V every frame)
  -- (lockReassertGrace: s to wait after the last game-owned frame - vault / hover jump / vehicle ... - before the first write)
  -- (lockReassertPause doubles with each pause in a row up to lockReassertPauseMax; lockReassertStable: s without a lost lock
  -- after which the backoff starts again)
  lockReassert = true, lockReassertMax = 8, lockReassertWindow = 1.0, lockReassertPause = 2.0, lockReassertGrace = 0.05,
  lockReassertPauseMax = 30.0, lockReassertStable = 10.0,
  -- (lockReassertSetHold: s an unlisted camera parameter set queued by the PSM counts as the game's before it is taken for plain)
  lockReassertSetHold = 3.0, lockReassertSetGap = 0.5, -- (the 3 s start again after a gap of lockReassertSetGap s between two reads)
  -- unarmed zoom (RMB with empty hands in third person, CM.zoomUnarmed*): steps 0..N-1 over the shoulder, the
  -- wheel step past the last one hands over to first person (the game's own Kiroshi zoom). Distance linear from
  -- zoomUnarmedDistance to zoomUnarmedLastDistance, FOV geometric from zoomUnarmedFov to zoomUnarmedLastFov (45/35/28)
  zoomUnarmedEnabled = true, zoomUnarmedSteps = 4, zoomUnarmedDistance = 0.9, zoomUnarmedLastDistance = 0.6,
  zoomUnarmedFov = 45, zoomUnarmedLastFov = 28, zoomUnarmedSide = 0.45, zoomUnarmedBlend = 0.15,
  zoomUnarmedEnterDelay = 0.1, zoomUnarmedFppHandoff = false,
  -- vehicle (internal constants, not registered settings): own pitch (vehicle hold, traversal, ownPitchHold s
  -- after the game look write turned off) and its glide cap (deg/s); the vehicle release pitch target range (deg);
  -- framing band after the unmount (V's pivot = player + vehFrameHeight m inside [vehFrameLow, vehFrameHigh] of the
  -- half vertical FOV); the first aim after a vehicle hold blends in over >= vehAimBlendIn s (within
  -- vehAimSoftWindow s after the hold)
  ownPitchHold = 0.5, ownPitchGlideRate = 120, ownPitchVehMax = 3.0, ownPitchVehMargin = 5,
  vehReleasePitchMin = -25, vehReleasePitchMax = 5,
  vehFrameHeight = 1.4, vehFrameLow = -0.35, vehFrameHigh = 0.25, vehFrameRate = 120,
  -- jump-out hold (vh.exitHold; a car also once V was seen in flight, vh.airAt): the framing correction ramps with vehFrameAccel (deg/s^2, 0 = the plain vehFrameRate clamp) up to
  -- vehFrameEaseRate (deg/s) and eases into its target (VehicleCompose); a normal exit keeps the plain clamp bit for bit unless V's z jumps by more than vehLandVz m/s in one frame
  -- after the unmount (that sets vh.airAt too) AND her pivot then leaves the framing band: only then the eased path is used on a plain car exit
  vehFrameAccel = 300, vehFrameEaseRate = 60,
  vehAimBlendIn = 0.20, vehAimSoftWindow = 1.0,
  -- jump-out landing: the release to the orbit (carry) ends the vehicle hold (high priority off, the game look write,
  -- the own-pitch phase) at a fixed time, which landed on the touchdown (hold OFF 0.15 s before it). While V is
  -- in flight (vertical speed over vehLandVz m/s, seen within the last vehLandSettle s) the hold stays on after the blend (the
  -- live orbit is shown, as through the blend) and ends vehLandSettle s after V landed, at most vehLandMax s after the blend.
  -- vehLandHold = false: the old fixed end.
  vehLandHold = true, vehLandVz = 3.0, vehLandSettle = 0.2, vehLandMax = 2.5,
  -- exit pose governor (game logs, normal exits standard3 / utility4: 0.18 - 0.32 m per frame = 11 - 20 m/s while the exit match
  -- crossed 4 - 7.7 m in 1.00 s; the jump-out unmounts 1.04 s after the exit start, so a slower match cannot be timed to end before it). From the exit
  -- take (TakeFromVehicle with an exit reason, TakeForVehicleExit) to the release, every override pose of the hold is the requested one played through
  -- CM.poseGovStep: the request's own velocity is fed forward, the shown pose closes the remaining gap at most vehExitGovSpeed m/s beyond V's own speed
  -- (vehExitGovAccel m/s^2, braking onto the request without an overshoot), yaw / pitch at vehExitGovYawRate deg/s (vehExitGovYawAccel deg/s^2), FOV at
  -- vehExitGovFovRate deg/s. A slow request is shown as it is (feed-forward, exact once the gap closed). vehExitGov = false: the request as before.
  vehExitGov = true, vehExitGovSpeed = 5.0, vehExitGovAccel = 14, vehExitGovYawRate = 90, vehExitGovYawAccel = 360, vehExitGovFovRate = 30,
  vehExitGovFovAccel = 120, vehExitGovRefMax = 15,
  -- a jump-out (V seen in flight, or the landing mode armed with V faster than vehExitGovJumpSpeed m/s) hands the governor's offset over as a
  -- decaying tail (vehExitGovTailRate 1/s); a rolling exit below that speed stays governed (6 m/s: between the logged rolling exits classed as
  -- jump-outs, 4.0 / 4.2 m/s, and the slowest replayed moving jump-out, 7 m/s)
  vehExitGovJumpSpeed = 6, vehExitGovTailRate = 8,
  -- release speed: the release to the orbit is a smoothstep over its blend time (peak speed 1.5 x distance / time: 4.2 m in 0.80 s = 7.9 m/s,
  -- 0.13 m per frame, standard3 'behind'). The blend time grows to 1.5 x distance / vehReleaseSpeed (at most vehReleaseTimeMax s); a short
  -- distance keeps the caller's time. A release that starts while the exit governor still moves carries its velocity (the start-velocity term of the
  -- jump-out release). Only after a governed exit hold; a target farther than vehReleaseDistMax m keeps the caller's time (no continuity case).
  -- vehReleaseSpeed = 0: the caller's time as before.
  vehReleaseSpeed = 5.5, vehReleaseTimeMax = 1.8, vehReleaseDistMax = 20,
  -- ragdoll camera (V knocked down by a car, the DLL's production ragdoll: Game.UTPPRagdollState 1 = the ragdoll owns the body,
  -- 2 = blend-out; Game.UTPPRagdollPelvis = the Hips in V's model space). The eye / capsule height of the normal pivot is meaningless while
  -- the body tumbles and lies on the ground, so for the episode (state 1 / 2, then the PSM knockdown get-up until it ends) the pivot is the
  -- world pelvis + ragLift m, followed by a critically damped spring per axis (ragSmoothActive s while the ragdoll owns the body,
  -- ragSmoothGetUp s after it). Enter: the spring starts at the pivot shown last frame with its velocity (no step). Exit: the pivot crosses
  -- over to the normal capsule pivot in ragExitBlend s (smoothstep). The capsule pivot's own fast-fall / snap logic is pinned (CM.pivotStep
  -- pin) for the episode. The look is not locked: the orbit keeps its world yaw (mouse / stick still turn it) and its own pitch, the game's
  -- look is not read. The boom is ragDistAdd m longer and looks ragPitchBias deg further down (added to the shown pitch only, never to the
  -- look state), weighted in over ragBlendIn s and out over ragBlendOut s. Collision is unchanged. A pelvis older than the DLL's freshness
  -- window (stale) keeps its last model-space point, which rises to ragStandPelvis m over ragRise s (the get-up). The episode ends ragEndHold s
  -- after the state is 0 and the PSM knockdown is over, at the latest ragMaxTime s after it began. ragdollCam = false: the old camera.
  ragdollCam = true, ragLift = 0.35, ragSmoothActive = 0.15, ragSmoothGetUp = 0.25, ragExitBlend = 0.4,
  ragDistAdd = 0.6, ragPitchBias = -10, ragBlendIn = 0.5, ragBlendOut = 0.6,
  ragStandPelvis = 1.0, ragRise = 1.5, ragEndHold = 0.1, ragMaxTime = 12.0,
  ragLockGrace = 0.5, ragLogGap = 2.0, -- (utpp_camera/init.lua: heading lock re-assert waits ragLockGrace s after the episode; log rate limit)
  -- ragdoll camera blend-in: the pivot spring starts with a slower smooth time (ragEnterSmooth s) that eases to the normal one
  -- over ragEnterBlend s (smoothstep), so the first frames of the episode follow a launched body with a bounded step. ragEnterBlend = 0 or
  -- ragEnterSmooth = 0: the old constant spring. Collision is unchanged.
  ragEnterBlend = 0.3, ragEnterSmooth = 0.3,
  -- early return (native get-up only; inp.ragdoll.getup = { state = Game.UTPPRagdollGetUpState(), frac = clip time / length }): while the
  -- episode still runs, the return starts when the native get-up plays (state 2) and its clip passed ragReturnFrac of its length, or at the hand-off
  -- (state 3) at the latest. For ragReturnTime s (smoothstep) the pivot target crosses from the pelvis point to the capsule pivot (the pelvis keeps
  -- rising with V's head until then), the boom bias / length weight falls to 0 and, when the orbit is more than ragRecenterMin deg off V's facing,
  -- its yaw eases onto V's facing (ragRecenter). The episode itself (freeze, natives, end rule, exit blend) is unchanged and ends as before.
  -- ragReturnEarly = false or ragReturnTime = 0 (or no getup input = the vanilla path): the old return after the episode end.
  -- (the view must stay free while V lies and gets up; during RETURN the yaw drifted 18 deg onto V's facing
  -- without input): ragRecenter is off by default, the pivot / boom return stays. ragRecenter = true: the old ease, cancelled for the rest of the
  -- episode by the first frame with look input (ragRecenterInput deg of yaw or pitch input, 0 = any).
  ragReturnEarly = true, ragReturnFrac = 0.45, ragReturnTime = 0.9, ragRecenter = false, ragRecenterMin = 15, ragRecenterInput = 0,
  -- framing constraint (the slower enter spring lagged a launched body: V's head left the frame at the bottom). During the
  -- episode (phases "on" / "out") V's head (pelvis + ragHeadUp m) and pelvis (- ragLowDown m for the low point) are projected into the camera of this frame
  -- (the boom before the collision shortening, the shown orientation and FOV, ragFrameAspect = width / height); the part outside ragFrameMargin of the half
  -- FOV (vertical) / half horizontal FOV is taken back by shifting the pivot (and so the camera) sideways / up by the missing amount only. The shift is a
  -- continuous function of V's position (zero while V is inside the band): no cut, no extra state, the orientation is untouched. ragFrame = false: off.
  -- ragFrameRate (m/s): the applied shift follows the wanted one at most this fast (0 = no limit), so a jump in the body data cannot cut the view.
  ragFrame = true, ragFrameMargin = 0.75, ragFrameAspect = 16 / 9, ragHeadUp = 0.75, ragLowDown = 0.25, ragFrameRate = 8,
}
function CM.config(over)
  local c = {}
  for k, v in pairs(CM.DEFAULTS) do c[k] = v end
  for k, v in pairs(over or {}) do c[k] = v end
  return c
end

-- ================================================================ input
-- acc = { mouseX, mouseY (sum of this frame), padX, padY (current stick) } -> dyaw, dpitch (deg; + yaw = left)
function CM.inputDelta(acc, cfg, dt, degPerUnit)
  local sx = (cfg.invertX and -1 or 1) * cfg.sens
  local sy = (cfg.invertY and -1 or 1) * cfg.sens
  local unit = degPerUnit or cfg.degPerMouseUnit
  local dyaw = -(acc.mouseX or 0) * unit * sx
  local dpitch = (acc.mouseY or 0) * unit * sy
  local function dz(v)
    v = v or 0
    local a = abs(v)
    if a <= cfg.padDeadzone then return 0 end
    local s = (a - cfg.padDeadzone) / (1 - cfg.padDeadzone)
    return (v < 0 and -1 or 1) * clamp(s, 0, 1)
  end
  dyaw = dyaw - dz(acc.padX) * cfg.padDegPerSec * sx * dt
  dpitch = dpitch + dz(acc.padY) * cfg.padDegPerSec * cfg.padPitchScale * sy * dt
  return dyaw, dpitch
end
-- Mouse scale calibration: while V follows the camera the game itself turns the body by the mouse; the ratio body
-- yaw change / mouse units of the same frame gives degrees per unit (EMA). Returns the calibrated value or nil.
function CM.newCalibration() return { value = nil, samples = 0 } end
-- band (> 1) = once calibrated (>= 10 samples) a ratio outside [value / band, value * band] is an
-- outlier (body turned by something else while the mouse moved: heading unlock, motion matching) and rejected; streak
-- rejections in a row re-seed the value (the old value was the wrong one).
function CM.calibrate(cal, mouseX, bodyYawDelta, band, streak)
  if abs(mouseX or 0) < 2 or abs(bodyYawDelta or 0) < 0.05 then return cal.value end
  local ratio = bodyYawDelta / -mouseX
  if ratio <= 0 then return cal.value end -- opposite sign: not caused by this input
  ratio = clamp(ratio, 0.001, 2)
  if band and band > 1 and cal.value and cal.samples >= 10 and (ratio > cal.value * band or ratio < cal.value / band) then
    cal.rejected = (cal.rejected or 0) + 1
    cal.streak = (cal.streak or 0) + 1
    if cal.streak < (streak or 20) then return cal.value end
    cal.value, cal.streak, cal.reseeds = ratio, 0, (cal.reseeds or 0) + 1
    return cal.value
  end
  cal.streak = 0
  cal.samples = cal.samples + 1
  if cal.value == nil then cal.value = ratio else cal.value = cal.value + (ratio - cal.value) * 0.1 end
  return cal.value
end
function CM.mouseUnit(cal, cfg, override)
  if override and override > 0 then return override end
  if cal and cal.value and cal.samples >= 10 then return cal.value end
  return cfg.degPerMouseUnit
end
-- game look sync evidence. ls = CM.newLookSync(); one sample per frame while the heading lock is
-- wanted: s = { want, locked (FPP headingLocked read back), mouseX (units this frame), pad (stick input), bodyDelta,
-- lookDelta (deg, this frame) }. Mouse only and V still: the look must move with the mouse (confirm). V turned >= 1 deg
-- without look input: the look must stay in the world (maintained, confirm) - turning with V (carried) lookSyncCarried
-- times in a row asks for a resend (<= lookSyncResends), then the sync is broken. Returns "ok" | "resend" | "broken" | nil.
function CM.newLookSync()
  return { ok = false, broken = false, confirm = 0, maintained = 0, carried = 0, carriedTotal = 0, resends = 0 }
end
function CM.lookSyncSample(ls, cfg, s)
  if not s.want or not s.locked or type(s.bodyDelta) ~= "number" or type(s.lookDelta) ~= "number" then
    ls.carried = 0
    return nil
  end
  local bd, ld, mx = s.bodyDelta, CM.wrapDeg(s.lookDelta), s.mouseX or 0
  local need = cfg.lookSyncConfirm or 2
  if abs(bd) < 0.05 and abs(mx) >= 2 and not s.pad then
    if ld * -mx > 0 and abs(ld) >= 0.05 then
      ls.confirm = ls.confirm + 1
      if not ls.ok and not ls.broken and ls.confirm >= need then ls.ok = true; return "ok" end
    end
  elseif abs(bd) >= 1 and abs(mx) < 1 and not s.pad then
    if abs(ld) < 0.2 * abs(bd) then
      ls.maintained, ls.carried = ls.maintained + 1, 0
      if not ls.ok and not ls.broken and ls.maintained >= need then ls.ok = true; return "ok" end
    elseif abs(CM.wrapDeg(ld - bd)) < 0.2 * abs(bd) then
      ls.carried, ls.carriedTotal = ls.carried + 1, ls.carriedTotal + 1
      if ls.carried >= (cfg.lookSyncCarried or 3) then
        ls.carried = 0
        if ls.broken then return nil end
        if ls.resends < (cfg.lookSyncResends or 2) then ls.resends = ls.resends + 1; return "resend" end
        ls.broken, ls.ok = true, false
        return "broken"
      end
    end
  end
  return nil
end

-- game look write: true when the game's look is set to the shown camera this frame:
-- camera.lookWrite on, the world camera written this frame (s.world), the orbit pose (s.kind, not a vehicle hold / release
-- blend), third person (s.state "tpp") and the FPP heading lock set (s.locked: free camera or aiming / unarmed zoom).
function CM.lookWriteWant(cfg, s)
  return cfg.lookWrite ~= false and s.world == true and s.kind == "orbit" and s.state == "tpp" and not s.held
    and s.locked == true
end

-- ================================================================ look (yaw offset / pitch)
-- Two modes (design 2.2): follow (V turns with the mouse, camera behind; yaw = body yaw + decaying offset, pitch from
-- the FPP camera) and free (free look held or idle orbit: own world yaw/pitch from input, V does not turn).
-- out = shown world yaw (follow: rate limited), vel = its speed (deg/s), hold = world yaw hold left (s), hist = body
-- turn without the mouse over the last flipWindow s, flip = event of this frame for the log (taken by CM.solve)
function CM.newLook()
  return { yawOff = 0, freeYaw = nil, pitch = 0, free = false, recovering = false, still = 0, out = nil, vel = 0, hold = 0,
           hist = {}, prevBody = nil, flip = nil, flips = 0 }
end
-- one step of an offset (deg) toward 0: offset * (1 - exp(-aimAlignGain dt)), at least aimAlignMin
-- deg/s, at most aimAlignRate deg/s (0 = no cap), never past 0
function CM.alignStep(off, cfg, dt)
  local a = abs(off or 0)
  if a < 1e-6 then return 0 end
  local step = math.max(a * CM.expAlpha(cfg.aimAlignGain or 10, dt), (cfg.aimAlignMin or 0) * math.max(dt, 0))
  if (cfg.aimAlignRate or 0) > 0 then step = math.min(step, cfg.aimAlignRate * math.max(dt, 0)) end
  step = math.min(step, a)
  return off > 0 and (off - step) or (off + step)
end
-- yaw step (deg) the entity will turn in the coming game tick, predicted from the last measured body
-- step (bodyDelta over prevDt) scaled to this frame's dt; 0 for steps below 0.05 deg or above maxStep (teleport / one-off
-- turns are not repeated). The camera component lives in the entity space and the CET frame runs before the game tick,
-- so a turn applied in that tick would carry the camera with it.
function CM.predictTurn(bodyDelta, dt, prevDt, maxStep)
  local d = tonumber(bodyDelta) or 0
  if abs(d) < 0.05 or abs(d) > (maxStep or 15) or not prevDt or prevDt <= 0 or not dt or dt <= 0 then return 0 end
  return d * clamp(dt / prevDt, 0.5, 2)
end
-- orbit yaw back behind V's back (return from aiming); keepOut = keep the shown yaw (the boom is partly out: the
-- limiter glides back instead of a jump)
function CM.lookReset(L, bodyYaw, keepOut)
  L.yawOff, L.free, L.recovering, L.still, L.freeYaw, L.hold, L.hist = 0, false, false, 0, bodyYaw, 0, {}
  L.sync, L.syncP = false, false
  if not keepOut then L.out, L.vel = nil, 0 end
end
-- follow-mode limiter: shown yaw L.out towards target (deg/s capped, ease-in), the mouse part dyaw applied at once
local function followLimit(L, cfg, dt, target, dyaw, sum)
  if (cfg.followMaxRate or 0) <= 0 or L.out == nil then
    L.out, L.vel, L.hold = target, 0, 0
    return
  end
  L.out = CM.wrapDeg(L.out + (dyaw or 0))
  if cfg.followHoldOnFlip and L.hold <= 0 and abs(sum) > cfg.flipDeg then
    L.hold, L.vel = cfg.flipHold, 0
    L.flip = { deg = sum, window = cfg.flipWindow, hold = cfg.flipHold, rate = cfg.followMaxRate }
    L.flips = (L.flips or 0) + 1
    L.hist = {}
  end
  if L.hold > 0 then
    L.hold = L.hold - dt -- world yaw held (only the mouse turns the camera)
    return
  end
  local err = CM.wrapDeg(target - L.out)
  local vd = clamp(err * cfg.followGain, -cfg.followMaxRate, cfg.followMaxRate)
  local v = L.vel or 0
  if v * vd < 0 then v = 0 end
  if abs(vd) > abs(v) then
    local acc = cfg.followMaxRate / math.max(cfg.followEase or 0, 1e-3)
    v = v + (vd > 0 and 1 or -1) * math.min(abs(vd) - abs(v), acc * dt)
  else
    v = vd
  end
  local step = v * dt
  if abs(step) >= abs(err) then step, v = err, (abs(err) < 1e-6) and 0 or v end
  L.out, L.vel = CM.wrapDeg(L.out + step), v
end
-- env: bodyYaw (deg), fppPitch (deg|nil), freeHeld, moving (bool), teleport (bool: the follow yaw snaps), snap (bool:
-- limiter off this frame - the orbit is hidden behind a vehicle hold pose / release blend), aim (bool: forced
-- follow - free look / idle orbit off, limiter off), aimW (0..1: aim weight of the pose tween; widens the pitch range
-- smoothly; nil = 1). refYaw (deg|nil: the game's look yaw = FPP camera world yaw; aim reference,
-- nil = body yaw): the yaw / pitch shown when the aim starts is kept and its offset to refYaw / fppPitch shrinks with
-- CM.alignStep (no snap onto the body yaw, which may face the movement direction or turn under the camera).
-- lookYaw (deg|nil: the game's look yaw while it is synced - its world heading kept while V turns):
-- free look / own aim yaw = lookYaw and pitch = fppPitch, each plus the offset at the switch moment (CM.alignStep)
-- lookEcho (bool: the game's look was set to our target last frame - fppPitch is that echo, not the mouse):
-- the aim pitch integrates dpitch itself
-- lookWriteOn (bool: the game's look is written this frame - camera.lookWrite on, the native live, not tripped):
-- the aim / zoom pitch and yaw are the camera's own (mouse integration, as free look) from the first aim frame; the
-- game's look (fppPitch / refYaw / lookYaw) is never read back into the camera while it is written
-- adjustPending (bool: utpp_bodyturn requested an adjustTransform turn of V this frame or the last one -
-- the game overrides its FPP look in the ticks it lands, pitch ~0 / V's yaw): without the write the aim / zoom pitch does
-- not read fppPitch in such a frame (the mouse is integrated, the offset to the game's pitch kept; an aim that starts in
-- such a frame takes its offset at the first clean frame)
function CM.lookStep(L, cfg, dt, dyaw, dpitch, env)
  local aim = env.aim == true
  -- idle orbit timer (held at 0 while aiming - no idle orbit right after the aim ends)
  if env.moving or aim then L.still = 0 else L.still = L.still + dt end
  -- env.shooterMode (Shooter mode, utpp_bodyturn): no idle orbit - the camera always follows V's capsule (which follows
  -- the mouse); the standing V keeps her visible direction in the DLL instead
  local idle = cfg.idleOrbit and not env.shooterMode and L.still >= cfg.idleDelay
  -- decoupled: UTPP Locomotion turns V toward the movement (strafes flip the body yaw every frame); following
  -- the body then shakes the camera on left/right and forward/back changes, so the camera keeps its own world yaw as in
  -- free look (mouse / stick only) while Locomotion is active and V is not aiming
  -- env.holdYaw (vehicle hold with the orbit shown - no pose override / release blend yet): the
  -- world yaw is kept like free look (mouse / stick still apply), so a body turn at the mount start does not whip the view
  local free = ((env.freeHeld or idle) or env.decoupled == true or env.holdYaw == true) and not aim
  -- aim starts from the shown yaw / pitch (a lagging follow limiter / free look must not jump).
  -- both offsets are kept relative to the aim reference (refYaw / fppPitch) and shrink with CM.alignStep (time based)
  local ref = env.refYaw or env.bodyYaw
  if aim and not L.aim then
    local shown = L.out or CM.wrapDeg(env.bodyYaw + L.yawOff)
    L.aimYawOff = CM.wrapDeg(shown - ref)
    L.aimPitchOff = env.fppPitch ~= nil and (L.pitch - env.fppPitch) or 0
    -- fppPitch of this frame is the adjustTransform override - the offset is taken at the first clean frame
    if env.adjustPending == true then L.aimEcho = true end
  end
  -- the aim ended while its pitch was our own (look echo): the game's look holds the last written target
  -- (parallax corrected) - the follow pitch glides onto it (pitchRecoverRate) instead of jumping
  if not aim and L.aim and L.aimEcho then L.recovering = true end
  L.aim = aim
  if not aim then L.aimEcho = false end
  -- Free look always starts from the displayed world yaw, including a settled limiter.
  -- A body turn on the entry frame must not be added to the view (Locomotion/hold handoff).
  if free and not L.free then
    L.freeYaw = L.out or CM.wrapDeg(env.bodyYaw + L.yawOff)
  end
  if not free and L.free then L.recovering = true end
  L.free = free
  -- (world camera): aiming / zooming keep the camera's own yaw (mouse / stick) - the game's
  -- look (FPP camera) turned together with V, so aligning onto it made V and the camera chase each other (aim align
  -- stopped after 540 deg). utpp_bodyturn turns V onto this camera yaw instead (cfg.aimOwnYaw, default on).
  local own = aim and (cfg.aimOwnYaw ~= false or env.lookWriteOn == true)
  if own and not L.own then L.freeYaw = L.out or CM.wrapDeg(env.bodyYaw + L.yawOff) end
  L.own = own
  -- the game's look (synced) drives the free / own yaw; the mouse integration is the fallback
  local sync = type(env.lookYaw) == "number" and (free or own)
  if free or own then
    if sync then
      if not L.sync then L.syncOff = CM.wrapDeg((L.freeYaw or env.lookYaw) - env.lookYaw) end
      L.syncOff = CM.alignStep(L.syncOff or 0, cfg, dt)
      L.freeYaw = CM.wrapDeg(env.lookYaw + L.syncOff)
    else
      L.freeYaw = CM.wrapDeg(L.freeYaw + dyaw)
    end
    L.yawOff = CM.wrapDeg(L.freeYaw - env.bodyYaw) -- robust when the game turns V anyway (heading lock missing)
  else
    if aim then
      L.aimYawOff = CM.alignStep(L.aimYawOff or 0, cfg, dt)
      L.yawOff = CM.wrapDeg(ref + L.aimYawOff - env.bodyYaw)
    else
      L.yawOff = CM.wrapDeg(L.yawOff) * (1 - CM.expAlpha(cfg.recenterRate, dt))
      if abs(L.yawOff) < 0.01 then L.yawOff = 0 end
    end
    L.freeYaw = env.bodyYaw + L.yawOff
  end
  -- body turn without the mouse over the last flipWindow s (in follow mode the game turns V by the mouse = dyaw). The
  -- mouse part passed at once is capped by the body's own turn of this frame (a wrong mouse scale before the calibration
  -- cannot over-rotate the camera); frames with mouse / stick input do not count for the flip detection.
  if env.teleport then L.out, L.vel, L.hold, L.hist = nil, 0, 0, {} end
  local bodyDelta = L.prevBody and CM.wrapDeg(env.bodyYaw - L.prevBody) or 0
  L.prevBody = env.bodyYaw
  dyaw = dyaw or 0
  local mousePart = 0
  if not free and dyaw * bodyDelta > 0 then mousePart = (dyaw > 0 and 1 or -1) * math.min(abs(dyaw), abs(bodyDelta)) end
  local auto = (abs(dyaw) < 0.5) and (bodyDelta - mousePart) or 0
  local h = L.hist
  h[#h + 1] = { dt = dt, d = auto }
  local sum, span = 0, 0
  for i = #h, 1, -1 do
    span = span + h[i].dt
    if span > (cfg.flipWindow or 0.3) + 1e-9 and i < #h then
      for _ = 1, i do table.remove(h, 1) end
      break
    end
    sum = sum + h[i].d
  end
  local target = CM.wrapDeg(env.bodyYaw + L.yawOff)
  if free or env.snap or aim then -- snap: the orbit is not shown (vehicle hold) - rigid follow; aim: the
    L.out, L.vel, L.hold = target, 0, 0 -- camera forward is the aim direction (no limiter lag)
  else
    followLimit(L, cfg, dt, target, mousePart, sum)
  end
  -- with camera.pitchSource=input (persisted by the old frozen-pitch fallback) the aim / unarmed
  -- zoom pitch was the own mouse pitch: it ran to the -80 limit while the game looked at -40 (top-down view over the head).
  -- Aiming / zooming always follow the game's look pitch (= where the shot goes) when it is known.
  local useFpp = env.fppPitch ~= nil and not free and (cfg.pitchSource == "fpp" or aim)
  -- synced free look = the game's look pitch (+ the switch offset); aiming keeps its own path below
  local syncP = sync and env.fppPitch ~= nil and not aim
  L.sync = sync
  -- vehicle: env.ownPitch (vehicle hold, traversal, the 0.5 s after the game look write
  -- turned off): the game's look pitch is animation driven there (vault / climb / mount / exit sets: -60 at the end of an
  -- exit) - the pitch is the camera's own (mouse only) and fppPitch is not read; when it ends the follow pitch glides onto
  -- the game's (pitchRecoverRate, at most cfg.ownPitchGlideRate deg/s) instead of a one-frame snap
  local ownP = env.ownPitch == true and not aim
  if L.ownP and not ownP and not aim then L.recovering, L.ownGlide = true, true end
  L.ownP = ownP
  if ownP then
    L.pitch, L.recovering, L.ownGlide = L.pitch + (dpitch or 0), false, false
    syncP = false
  elseif syncP then
    if not L.syncP then L.syncPitchOff = L.pitch - env.fppPitch end
    L.syncPitchOff = CM.alignStep(L.syncPitchOff or 0, cfg, dt)
    L.pitch, L.recovering = env.fppPitch + L.syncPitchOff, false
  elseif aim and (env.lookWriteOn == true or (useFpp and env.lookEcho == true)) then
    -- the game's look was set to our target last frame (game look write: the native replaces
    -- the game's mouse delta, so fppPitch only echoes what was written - reading it froze the aim pitch, and a parallax
    -- corrected target would feed back): the aim pitch integrates the mouse itself (as free look); the offset to the
    -- game's pitch is taken again when the echo ends
    -- (aiming jerked up / down / left / right; the trace showed the aim pitch following the game's
    -- look pitch, which jumped by 30-60 deg between frames): while the write is on (env.lookWriteOn) this holds from the
    -- first aim frame, independent of the echo of the last frame
    L.pitch, L.recovering, L.aimEcho = L.pitch + (dpitch or 0), false, true
  elseif useFpp and aim and env.adjustPending == true then
    -- (aiming jerked; the aim pitch followed the game's look pitch, which the adjustTransform
    -- of utpp_bodyturn's aim align overrode for a frame: 52.9 -> -8.7 -> 51.1): the mouse is integrated in such a frame,
    -- the offset to the game's pitch (and a pending echo re-take) stay for the next clean frame
    -- recheck: mark the echo so the first clean frame re-takes the offset (no one-frame jump)
    L.pitch, L.recovering, L.aimEcho = L.pitch + (dpitch or 0), false, true
  elseif useFpp and aim then
    if L.aimEcho then L.aimPitchOff = L.pitch - env.fppPitch end
    L.aimEcho = false
    L.aimPitchOff = CM.alignStep(L.aimPitchOff or 0, cfg, dt)
    L.pitch, L.recovering = env.fppPitch + L.aimPitchOff, false
  elseif useFpp then
    if L.recovering then
      local before = L.pitch
      L.pitch = CM.approach(L.pitch, env.fppPitch, cfg.pitchRecoverRate, dt)
      -- the glide after an own-pitch phase is also rate limited (a 55 deg gap after an exit would
      -- move 8 deg in the first frame with the exponential alone)
      if L.ownGlide and (cfg.ownPitchGlideRate or 0) > 0 then
        local cap = cfg.ownPitchGlideRate * math.max(dt, 0)
        L.pitch = before + clamp(L.pitch - before, -cap, cap)
      end
      -- (review: the release still jumped one frame later): while fppPitch is our own echo (the
      -- corrected look written last frame = the camera pitch) the recovery stays on - the game's raw pitch arrives the
      -- frame after and is glided onto, not snapped
      if abs(L.pitch - env.fppPitch) < cfg.pitchSnapDeg and env.lookEcho ~= true then L.recovering = false end
    end
    if not L.recovering then L.pitch = env.fppPitch end
  else
    L.pitch = L.pitch + dpitch
    L.recovering = false
  end
  if not L.recovering then L.ownGlide = false end
  L.syncP = syncP
  local aw = clamp(env.aimW or (aim and 1 or 0), 0, 1)
  -- crouched, looking up put the camera on the ground under V (pivot 1.05 m, collision pulls
  -- the boom in) - the orbit look-up is capped lower while crouched (camera cfg crouchPitchMax)
  local pMax = cfg.pitchMax
  if env.crouch == true and cfg.crouchPitchMax then pMax = math.min(pMax, cfg.crouchPitchMax) end
  L.pitch = clamp(L.pitch, cfg.pitchMin + ((cfg.aimPitchMin or cfg.pitchMin) - cfg.pitchMin) * aw,
    pMax + ((cfg.aimPitchMax or pMax) - pMax) * aw)
  return CM.wrapDeg(L.out), L.pitch
end

-- pitch (deg) that keeps the point pivot inside the vertical band [cfg.vehFrameLow,
-- cfg.vehFrameHigh] of the half vertical FOV (fov deg; + = above the screen centre) seen from cam: the current pitch
-- clamped to [need - high * h, need - low * h], need = the elevation of cam -> pivot. Unchanged when the pivot is
-- (nearly) straight above / below the camera or the input is missing.
function CM.vehFramePitch(pitch, cam, pivot, fov, cfg)
  if type(pitch) ~= "number" or not cam or not pivot then return pitch end
  local dx, dy, dz = pivot.x - cam.x, pivot.y - cam.y, pivot.z - cam.z
  local hd = sqrt(dx * dx + dy * dy)
  if hd < 0.3 then return pitch end
  local need = deg(atan2(dz, hd))
  local h = 0.5 * (tonumber(fov) or 60)
  local lo, hi = (cfg and cfg.vehFrameLow) or -0.35, (cfg and cfg.vehFrameHigh) or 0.25
  return clamp(pitch, need - hi * h, need - lo * h)
end

-- ================================================================ profiles
CM.PROFILE_FIELDS ={ "distance", "height", "side", "fov", "sens" }
CM.PROFILE_LIMITS = {
  distance = { 0.6, 6.0, 0.05 }, height = { -0.5, 1.0, 0.01 }, side = { -1.2, 1.2, 0.01 },
  fov = { 30, 100, 1 }, sens = { 0.2, 3.0, 0.05 },
}
CM.DEFAULT_PROFILES = {
  -- fov in camera component units (the FPP component has 51 while the game setting shows 80: vertical-like units)
  { name = "Close",     distance = 1.25, height = 0.08, side = 0.42, fov = 56, sens = 1.0 },
  { name = "Medium",    distance = 2.75, height = -0.35, side = 0.50, fov = 55, sens = 1.0 },
  { name = "Far",       distance = 3.20, height = 0.25, side = 0.32, fov = 58, sens = 1.0 },
  { name = "Centered",  distance = 2.60, height = 0.30, side = 0.00, fov = 55, sens = 1.0 },
  { name = "Cinematic", distance = 3.80, height = 0.35, side = 0.55, fov = 48, sens = 0.8 },
}
function CM.copyProfile(p)
  local r = { name = p.name }
  for _, f in ipairs(CM.PROFILE_FIELDS) do r[f] = p[f] end
  return r
end
function CM.profileLerp(a, b, t)
  local r = { name = (t < 0.5) and a.name or b.name }
  for _, f in ipairs(CM.PROFILE_FIELDS) do r[f] = a[f] + (b[f] - a[f]) * t end
  return r
end
-- wrapped 1-based index in a pool of n
function CM.cycle(i, n, dir)
  n = math.max(1, n)
  return ((i - 1 + dir) % n) + 1
end
function CM.newBlend(p) return { from = CM.copyProfile(p), to = CM.copyProfile(p), t = 1, dur = 0.5 } end
function CM.blendTo(b, p, dur)
  b.from = CM.blendCurrent(b)
  b.to = CM.copyProfile(p)
  b.t = 0
  b.dur = math.max(dur or 0.5, 0)
  if b.dur == 0 then b.t = 1 end
end
-- live edit of the target profile (slider) without restarting a running blend
function CM.blendRetarget(b, p) b.to = CM.copyProfile(p) end
function CM.blendCurrent(b) return CM.profileLerp(b.from, b.to, CM.smoothstep(b.t)) end
function CM.blendStep(b, dt)
  if b.t < 1 then b.t = (b.dur > 0) and math.min(1, b.t + dt / b.dur) or 1 end
  return CM.blendCurrent(b)
end

-- ================================================================ shoulder
function CM.newShoulder(side) side = side or 1; return { s = side, target = side, from = side, t = 1 } end
function CM.shoulderSwap(st)
  st.from = st.s
  st.target = -st.target
  st.t = 0
  return st.target
end
function CM.shoulderStep(st, dt, dur)
  if st.t < 1 then
    st.t = (dur and dur > 0) and math.min(1, st.t + dt / dur) or 1
    st.s = st.from + (st.target - st.from) * CM.smoothstep(st.t)
  else
    st.s = st.target
  end
  return st.s
end

-- ================================================================ aim (over the shoulder in third person)
-- gamedataItemType names of melee weapons / fists / melee cyberware: not aiming (a block) - same list as utpp_perspective
-- P.MELEE_TYPES (weapon.script IsBlade / IsBlunt)
CM.MELEE_TYPES = {
  Wea_Fists = true, Wea_Melee = true, Wea_Katana = true, Wea_Knife = true, Wea_OneHandedClub = true, Wea_TwoHandedClub = true,
  Wea_Hammer = true, Wea_Machete = true, Wea_Sword = true, Wea_Chainsword = true, Wea_Axe = true, Wea_LongBlade = true,
  Wea_ShortBlade = true, Cyb_MantisBlades = true, Cyb_StrongArms = true, Cyb_NanoWires = true,
}
-- item in WeaponRight counts as ranged: present, type not melee (nil = type unreadable: ranged), PSM MeleeWeapon not a
-- block / targeting state (5-10, gamePSMMeleeWeapon)
function CM.weaponRanged(present, itemType, meleeState)
  if not present then return false end
  if type(itemType) == "string" and (itemType == "" or CM.MELEE_TYPES[itemType]) then return false end
  local ms = tonumber(meleeState)
  if ms and ms >= 5 and ms <= 10 then return false end
  return true
end
-- enter / stay decision. o = { tpp (utppCamera renders: third person entered and wanted), upperBody (PSM UpperBody int),
-- weaponRanged (bool), aimingSetting (perspective.aiming: "tpp" | "fpp" | false = legacy tpp), vehicleHeld, poseOverride,
-- enabled (camera.aim.enabled, nil = on) } -> wanted (bool), reason (first failing condition | "aiming")
function CM.aimWanted(o)
  if not o then return false, "no data" end
  if o.enabled == false then return false, "camera.aim.enabled off" end
  if o.vehicleHeld then return false, "vehicle hold" end
  if o.poseOverride then return false, "pose override" end
  local s = o.aimingSetting
  if not (s == "tpp" or s == false) then return false, "perspective.aiming=" .. tostring(s) end
  if not o.tpp then return false, "not third person" end
  if tonumber(o.upperBody) ~= 6 then return false, "UpperBody=" .. tostring(o.upperBody) end
  if not o.weaponRanged then return false, "no ranged weapon" end
  return true, "aiming"
end
-- aim state: on = the aim decision, w = pose weight (0 = profile, 1 = aim pose), tween like CM.blendTo (linear t over dur,
-- smoothstep, restarts from the current weight when interrupted)
function CM.newAim()
  return { on = false, w = 0, from = 0, target = 0, t = 1, dur = 0, since = 0, clock = 0, enters = 0, exits = 0, swaps = 0,
           reason = nil, swap = CM.newAimSwap() }
end
-- one frame: returns "enter" | "exit" | nil (event of this frame), weight
function CM.aimStep(A, cfg, dt, want, reason)
  dt = math.max(dt or 0, 0)
  A.clock = A.clock + dt
  A.prevT = A.t
  local ev = nil
  want = want and true or false
  if want ~= A.on then
    A.on = want
    A.from, A.target, A.t = A.w, want and 1 or 0, 0
    A.dur = math.max(want and (cfg.aimBlendIn or 0.12) or (cfg.aimBlendOut or 0.20), 0)
    A.prevSince, A.since, A.reason = A.since, A.clock, reason
    if want then A.enters = A.enters + 1 else A.exits = A.exits + 1 end
    A.swap.t = 0
    ev = want and "enter" or "exit"
    if A.dur == 0 then A.t = 1 end
  elseif A.t < 1 then
    A.t = (A.dur > 0) and math.min(1, A.t + dt / A.dur) or 1
  end
  A.w = A.from + (A.target - A.from) * CM.smoothstep(A.t)
  return ev, A.w
end
-- the tween is running or the aim pose is (partly) shown; also the frame the exit tween ends (exact profile FOV written)
function CM.aimActive(A) return A ~= nil and (A.on or A.w > 0 or A.t < 1 or (A.prevT or 1) < 1) end
-- seconds in the current aim state (on: since enter, off: since exit)
function CM.aimTime(A) return A and (A.clock - A.since) or 0 end
-- aim pose as a profile: absolute distance / side (times the shoulder sign in the solver) / height / FOV; sens of the base
function CM.aimProfile(base, cfg, fov)
  local f = fov or cfg.aimFov or 42
  if cfg.aimZoomFov == false and base and base.fov then f = base.fov end -- no FOV zoom while aiming
  return { name = "Aim", distance = cfg.aimDistance or 0.8, height = cfg.aimHeight or 0.05, side = cfg.aimSide or 0.45,
           fov = f, sens = base and base.sens or 1 }
end
-- live profile (profile blend) mixed with the aim pose by the aim weight
function CM.aimCompose(base, A, cfg, fov)
  if not A or A.w <= 0 then return base end
  return CM.profileLerp(base, CM.aimProfile(base, cfg, fov), A.w)
end
-- shoulder auto-swap: hitCur = distance of the shoulder-side ray hit (nil = clear); a wall within aimSwapWall m for
-- aimSwapHold s continuously -> swap, when the other side (other() -> distance | nil, asked only then) is clearer.
-- The timer restarts after a swap (the new side needs its own aimSwapHold s: hysteresis, no ping-pong).
function CM.newAimSwap() return { t = 0, blockedBoth = false } end
function CM.aimSwapStep(st, cfg, dt, hitCur, other)
  local wall = cfg.aimSwapWall or 0.5
  if hitCur == nil or hitCur > wall then
    st.t, st.blockedBoth = 0, false
    return false
  end
  st.t = st.t + math.max(dt or 0, 0)
  if st.t + 1e-9 < (cfg.aimSwapHold or 0.3) then return false end
  local o = nil
  if type(other) == "function" then o = other() else o = other end
  if o == nil or o > hitCur + 0.05 then
    st.t, st.blockedBoth = 0, false
    return true, hitCur, o
  end
  st.blockedBoth = true -- both sides blocked: keep the side
  return false
end

-- ================================================================ unarmed zoom (RMB with empty hands, third person)
-- Vanilla 2.31: RMB raises PSM UpperBody Aim (6) also without a weapon (upperBodyTransitions AimingStateDecisions: ADS
-- input, no weapon requirement); zoomTransitions then sits in zoomLevelAim and the "Aiming" input context maps the wheel
-- to ZoomIn / ZoomOut (Kiroshi zoom levels on the FPP camera). Here: over-the-shoulder pose steps k = 0..N-1, the wheel
-- step past N-1 = "fpp handoff" (first person: the game's own zoom is shown), RMB release = exit.
-- PSM Locomotion Dodge 6 / DodgeAir 7 / Slide 9 / SlideFall 10 / CrouchDodge 12: no ENTRY (empty-handed dodges / slides
-- raise UpperBody Aim for a moment, utpp_perspective logs)
CM.ZOOM_UNARMED_BLOCK_LOCO = { [6] = "dodge", [7] = "dodge", [9] = "slide", [10] = "slide", [12] = "dodge" }
-- 4 steps by default (RMB + 3 wheel steps), no first person handoff - every wheel step zooms
function CM.zoomUnarmedSteps(cfg) return math.max(1, floor(tonumber(cfg and cfg.zoomUnarmedSteps) or 4)) end
-- pose of step k (clamped to 0..N-1) as a profile: distance linear, FOV geometric (45 / 38.4 / 32.8 / 28 for N = 4)
function CM.zoomUnarmedPose(cfg, k, base)
  local n = CM.zoomUnarmedSteps(cfg)
  k = clamp(floor(tonumber(k) or 0), 0, n - 1)
  local f = (n > 1) and (k / (n - 1)) or 0
  local d0, d1 = cfg.zoomUnarmedDistance or 0.9, cfg.zoomUnarmedLastDistance or 0.6
  local f0, f1 = cfg.zoomUnarmedFov or 45, cfg.zoomUnarmedLastFov or 28
  local fov = (f0 > 0 and f1 > 0) and f0 * (f1 / f0) ^ f or (f0 + (f1 - f0) * f)
  return { name = "ZoomUnarmed", distance = d0 + (d1 - d0) * f, height = cfg.aimHeight or 0.05,
           side = cfg.zoomUnarmedSide or cfg.aimSide or 0.45, fov = fov, sens = base and base.sens or 1 }
end
-- enter / stay decision. o = { enabled (camera.zoomUnarmed.enabled, nil = on), tpp (utppCamera renders third person),
-- upperBody (PSM int), weaponInHand (false = WeaponRight empty; true / nil (unreadable) refuse), meleeState (PSM
-- MeleeWeapon), gadget (PSM CombatGadget), lhcw (PSM LeftHandCyberware), loco (PSM Locomotion), vehicleHeld,
-- poseOverride, on (state already on), handoff (first person handoff active: third person not required), held (s of
-- UpperBody 6), enterDelay (s) } -> wanted, reason (first failing condition | "unarmed zoom" | "first person handoff")
function CM.zoomUnarmedWanted(o)
  if not o then return false, "no data" end
  if o.enabled == false then return false, "camera.zoomUnarmed.enabled off" end
  if o.vehicleHeld then return false, "vehicle hold" end
  if o.poseOverride then return false, "pose override" end
  if tonumber(o.upperBody) ~= 6 then return false, "UpperBody=" .. tostring(o.upperBody) end
  if o.weaponInHand ~= false then
    return false, o.weaponInHand == nil and "WeaponRight unreadable" or "item in WeaponRight"
  end
  local ms, g, l = tonumber(o.meleeState), tonumber(o.gadget), tonumber(o.lhcw)
  if ms and ms >= 5 and ms <= 10 then return false, "melee block (PSM MeleeWeapon=" .. ms .. ")" end
  if g and g >= 1 and g <= 6 then return false, "gadget (PSM CombatGadget=" .. g .. ")" end
  if l and l >= 2 and l <= 10 then return false, "arm cyberware (PSM LeftHandCyberware=" .. l .. ")" end
  if not o.handoff and not o.tpp then return false, "not third person" end
  if not o.on then
    local lk = CM.ZOOM_UNARMED_BLOCK_LOCO[tonumber(o.loco) or -1]
    if lk then return false, lk .. " (PSM Locomotion=" .. tostring(o.loco) .. ")" end
    if (tonumber(o.held) or 0) + 1e-9 < (tonumber(o.enterDelay) or 0) then return false, "enter delay" end
  end
  return true, o.handoff and "first person handoff" or "unarmed zoom"
end
-- state: on, k (step; N = first person handoff), fpp (handoff active), w (pose weight, tween like CM.aimStep), pose
-- (CM.newBlend of the step poses)
function CM.newZoomUnarmed()
  return { on = false, k = 0, fpp = false, w = 0, from = 0, target = 0, t = 1, dur = 0, prevT = 1, since = 0, clock = 0,
           enters = 0, exits = 0, steps = 0, handoffs = 0, reason = nil, pose = nil }
end
-- one frame: wheel = net wheel steps of this frame (+ = ZoomIn / closer). Returns the events of this frame, in order:
-- { kind = "enter" | "step" | "handoff" | "exit", k, dir, reason, dur (exit: s in the state), wasFpp (exit) }.
-- Wheel steps count only while on (also the enter frame) and not in the handoff (the game's zoom owns the wheel there).
function CM.zoomUnarmedStep(Z, cfg, dt, want, reason, wheel)
  dt = math.max(dt or 0, 0)
  Z.clock = Z.clock + dt
  Z.prevT = Z.t
  local ev = {}
  local n = CM.zoomUnarmedSteps(cfg)
  local blend = math.max(tonumber(cfg.zoomUnarmedBlend) or 0.15, 0)
  if Z.pose then CM.blendStep(Z.pose, dt) end
  want = want and true or false
  if want ~= Z.on then
    Z.on = want
    Z.from, Z.target, Z.t, Z.dur = Z.w, want and 1 or 0, 0, blend
    if Z.dur == 0 then Z.t = 1 end
    Z.reason = reason
    if want then
      Z.k, Z.fpp = 0, false
      Z.pose = CM.newBlend(CM.zoomUnarmedPose(cfg, 0))
      Z.since, Z.enters = Z.clock, Z.enters + 1
      ev[#ev + 1] = { kind = "enter", k = 0, reason = reason }
    else
      ev[#ev + 1] = { kind = "exit", k = Z.k, reason = reason, dur = Z.clock - Z.since, wasFpp = Z.fpp }
      Z.fpp = false
      Z.since, Z.exits = Z.clock, Z.exits + 1
    end
  elseif Z.t < 1 then
    Z.t = (Z.dur > 0) and math.min(1, Z.t + dt / Z.dur) or 1
  end
  Z.w = Z.from + (Z.target - Z.from) * CM.smoothstep(Z.t)
  wheel = floor(tonumber(wheel) or 0)
  if Z.on and wheel ~= 0 and not Z.fpp then
    local dir = wheel > 0 and 1 or -1
    local k0 = Z.k
    for _ = 1, math.min(abs(wheel), 8) do
      if Z.fpp then break end
      if dir > 0 then
        if Z.k < n - 1 then
          Z.k = Z.k + 1
          Z.steps = Z.steps + 1
          ev[#ev + 1] = { kind = "step", k = Z.k, dir = 1 }
        elseif cfg.zoomUnarmedFppHandoff == true then -- opt-in (default: the wheel stops at the last step)
          Z.k, Z.fpp = n, true
          Z.handoffs = Z.handoffs + 1
          ev[#ev + 1] = { kind = "handoff", k = n, dir = 1 }
        end
      elseif Z.k > 0 then
        Z.k = Z.k - 1
        Z.steps = Z.steps + 1
        ev[#ev + 1] = { kind = "step", k = Z.k, dir = -1 }
      end
    end
    if Z.k ~= k0 and Z.pose then CM.blendTo(Z.pose, CM.zoomUnarmedPose(cfg, Z.k), blend) end
  end
  -- live settings (steps / distances / FOVs) reach the pose of the current step
  if Z.pose then CM.blendRetarget(Z.pose, CM.zoomUnarmedPose(cfg, math.min(Z.k, n - 1))) end
  return ev
end
-- the first person handoff could not be requested (no route): back to the last third-person step
function CM.zoomUnarmedCancelHandoff(Z, cfg)
  if not Z.fpp then return false end
  Z.fpp, Z.k = false, CM.zoomUnarmedSteps(cfg) - 1
  return true
end
-- the pose is (partly) shown or still moving; also the frame the exit tween ends (exact profile FOV written)
function CM.zoomUnarmedActive(Z)
  return Z ~= nil and (Z.on or Z.w > 0 or Z.t < 1 or (Z.prevT or 1) < 1 or (Z.pose ~= nil and Z.pose.t < 1))
end
-- live profile mixed with the current step pose (pose blend between steps) by the weight
function CM.zoomUnarmedCompose(base, Z)
  if not Z or Z.w <= 0 or not Z.pose then return base end
  local zp = CM.blendCurrent(Z.pose)
  zp.sens = base.sens
  return CM.profileLerp(base, zp, Z.w)
end

-- ================================================================ aim occlusion (level A, detection only)
-- 2.31 fires the player's shots from the ACTIVE camera along its forward (probe): behind the shoulder a wall
-- between the muzzle and the aim point does not stop the shot. Ray 1 runs along the camera forward from the muzzle plane
-- (t0) to aimRange -> aim point P (hit or far end); ray 2 runs muzzle -> P; blocked = ray 2 hits clearly before P.
-- Vectors are world space, fwd is a unit vector. t0 = distance of the muzzle plane along the camera ray (0 when the muzzle
-- is behind the camera); starting ray 1 there skips V's own shoulder / weapon between the camera and the muzzle (and is
-- the shot origin cam + fwd * t0 of level B).
function CM.aimOccT0(cam, fwd, muzzle) return math.max(0, CM.dot(CM.sub(muzzle, cam), fwd)) end
-- ray 1 segment from cam + fwd * t0 to cam + fwd * range; nil when the muzzle plane is at / beyond the range
function CM.aimOccRay1(cam, fwd, t0, range)
  if t0 >= range then return nil end
  return CM.add(cam, CM.scale(fwd, t0)), CM.add(cam, CM.scale(fwd, range))
end
-- aim point from the ray 1 result (hit = distance from the ray 1 start, nil = no hit: the far end) -> P, t (distance of P
-- from the camera along fwd)
function CM.aimOccPoint(cam, fwd, t0, range, hit)
  local t = range
  if hit ~= nil then t = math.min(range, t0 + math.max(0, hit)) end
  return CM.add(cam, CM.scale(fwd, t)), t
end
-- decision: hit2 = distance of the ray 2 hit from the muzzle (nil = clear) -> blocked, dMuzzleP, reason
-- ("P behind the muzzle" | "clear" | "hit at P" (within aimOccTol) | "blocked")
function CM.aimOccDecide(cfg, fwd, muzzle, P, hit2)
  local dMP = CM.dist(P, muzzle)
  if CM.dot(CM.sub(P, muzzle), fwd) <= (cfg.aimOccMinAhead or 0.1) then return false, dMP, "P behind the muzzle" end
  if hit2 == nil then return false, dMP, "clear" end
  if hit2 < dMP - (cfg.aimOccTol or 0.15) then return true, dMP, "blocked" end
  return false, dMP, "hit at P"
end
-- one sample with ray(from, to) -> hit distance from `from` | nil (ray 2 only when P is in front of the muzzle) ->
-- { raw, reason, t0, t, p, dMuzzleP, dMuzzleHit, rays }
function CM.aimOccSample(cfg, cam, fwd, muzzle, ray)
  local range = cfg.aimRange or 60
  local t0 = CM.aimOccT0(cam, fwd, muzzle)
  local info = { t0 = t0, rays = 0 }
  local a, b = CM.aimOccRay1(cam, fwd, t0, range)
  local hit1 = nil
  if a then hit1 = ray(a, b); info.rays = 1 end
  info.p, info.t = CM.aimOccPoint(cam, fwd, a and t0 or range, range, hit1)
  info.dMuzzleP = CM.dist(info.p, muzzle)
  if a and CM.dot(CM.sub(info.p, muzzle), fwd) > (cfg.aimOccMinAhead or 0.1) then
    info.dMuzzleHit = ray(muzzle, info.p)
    info.rays = 2
  end
  local _
  info.raw, _, info.reason = CM.aimOccDecide(cfg, fwd, muzzle, info.p, info.dMuzzleHit)
  return info
end
-- flag state: blocked (after hysteresis), pend (sample clock of the first differing raw result), since (clock of the last
-- edge), last (clock of the last sample, nil = sample on the next frame), frames (frames since the last sample)
function CM.newAimOcc() return { blocked = false, pend = nil, since = 0, last = nil, frames = 0, samples = 0, blocks = 0, info = nil } end
-- throttle (call once per frame while sampling is allowed): true when the two rays are due this frame - at least
-- aimOccMinFrames frames and 1 / aimOccHz s after the previous sample (the first frame always samples)
function CM.aimOccDue(st, cfg, clock)
  st.frames = st.frames + 1
  if st.last == nil then return true end
  if st.frames < (cfg.aimOccMinFrames or 2) then return false end
  local hz = cfg.aimOccHz or 30
  return hz <= 0 or clock - st.last >= 1 / hz - 1e-6
end
-- one sample result with hysteresis on both edges: the raw result must differ from the flag for aimOccHold s of sample
-- clock before the flag flips. Returns "blocked" | "clear" | nil, seconds in the previous state
function CM.aimOccStep(st, cfg, clock, raw, info)
  st.last, st.frames, st.samples, st.info = clock, 0, st.samples + 1, info
  raw = raw and true or false
  if raw == st.blocked then st.pend = nil; return nil end
  if st.pend == nil then st.pend = clock end
  if clock - st.pend + 1e-9 < (cfg.aimOccHold or 0.1) then return nil end
  local dur = clock - st.since
  st.blocked, st.pend, st.since = raw, nil, clock
  if raw then st.blocks = st.blocks + 1 end
  return raw and "blocked" or "clear", dur
end
-- sampling stopped (aim weight <= aimOccMinW, setting off, camera inactive, session reset): flag off at once.
-- Returns "clear", seconds blocked when it was blocked
function CM.aimOccReset(st, clock)
  local was, dur = st.blocked, clock - (st.since or clock)
  st.blocked, st.pend, st.last, st.frames, st.info = false, nil, nil, 0, nil
  if was then st.since = clock; return "clear", dur end
  return nil
end

-- ================================================================ stance filter (port of adapter::FilteredStance)
-- The PSM Locomotion of the game says Crouch 1 for a CrouchSprint too (LocomotionDetailed 30), while the DLL plays
-- that as Stand + Sprint (PsmCrouchStance = Crouch 3 only) and filters a stance change over adapter::kStanceFilterSeconds (the PSM flickers
-- Crouch 3 <-> CrouchSprint 30 every 0.13 s while crouch + sprint are held). The camera follows the same stance, with the same filter,
-- so the pivot height and the body agree (P7: the camera sat at the crouch height through whole runs; P1: the filter itself must not delay a
-- clean crouch / stand-up).
-- st = CM.newStanceFilter(); out, bypassed = CM.stanceFilterStep(st, cfg, raw (bool crouch), det (LocomotionDetailed), speed (m/s), dt)
-- Rules as in MotionAdapter.hpp: slide frames (det 5) and the first frame after them pass through and make the value provisional (replaced at
-- once by the opposite value of any detail but CrouchSprint 30 / Dodge 7 until it held the filter time); a Dodge (det 7) reads as the last
-- raw value before it; the pending change is drained, not reset, by a frame of the old value; a CLEAN edge (see cfg.stanceCleanHold) is
-- accepted on its own frame (bypassed = true).
function CM.newStanceFilter()
  return { out = nil, cand = 0, prov = false, conf = 0, rawLast = nil, runVal = nil, runAge = 0, detLast = nil, slideAct = false, exitAge = 1e9 }
end
local CLEAN_DET = { [1] = true, [2] = true, [3] = true }
function CM.stanceFilterStep(st, cfg, raw, det, speed, dt)
  raw = raw == true
  det = det or 0
  local fT = cfg.stanceFilter or 0
  local slideNow = det == 5
  local pass = slideNow or st.slideAct
  if det == 7 and st.out ~= nil then raw = st.rawLast ~= nil and st.rawLast or st.out else st.rawLast = raw end
  local rawEdge = raw ~= st.runVal
  local prevRunAge = rawEdge and st.runAge or 0
  local detLast = st.detLast
  st.runAge = rawEdge and dt or st.runAge + dt
  st.runVal, st.detLast = raw, det
  local out, bypassed = nil, false
  if st.out == nil or pass or not (fT > 0) then
    st.out, st.cand, st.prov, st.conf = raw, 0, pass and fT > 0, 0
    out = raw
  elseif raw == st.out then
    st.cand = math.max(0, st.cand - dt)
    if st.prov then
      st.conf = st.conf + dt
      if st.conf >= fT then st.prov = false end
    end
    out = raw
  elseif st.prov and det ~= 30 and det ~= 7 then
    -- a one-feed value right after a slide is not held by the filter: the opposite value of any state but the known flicker replaces it
    st.out, st.cand, st.prov = raw, 0, false
    out = raw
  elseif cfg.stanceCleanEdge ~= false and rawEdge and not st.prov and not st.slideAct and st.exitAge >= (cfg.stanceExitMap or 0.6)
    and CLEAN_DET[det] and CLEAN_DET[detLast] and prevRunAge >= (cfg.stanceCleanHold or fT)
    and (not raw or (speed or 0) < (cfg.stanceCleanMaxSpeed or 3.0)) then
    st.out, st.cand = raw, 0
    out, bypassed = raw, true
  else
    st.prov = false
    st.cand = st.cand + dt
    if st.cand >= fT then st.out, st.cand = raw, 0 end
    out = st.out
  end
  -- slide bookkeeping after the decision (the filter reads the previous frame's values, like MapState::slideActive / slideExitAge)
  st.exitAge = (st.slideAct and not slideNow) and 0 or st.exitAge + dt
  st.slideAct = slideNow
  return out, bypassed
end
-- distance flown while a jerk limited deceleration profile stops the speed w (m/s, > 0): the acceleration a0 (m/s2, positive = braking, may be
-- negative) goes to the plateau ap at rate jerk, stays, and goes to 0 at rate jerk exactly when the speed is 0. nil: the speed does not reach
-- the plateau (the peak ap is too high for this speed).
function CM.stopDist(w, a0, ap, jerk)
  local sg = ap >= a0 and 1 or -1
  local t1 = math.abs(ap - a0) / jerk
  local w1 = w - 0.5 * (a0 + ap) * t1
  local w3 = ap * ap / (2 * jerk)
  if w1 < w3 - 1e-9 then return nil end
  local s1 = w * t1 - a0 * t1 * t1 / 2 - sg * jerk * t1 * t1 * t1 / 6
  local s2 = ap > 1e-9 and ((w1 + w3) / 2) * ((w1 - w3) / ap) or 0
  return s1 + s2 + ap * ap * ap / (6 * jerk * jerk)
end
-- the plateau peak (0 .. amax) of the profile that stops the speed w from the acceleration a0 in exactly the distance d; nil when even amax
-- cannot stop it in d. Bisection (the stop distance falls as the peak grows up to sqrt(jerk * w + a0^2 / 2)).
function CM.stopPeak(w, a0, d, jerk, amax)
  local lo, hi = 0, math.min(amax, math.sqrt(jerk * w + 0.5 * a0 * a0)) -- (a higher peak does not reach its plateau: the triangle is the shortest stop)
  local dh = CM.stopDist(w, a0, hi, jerk)
  if dh ~= nil and dh > d then return nil end
  for _ = 1, 30 do
    local mid = 0.5 * (lo + hi)
    local dm = CM.stopDist(w, a0, mid, jerk)
    if dm == nil or dm <= d then hi = mid else lo = mid end
  end
  return hi
end
-- jump-out landing (cfg.jumpOutGov): one frame of the latched jump-out mode of the capsule pivot base (see the cfg block). The base
-- is pv.base with its speed pv.vb; pv.joA = the smoothed acceleration, pv.joG = the ground estimate (nil = none), pv.joFb = the no-ground
-- spring is running, pv.joLeft = s since the input went off. playerPos = V, vV = V's vertical speed, want = the mode input (jo.on),
-- ray(from, to) -> distance|nil or nil. Writes pv.base / pv.vb; ends itself (pv.joOn = nil) once the input is off and the base settled on V.
-- The follower is an explicit integration: it runs in sub-steps of at most cfg.jumpOutSubStep s (a frame of 1/60 s or less is one step, exactly as
-- before), so a hitch of any length stays stable. The ground probe and the clamps / end test run once per frame.
function CM.jumpOutStep(pv, cfg, dt, playerPos, vV, want, ray)
  local z = playerPos.z
  if dt <= 1e-4 then return end
  if not pv.joOn then pv.joOn, pv.joA, pv.joG, pv.joFb, pv.joLeft, pv.joBr, pv.joRec, pv.joRet, pv.joCalmT = true, 0, nil, false, 0, nil, nil, nil, nil end
  -- the fast-fall mode is idle meanwhile (and cannot re-arm from stale counters after the end)
  pv.fastOn, pv.fastT, pv.fastN, pv.slowT, pv.slowN, pv.fastRamp, pv.ev = false, 0, 0, 0, 0, nil, 0
  local c, u, a, G = pv.base, pv.vb or 0, pv.joA or 0, pv.joG
  local Tp, Ta = cfg.jumpOutTp or 0.10, cfg.jumpOutTa or 0.035
  local amax, aup, abrake = cfg.jumpOutAmax or 60, cfg.jumpOutAup or 80, cfg.jumpOutAbrake or 35
  local vth, eps = cfg.jumpOutVth or 3, cfg.jumpOutEps or 0.10
  -- ground estimate: recomputed every frame of the fall (the first hit under a falling V may be the car body, not the ground); dropped once V is
  -- below it
  if G ~= nil and z < G - 0.05 then G, pv.joRec, pv.joBr = nil, true, nil end -- (V fell through the estimate: it was not the ground)
  if ray ~= nil and vV < -vth then
    local from = v3(playerPos.x, playerPos.y, z + eps)
    local len = (cfg.jumpOutRayLen or 8) + eps
    local d = ray(from, v3(playerPos.x, playerPos.y, z + eps - len))
    if d ~= nil and d <= len + 1e-3 then
      local hz = from.z - d
      if hz <= z + eps then
        -- the ground estimate fell by more than jumpOutDrop m (the first hit was a car roof, a ledge): the landing brake planned for the old one
        -- (jumpOutDropLatched ~= false: only when a landing brake is latched for the old estimate; a fall of the estimate before that - the first hit was a fender / roof of the
        -- car V just left - only replaces it, the nominal brake plan follows: one move ran its whole landing on the recovery brake at 34 m/s2)
        if G ~= nil and hz < G - (cfg.jumpOutDrop or 0.3) and (pv.joBr or cfg.jumpOutDropLatched == false) then pv.joRec, pv.joBr = true, nil end
        G = hz
        pv.joRet = nil -- (a new fall with a fresh ground estimate: the brake plan is the normal one again)
      end -- a hit above V's feet is not a floor
    end
  end
  -- (review): once the landing brake is latched it drove the base onto the ground estimate G and never looked at V again. V that landed on a
  -- curb / sill / roof edge the centre ray missed stops ABOVE G (the pivot kept lowering below V's real stance and rebounded), and a V that jumps again during
  -- the brake rises while the base still descends. V calm (vertical speed over -jumpOutCalmV) for jumpOutCalmT s and more than jumpOutLandMargin m above G:
  -- rising (over jumpOutCalmV) = the estimate is behind it, dropped (the follower, jerk limited by jumpOutRetJerk); otherwise V stands on a higher surface,
  -- G = V's height (the recovery brake stops the base there, at the jerk limit). jumpOutRetarget = false: the previous behaviour exactly.
  if cfg.jumpOutRetarget ~= false then
    local calmV = cfg.jumpOutCalmV or 1.5
    if vV > -calmV then pv.joCalmT = (pv.joCalmT or 0) + dt else pv.joCalmT = 0 end
    if G ~= nil and (pv.joCalmT or 0) >= (cfg.jumpOutCalmT or 0.04) and z > G + (cfg.jumpOutLandMargin or 0.08) then
      if vV > calmV then
        G, pv.joBr, pv.joRet = nil, nil, true
      else
        G, pv.joBr, pv.joRec, pv.joRet = z, nil, true, true
      end
    end
  end
  local fb = pv.joFb
  if G ~= nil then fb = false end
  local dtI = math.min(dt, cfg.jumpOutDtMax or 0.5) -- a longer frame (load, pause) is not integrated; the lag clamp below bounds the base
  local nSub = math.max(1, math.ceil(dtI / math.max(cfg.jumpOutSubStep or (1 / 30), 1e-4) - 1e-6))
  local h = dtI / nSub
  for _ = 1, nSub do
    if G == nil and (vV < -vth or fb == true) then
      -- no ground estimate: a critically damped spring to V (velocity continuous), floored at V once V stopped
      fb = abs(c - z) > 0.01 or abs(vV) > 0.05
      local o = 2 / math.max(cfg.jumpOutTfb or Tp, 1e-4)
      local x = o * h
      local e = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x)
      local ch = c - z
      local tmp = (u + o * ch) * h
      local un, cn = (u - o * tmp) * e, z + (ch + tmp) * e
      if abs(vV) < 0.05 and cn < z then cn, un = z, math.max(un, 0) end
      a = (un - u) / h
      c, u = cn, un
    else
      local close = (z - c) / Tp
      if (cfg.jumpOutCatch or 0) > 0 and vV > vth and close > cfg.jumpOutCatch then close = cfg.jumpOutCatch end -- rising flight: bounded catch-up
      local vd = vV + close
      if G ~= nil then vd = math.max(vd, -sqrt(2 * abrake * math.max(0, c - G))) end
      -- landing brake (jumpOutJerk > 0): the deceleration profile (a ramp up at jerk, a plateau, a ramp down to 0 together with the speed) that
      -- stops the base exactly on the ground estimate. The peak of the plateau is solved again from the state every sub-step (CM.stopPeak), so an
      -- error of the model or of the step is corrected at once; the acceleration follows it at jerk. It starts when the stop with a plateau of
      -- jumpOutDecel needs the whole remaining distance and is latched until the base stopped. When the ground estimate falls by more than
      -- jumpOutDrop (the first hit was a car roof) or V falls through it, pv.joRec switches to the stronger recovery brake (jumpOutRecJerk /
      -- jumpOutRecDecel) for the rest of the fall. Not reachable even with jumpOutAmax: the guard below.
      local brake = false
      local jerk = cfg.jumpOutJerk or 0
      if jerk > 0 and pv.joRec then jerk = cfg.jumpOutRecJerk or jerk end -- (after a ground drop: the recovery brake)
      if jerk > 0 and G ~= nil and u < 0 then
        local w, d = -u, c - G
        local aOn = pv.joRec and (cfg.jumpOutRecDecel or 26) or (cfg.jumpOutDecel or 18)
        local sd = pv.joBr and 0 or CM.stopDist(w, a, aOn, jerk)
        if pv.joBr or (d > 0.03 and sd ~= nil and sd >= d - w * h) then -- (one step ahead: the profile starts next)
          local ap = CM.stopPeak(w, a, d, jerk, amax)
          if ap == nil and pv.joRet then ap = amax end -- re-targeted onto V's height and the stop no longer fits: brake at the limit, still along the jerk limit (no one-frame step)
          if ap ~= nil then
            pv.joBr, brake = true, true
            local at = ap
            if w <= a * a / (2 * jerk) + 1e-6 then at = 0 end -- the last ramp down
            local an = a + clamp(at - a, -jerk * h, jerk * h)
            local un = u + 0.5 * (a + an) * h
            c, u, a = c + 0.5 * (u + un) * h, un, an
          end
        end
      else
        pv.joBr = nil
      end
      if not brake then
        local ad = (vd - u) / math.max(Ta, h)
        local lim = (ad > 0 and vV > vth) and aup or amax
        ad = clamp(ad, -lim, lim)
        local an = a + (ad - a) * (1 - exp(-h / math.max(Ta, 1e-4)))
        local rj = pv.joRet and (cfg.jumpOutRetJerk or cfg.jumpOutRecJerk or 0) or 0
        if rj > 0 then an = a + clamp(an - a, -rj * h, rj * h) end -- after a re-target onto V the acceleration changes at the jerk limit
        a = an
      end
      if not brake and G ~= nil and u < 0 then
        -- stopping-distance guard: the smoothed acceleration may lag behind the braking the remaining distance needs
        local need = u * u / (2 * math.max(c - G, 0.02))
        if need > a and need > (cfg.jumpOutGuard or 0.8) * abrake then
          local ag = math.min(need, amax)
          local rj = pv.joRet and (cfg.jumpOutRetJerk or cfg.jumpOutRecJerk or 0) or 0
          if rj > 0 then ag = a + clamp(ag - a, -rj * h, rj * h) end -- (also at the jerk limit after a re-target)
          a = ag
        end
      end
      if not brake then
        u = u + a * h
        c = c + u * h
      end
      if G ~= nil and c < G and vV > -vth and not pv.joRet then c, u, a = math.max(c, z), math.max(u, 0), math.max(a, 0) end -- never below the landed ground (not after a re-target onto V; the follower lifts the base)
    end
  end
  pv.joFb = fb
  local lagMax = cfg.jumpOutLagMax or 2.0
  if abs(c - z) > lagMax then
    pv.snapEv = { kind = "clamp", lag = z - c, vz = vV, dz = 0, dt = dt }
    c = z - (c < z and lagMax or -lagMax)
    u, a = vV, 0
  end
  pv.base, pv.vb, pv.joA, pv.joG = c, u, a, G
  pv.joDz, pv.joDv, pv.joDdt = z, vV, dt -- (V's height / vertical speed and the frame time of this step: the per-frame trace line of utpp_camera)
  if want then
    pv.joLeft = 0
  else
    pv.joLeft = (pv.joLeft or 0) + dt
    local calm = abs(vV) < vth
    local lagNow = abs(c - z)
    local pl = cfg.pivotLagMax or cfg.pivotSnap
    local done = false
    if lagNow < (cfg.jumpOutEndDist or 0.01) and abs(u) < (cfg.jumpOutEndSpeed or 0.05) and calm then
      done = true -- settled on V
    elseif pv.joLeft > (cfg.jumpOutHoldMax or 3) and calm and lagNow <= pl then
      done = true -- failsafe: V is not falling and the normal path has nothing to clamp (pv.base / pv.vb are carried over)
    elseif pv.joLeft > (cfg.jumpOutHoldHard or 10) then
      -- last resort (V still moves vertically 10 s after the input went off): hand over with the lag inside the normal path's bound
      if lagNow > pl then
        pv.snapEv = { kind = "clamp", lag = z - c, vz = vV, dz = 0, dt = dt }
        c = z - (c < z and pl or -pl)
        u = vV
        pv.base, pv.vb = c, u
      end
      done = true
    end
    if done then pv.joOn, pv.joA, pv.joG, pv.joFb, pv.joLeft, pv.joBr, pv.joRec, pv.joRet, pv.joCalmT = nil, nil, nil, nil, nil, nil, nil, nil, nil end
  end
end

-- exit pose governor (cfg.vehExitGov, see the cfg block). g = CM.poseGovNew(pose): the shown pose starts AT the given pose, at rest.
-- CM.poseGovStep(g, req, dt, cfg, refSpeed) -> the pose to show for the request req = { x, y, z, yaw, pitch, fov|nil } of this frame. refSpeed = V's own
-- speed (m/s, added to the speed cap: a camera carried with a moving vehicle / V is not held back). Each channel (position as a vector, yaw, pitch, FOV)
-- is a velocity follower: command = the request's velocity (fed forward) + the gap closed at most vmax with a stopping profile (no overshoot), the
-- command capped, the follower's acceleration limited; sub-steps of at most 1/120 s (frame rate independent). A request more than teleportDist m from
-- the last one, or a frame of dt <= 1e-4, restarts / repeats. g.gap = the remaining position gap (m), g.speed = the shown speed (m/s).
function CM.poseGovNew(pose)
  return { p = CM.v3(pose.x, pose.y, pose.z), v = CM.v3(0, 0, 0), yaw = pose.yaw or 0, wy = 0, pitch = pose.pitch or 0, wp = 0, fov = pose.fov, wf = 0,
           rq = { x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pose.pitch or 0, fov = pose.fov }, gap = 0, speed = 0 }
end
-- hand-off (a jump-out: V in flight, the request follows her): the governor's remaining offset to the request and its relative velocity become a
-- critically damped tail (rate w, 1/s) added to the request: x(t) = (x0 + (v0 + w x0) t) e^(-w t), continuous in position and velocity, gone within
-- about 5 / w s. CM.poseGovTail(g, w) -> tail; CM.poseGovTailAt(tail, s) -> dx, dy, dz, dyaw, dpitch (s = seconds since the hand-off).
function CM.poseGovTail(g, w)
  local rq, ff = g.rq, g.ff or CM.v3(0, 0, 0)
  return { w = w or 8, x0 = CM.v3(g.p.x - rq.x, g.p.y - rq.y, g.p.z - rq.z), v0 = CM.v3(g.v.x - ff.x, g.v.y - ff.y, g.v.z - ff.z),
           y0 = CM.wrapDeg(g.yaw - (rq.yaw or 0)), yv = g.wy - (g.fyaw or 0), p0 = g.pitch - (rq.pitch or 0), pv = g.wp - (g.fpit or 0) }
end
function CM.poseGovTailAt(tl, s)
  local w = tl.w
  local e = math.exp(-w * s)
  local function f(x0, v0) return (x0 + (v0 + w * x0) * s) * e end
  return f(tl.x0.x, tl.v0.x), f(tl.x0.y, tl.v0.y), f(tl.x0.z, tl.v0.z), f(tl.y0, tl.yv), f(tl.p0, tl.pv)
end
-- (the gap closing speed is also at most GOV_K x the gap: a linear zone near the request, no limit cycle of the square-root profile around it)
local GOV_K = 12
local function govOut(g) return { x = g.p.x, y = g.p.y, z = g.p.z, yaw = CM.wrapDeg(g.yaw), pitch = g.pitch, fov = g.fov } end
-- one scalar channel over a sub-step h: x / v follow the target tq moving at ff (units/s); vmax = gap-closing speed, cap = |command| cap, a = acceleration
local function gov1(x, v, tq, ff, vmax, cap, a, h, wrap)
  local e = tq - x
  if wrap then e = CM.wrapDeg(e) end
  local ae = math.abs(e)
  local vc = ae > 1e-9 and (e / ae) * math.min(vmax, math.sqrt(2 * 0.8 * a * ae), GOV_K * ae) or 0
  local wd = clamp(ff + vc, -cap, cap)
  v = v + clamp(wd - v, -a * h, a * h)
  return x + v * h, v
end
function CM.poseGovStep(g, req, dt, cfg, refSpeed)
  local rq = g.rq
  if dt <= 1e-4 then return govOut(g) end
  local jump = math.sqrt((req.x - rq.x) ^ 2 + (req.y - rq.y) ^ 2 + (req.z - rq.z) ^ 2)
  if jump > (cfg.teleportDist or 3) then -- a teleport of the request: the governor restarts there (no chase across the world)
    local n = CM.poseGovNew(req)
    for k, v in pairs(n) do g[k] = v end
    return govOut(g)
  end
  local vmax = cfg.vehExitGovSpeed or 5.0
  local a = math.max(cfg.vehExitGovAccel or 14, 0.1)
  local cap = vmax + clamp(refSpeed or 0, 0, cfg.vehExitGovRefMax or 15)
  local wmax, wa = cfg.vehExitGovYawRate or 90, math.max(cfg.vehExitGovYawAccel or 360, 1)
  local fmax, fa = cfg.vehExitGovFovRate or 30, math.max(cfg.vehExitGovFovAccel or 120, 1)
  local dtI = math.min(dt, 0.1) -- (a stalled frame is not integrated beyond 0.1 s: the gap is closed on the next frames)
  local fvx, fvy, fvz = (req.x - rq.x) / dt, (req.y - rq.y) / dt, (req.z - rq.z) / dt
  local fs = math.sqrt(fvx * fvx + fvy * fvy + fvz * fvz)
  if fs > cap then fvx, fvy, fvz = fvx * cap / fs, fvy * cap / fs, fvz * cap / fs end
  local ryaw0, rpit0 = rq.yaw or 0, rq.pitch or 0
  local dyawR = CM.wrapDeg((req.yaw or 0) - ryaw0)
  local fyaw, fpit = dyawR / dt, ((req.pitch or 0) - rpit0) / dt
  local fov0, fov1 = rq.fov, req.fov
  if g.fov == nil or fov1 == nil then g.fov, g.wf = fov1, 0 end
  local ffov = (fov0 and fov1) and (fov1 - fov0) / dt or 0
  local n = math.max(1, math.ceil(dtI / (1 / 120) - 1e-6))
  local h = dtI / n
  local p, v = g.p, g.v
  local px, py, pz, vx, vy, vz = p.x, p.y, p.z, v.x, v.y, v.z
  for i = 1, n do
    local s = (i - 1) / n -- (the request at the start of the sub-step, like the follower's state: the step lands on it at the end)
    local qx, qy, qz = rq.x + (req.x - rq.x) * s, rq.y + (req.y - rq.y) * s, rq.z + (req.z - rq.z) * s
    local ex, ey, ez = qx - px, qy - py, qz - pz
    local d = math.sqrt(ex * ex + ey * ey + ez * ez)
    local k = d > 1e-9 and math.min(vmax, math.sqrt(2 * 0.8 * a * d), GOV_K * d) / d or 0
    local wx, wy, wz = fvx + ex * k, fvy + ey * k, fvz + ez * k
    local wl = math.sqrt(wx * wx + wy * wy + wz * wz)
    if wl > cap then wx, wy, wz = wx * cap / wl, wy * cap / wl, wz * cap / wl end
    local dx, dy, dz = wx - vx, wy - vy, wz - vz
    local dl = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dl > a * h then dx, dy, dz = dx * a * h / dl, dy * a * h / dl, dz * a * h / dl end
    vx, vy, vz = vx + dx, vy + dy, vz + dz
    px, py, pz = px + vx * h, py + vy * h, pz + vz * h
    g.yaw, g.wy = gov1(g.yaw, g.wy, ryaw0 + dyawR * s, fyaw, wmax, wmax, wa, h, true)
    g.pitch, g.wp = gov1(g.pitch, g.wp, rpit0 + ((req.pitch or 0) - rpit0) * s, fpit, wmax, wmax, wa, h, false)
    if g.fov ~= nil and fov1 ~= nil then g.fov, g.wf = gov1(g.fov, g.wf, (fov0 or fov1) + (fov1 - (fov0 or fov1)) * s, ffov, fmax, fmax, fa, h, false) end
  end
  -- the gap closed and the speeds match the request: the request itself from here (exact, no residual drift)
  local gx, gy, gz = req.x - px, req.y - py, req.z - pz
  local gap = math.sqrt(gx * gx + gy * gy + gz * gz)
  if gap < 1e-3 and math.sqrt((vx - fvx) ^ 2 + (vy - fvy) ^ 2 + (vz - fvz) ^ 2) < 0.05 then px, py, pz, vx, vy, vz, gap = req.x, req.y, req.z, fvx, fvy, fvz, 0 end
  if math.abs(CM.wrapDeg((req.yaw or 0) - g.yaw)) < 0.01 and math.abs(g.wy - fyaw) < 0.5 then g.yaw, g.wy = req.yaw or 0, fyaw end
  if math.abs((req.pitch or 0) - g.pitch) < 0.01 and math.abs(g.wp - fpit) < 0.5 then g.pitch, g.wp = req.pitch or 0, fpit end
  if g.fov ~= nil and fov1 ~= nil and math.abs(fov1 - g.fov) < 0.01 and math.abs(g.wf - ffov) < 0.5 then g.fov, g.wf = fov1, ffov end
  g.p, g.v = CM.v3(px, py, pz), CM.v3(vx, vy, vz)
  g.gap, g.speed = gap, math.sqrt(vx * vx + vy * vy + vz * vz)
  g.ff, g.fyaw, g.fpit = CM.v3(fvx, fvy, fvz), fyaw, fpit -- (the request's own velocity of this frame: the hand-off tail, CM.poseGovTail)
  g.rq = { x = req.x, y = req.y, z = req.z, yaw = req.yaw or 0, pitch = req.pitch or 0, fov = req.fov }
  return govOut(g)
end


-- jump-out pivot xy (cfg.jumpOutXY, see the cfg block). pv.xyJ = nil (the pivot xy is V's) | { cx, cy, ux, uy, fx, fy } (the follower: position,
-- velocity, low passed V velocity). pv.xyLx / xyLy = V's xy of the last frame. act = the jump-out mode of the pivot runs: it creates the follower from V's
-- last position and velocity (no step); without it a running follower goes on until it sat on V. Returns the pivot x, y. A frame with dt = 0 returns the
-- result of the last one.
function CM.jumpOutXYStep(pv, cfg, dt, playerPos, act)
  local px, py = playerPos.x, playerPos.y
  local lx, ly = pv.xyLx, pv.xyLy
  local S = pv.xyJ
  if cfg.jumpOutXY == false then pv.xyJ, pv.xyLx, pv.xyLy = nil, px, py; return px, py end
  if lx == nil or dt <= 1e-4 then
    if lx == nil then pv.xyLx, pv.xyLy = px, py end
    if S == nil then return px, py end
    return S.cx, S.cy -- (a repeated frame: nothing moves)
  end
  pv.xyLx, pv.xyLy = px, py
  local step = math.sqrt((px - lx) ^ 2 + (py - ly) ^ 2)
  if step > (cfg.teleportDist or 3) or step > (cfg.pivotSnapSpeed or 50) * dt then pv.xyJ = nil; return px, py end -- (a teleport: the pivot is V)
  local vx, vy = (px - lx) / dt, (py - ly) / dt
  if S == nil then
    if not act then return px, py end
    S = { cx = lx, cy = ly, ux = vx, uy = vy, fx = vx, fy = vy } -- (V's last xy and velocity: no step, no kick)
    pv.xyJ = S
  end
  local om = 2 / math.max(cfg.jumpOutXYTime or 0.25, 1e-3)
  local Tv = math.max(cfg.jumpOutXYVel or 0.05, 1e-4)
  local L = cfg.jumpOutXYLag or 0.35
  local Amax = math.max(cfg.jumpOutXYAmax or 40, 1)
  local Lh = math.max(cfg.jumpOutXYLagHard or 1.0, L)
  local dtI = math.min(dt, cfg.jumpOutDtMax or 0.5)
  local n = math.max(1, math.ceil(dtI / (1 / 120) - 1e-6)) -- (sub-steps of at most 1/120 s: the result hardly depends on the frame rate)
  local h = dtI / n
  local kf = 1 - exp(-h / Tv)
  local cx, cy, ux, uy, fx, fy = S.cx, S.cy, S.ux, S.uy, S.fx, S.fy
  for i = 1, n do
    local qx, qy = lx + (px - lx) * (i - 1) / n, ly + (py - ly) * (i - 1) / n -- (V's xy at the start of the sub-step, like the follower's state)
    fx, fy = fx + (vx - fx) * kf, fy + (vy - fy) * kf
    local ax, ay = om * om * (qx - cx) + 2 * om * (fx - ux), om * om * (qy - cy) + 2 * om * (fy - uy)
    -- stopping guard: the offset to V grows at the relative speed w; when a stop at jumpOutXYAmax m/s^2 would end beyond the lag bound L, the offset's growth is braked
    -- at that deceleration (the pivot may pass L by the braking distance, at most to jumpOutXYLagHard): the hard clamp below stays a safety net instead of a wall
    local ox, oy = cx - qx, cy - qy
    local od = math.sqrt(ox * ox + oy * oy)
    if od > 1e-6 then
      local nx, ny = ox / od, oy / od
      local w = (ux - vx) * nx + (uy - vy) * ny
      if w > 0 then
        local r = (od + w * w / (2 * Amax) - L) / math.max(0.5 * L, 1e-3) -- (the braking comes in over half the lag bound: no step of the acceleration)
        if r > 0 then
          local have = -(ax * nx + ay * ny)
          if have < Amax then
            local add = math.min(r, 1) * (Amax - have)
            ax, ay = ax - add * nx, ay - add * ny
          end
        end
      end
    end
    ux, uy = ux + ax * h, uy + ay * h
    cx, cy = cx + ux * h, cy + uy * h
  end
  -- the lag clamp: the part of the relative velocity that would grow the lag is removed
  local dx, dy = px - cx, py - cy
  local d = math.sqrt(dx * dx + dy * dy)
  if d > Lh then
    local ex, ey = dx / d, dy / d
    cx, cy = px - ex * Lh, py - ey * Lh
    local rr = (ux - vx) * ex + (uy - vy) * ey
    if rr < 0 then ux, uy = ux - rr * ex, uy - rr * ey end
    d = Lh
  end
  if not act and d < 1e-4 and (ux - vx) ^ 2 + (uy - vy) ^ 2 < 1e-4 then pv.xyJ = nil; return px, py end -- sat on V
  S.cx, S.cy, S.ux, S.uy, S.fx, S.fy = cx, cy, ux, uy, fx, fy
  return cx, cy
end

-- jump-out landing framing (cfg.jumpOutFrame, see the cfg block). st = the follower state { b = tilt (deg, + = up), w = its rate (deg/s),
-- tg = the last target, armed, vf = V's low passed vertical speed }, pv = the pivot state after CM.pivotStep of this frame (joOn, joDv, joG, base, vb, pvz),
-- cam = the camera position of this frame, playerPos = V, weight = 0..1 (the target is scaled by it). Returns the tilt to add to the shown pitch (deg);
-- exactly 0 while nothing runs. A frame with dt <= 1e-4 returns the last tilt.
function CM.jumpOutFrameStep(st, pv, cfg, dt, cam, playerPos, weight)
  if cfg.jumpOutFrame == false then
    st.b, st.w, st.tg, st.armed, st.vf = 0, 0, nil, nil, nil
    return 0
  end
  if dt <= 1e-4 then return st.b or 0 end
  local raw = pv.pvz
  if type(raw) == "number" then
    st.vf = st.vf and (st.vf + (raw - st.vf) * (1 - exp(-dt / math.max(cfg.jumpOutFrameVelT or 0.03, 1e-4)))) or raw
  end
  local tg = 0
  if pv.joOn and cam ~= nil and playerPos ~= nil and type(pv.base) == "number" then
    if (pv.joDv or 0) < -(cfg.jumpOutVth or 3) then st.armed = true end -- (the descent: the rise and the apex keep their own framing)
    if st.armed then
      local dx, dy = playerPos.x - cam.x, playerPos.y - cam.y
      local hd = sqrt(dx * dx + dy * dy)
      if hd >= 0.3 then
        local hc = cfg.jumpOutFrameHeight or 1.3
        local vz, bz, cz = playerPos.z, pv.base, cam.z
        local tau, G = cfg.jumpOutFrameLead or 0, pv.joG
        if tau > 0 and G ~= nil then
          local vf = st.vf or 0
          if vf < 0 then vz = math.max(G, math.min(vz + vf * tau - 0.5 * (cfg.jumpOutFrameGrav or 15) * tau * tau, vz)) end
          local bp = math.max(G, bz + (pv.vb or 0) * tau + 0.5 * (pv.joA or 0) * tau * tau) -- (the base's own speed and braking)
          cz, bz = cz + (bp - bz), bp -- (the camera moves with the base)
        end
        tg = deg(atan2(vz + hc - cz, hd) - atan2(bz + hc - cz, hd))
      end
    end
  else
    st.armed = nil
  end
  tg = tg * clamp(weight or 1, 0, 1)
  local b, w = st.b or 0, st.w or 0
  if b == 0 and w == 0 and tg == 0 then st.tg = 0; return 0 end
  local Ra, Rb = math.max(cfg.jumpOutFrameRate or 90, 0), math.max(cfg.jumpOutFrameRateBack or 55, 0)
  local A = math.max(cfg.jumpOutFrameAccel or 2000, 1)
  local K = math.max(cfg.jumpOutFrameGain or 20, 0)
  local tg0 = st.tg or 0
  local dtI = math.min(dt, 0.1) -- (a stalled frame is not integrated beyond 0.1 s)
  local n = math.max(1, math.ceil(dtI / (1 / 120) - 1e-6))
  local h = dtI / n
  for i = 1, n do
    local err = tg0 + (tg - tg0) * i / n - b
    local ae = abs(err)
    local R = ((err > 0) == (b < 0)) and Rb or Ra -- (towards 0: the slower return)
    local vt = ae > 1e-9 and (err / ae) * math.min(R, sqrt(2 * A * ae), K * ae) or 0
    w = w + clamp(vt - w, -A * h, A * h)
    b = b + w * h
  end
  if tg == 0 and abs(b) < 1e-3 and abs(w) < 0.05 then b, w = 0, 0 end -- (settled: exactly 0 again)
  st.b, st.w, st.tg = b, w, tg
  return b
end

-- ================================================================ pivot
function CM.newPivot() return { z = nil, vz = 0, base = nil, vb = 0, crouchW = nil } end
-- XY follows the player directly, Z springs to the eye height (stairs, crouch) and snaps on large jumps (teleport).
-- pivotSource "capsule": Z = player Z (spring, snap) + stance height (stand / crouch blended over stanceBlend s);
-- the FPP camera (eyeZ) is ignored. crouch = PSM crouch state (bool).
-- slide (bool, optional): the seated slide has its own, lower pivot (cfg.slideHeight, blended over cfg.slideBlend).
-- Entering: the pivot of the moment (the stance height, or a blend still running) is the start of one blend of slideBlend s to the
-- slide height (stand -> slide is one fall of 0.93 m, not a crouch blend plus a slide blend at once = 5.8 m/s; the crouch weight
-- is held during the slide). Leaving: one blend of slideBlend s from the pivot of the moment to the stance height of the new
-- state (crouch 1.05 / stand 1.55), also for a slide interrupted before the blend ended. Each is a single smoothstep, so the
-- move is monotonic (the held crouch weight and the falling slide weight together made a 3-6 cm hump, and
-- a dip, at the exit to the crouch).
-- aimW (0..1, optional): the lowest pivot is only for the aim camera (1.58 m boom, the head fell out of the frame): the slide
-- pivot height is the crouch height at aimW = 0 and cfg.slideHeight at aimW = 1 (linear, like the aim pose tween of the boom;
-- during the slide the aim weight is rate-limited to slideBlend s per 0..1, pv.aimS; held in the exit blend).
-- The free camera (2.8 m boom) frames the sliding V well at the crouch pivot; at 0.62 m it sat 0.35 m over the floor and the
-- floor ray pushed it in to 0.85 m from V's head when looking up 10 deg (offline floor model).
-- pv.slideW = the blend parameter of the slide / exit blend only (it restarts with each blend). CM.solve fades the aimed slide's
-- arm limit with the pivot HEIGHT of the moment (continuous across entry, exit, re-entry and stance changes), not with it.
-- fast (bool, optional): the stance change that started the current crouch <-> stand move was a clean in-place edge: the
-- pivot waits cfg.crouchDelay / standDelay and then blends over cfg.crouchBlend / standBlend (the body's clip), else over cfg.stanceBlend
-- at once. Latched when the target changes.
-- air (bool, optional; P7 F2): the player is in the air (PSM LocomotionDetailed Fall / Jump / SlideFall ...): only used while the exit blend of
-- the slide pivot runs (cfg.slideAirExitBlend, cfg.slideAirExitSmooth).
-- pin (bool, optional; ragdoll camera, CM.ragdollPivot): the base is V's own height every frame (no spring, no fast mode, no snap /
-- clamp event, vertical velocity 0): V's z during a ragdoll episode is the body tumbling over the ground, the capsule pivot is not shown and
-- must not run its fast-fall logic on it; the first frame after the episode starts from V's z.
-- jo (table, optional; jump-out landing, CM.jumpOutStep): { on = the vehicle exit hold is on, ray = raycast(from, to) -> distance|nil
-- or nil }. While on, and after it until the base settled on V, the jump-out mode replaces the fast-fall mode and the lag clamp (cfg.jumpOutGov).
-- nil / off: the previous behaviour exactly.
function CM.pivotStep(pv, cfg, dt, playerPos, eyeZ, crouch, slide, aimW, fast, air, pin, jo)
  if cfg.pivotSource ~= "fpp" then
    local slideH = cfg.slideHeight or cfg.crouchHeight
    local sl = slide == true and slideH < cfg.crouchHeight
    local cw = (crouch and not sl) and 1 or 0
    local function stance(w) return cfg.standHeight + (cfg.crouchHeight - cfg.standHeight) * CM.smoothstep(w) end
    local blend = (cfg.slideBlend or 0) > 0 and dt / cfg.slideBlend or 1
    if pv.crouchW == nil then pv.crouchW, pv.cwTarget = cw, cw
    elseif not sl and pv.slideMode == nil then
      if pv.cwTarget ~= cw then pv.cwTarget, pv.cwAge, pv.cwFast = cw, 0, fast == true end
      local delay, blendT = 0, cfg.stanceBlend or 0
      if pv.cwFast then
        if cw == 1 then delay, blendT = cfg.crouchDelay or 0, cfg.crouchBlend or blendT
        else delay, blendT = cfg.standDelay or 0, cfg.standBlend or blendT end
      end
      if (pv.cwAge or 0) >= delay then
        local step = blendT > 0 and dt / blendT or 1
        if pv.crouchW < cw then pv.crouchW = math.min(cw, pv.crouchW + step) else pv.crouchW = math.max(cw, pv.crouchW - step) end
      end
      pv.cwAge = (pv.cwAge or 0) + dt
    end
    local hh
    -- slide mode: nil = stance, "slide" (entering / in the slide), "exit" (blend back to the stance)
    if sl and pv.slideMode ~= "slide" then
      -- the pivot of the last frame is where the blend starts (a stance or an exit blend in progress); the first frame of the
      -- module starts in the slide at once
      pv.slideBase = pv.hh or stance(pv.crouchW)
      pv.slideW = pv.hh == nil and 1 or 0
      pv.slideMode, pv.slideExit = "slide", nil
    elseif not sl and pv.slideMode == "slide" then
      pv.slideMode, pv.slideExit = "exit", { from = pv.hh, w = 0, cw = cw }
      pv.crouchW = cw -- the stance of the new state is the exit blend's target
    end
    -- the aim weight reaches the pivot through its own rate limit (full travel in slideBlend s) while the slide lasts: aim and
    -- slide starting together (slide, aimW 0 -> 1 within 0.2 s) was one 3.3-3.6 m/s fall with the aim tween
    -- at full speed (2.75 m/s with the limit; the crouch alone was 3.0); outside the slide it follows aimW at once
    local aw = clamp(aimW or 0, 0, 1)
    if pv.slideMode == nil or pv.aimS == nil then pv.aimS = aw
    elseif pv.slideMode == "slide" then
      if pv.aimS < aw then pv.aimS = math.min(aw, pv.aimS + blend) else pv.aimS = math.max(aw, pv.aimS - blend) end
    end -- in the exit blend: held - an aim that starts there must not pull the pivot down and back
    if pv.slideMode == "slide" then
      pv.slideW = math.min(1, pv.slideW + blend)
      local slideTarget = cfg.crouchHeight + (slideH - cfg.crouchHeight) * pv.aimS
      hh = pv.slideBase + (slideTarget - pv.slideBase) * CM.smoothstep(pv.slideW)
    elseif pv.slideMode == "exit" then
      local ex = pv.slideExit
      if cw ~= ex.cw then -- crouch <-> stand during the exit: a new blend from the pivot of the moment
        ex.from, ex.w, ex.cw = pv.hh, 0, cw
        pv.crouchW = cw
      end
      ex.w = math.min(1, ex.w + ((air == true and (cfg.slideAirExitBlend or 0) > 0) and dt / cfg.slideAirExitBlend or blend))
      hh = ex.from + (stance(cw) - ex.from) * CM.smoothstep(ex.w)
      pv.slideW = 1 - ex.w
      if ex.w >= 1 then pv.slideMode, pv.slideExit, pv.slideW = nil, nil, 0 end
    else
      hh = stance(pv.crouchW)
      pv.slideW = 0
    end
    -- fast fall (see cfg.pivotLagMax .. pivotFastRamp): V's own one-frame z step, vertical speed and the lag bound
    local z = playerPos.z
    local lastZ = pv.lastPz
    local stepZ = lastZ ~= nil and (z - lastZ) or nil
    local vzNow = pv.pvz or 0
    if stepZ ~= nil and dt > 1e-4 then vzNow = stepZ / dt end
    pv.pvz, pv.lastPz = vzNow, z
    local lagMax = cfg.pivotLagMax or cfg.pivotSnap
    -- pivotFast = false: exactly the old rule (the base snaps to V when it is further than pivotSnap, then the plain spring); the own-step
    -- teleport test and the lag clamp belong to the fast-fall mode
    local oldSnap = cfg.pivotFast == false
    local joNow = jo ~= nil and jo.on == true and cfg.jumpOutGov ~= false and not oldSnap
    local tele = stepZ ~= nil and abs(stepZ) > cfg.pivotSnap
      and (abs(stepZ) > (cfg.teleportDist or 3) or abs(stepZ) > (cfg.pivotSnapSpeed or 50) * dt)
    if pin == true then
      pv.base, pv.vb, pv.pvz, pv.lastPz = z, 0, 0, z
      pv.fastOn, pv.fastT, pv.fastN, pv.slowT, pv.slowN, pv.fastRamp, pv.ev, pv.snapEv = false, 0, 0, 0, 0, nil, 0, nil
      pv.joOn, pv.joA, pv.joG, pv.joFb, pv.joLeft = nil, nil, nil, nil, nil
      pv.xyJ = nil
    elseif pv.base == nil or (oldSnap and abs(z - pv.base) > cfg.pivotSnap) or (not oldSnap and (tele or (stepZ == nil and abs(z - pv.base) > cfg.pivotSnap))) then
      -- first frame or a teleport (V's own step, not the accumulated lag): base = V
      if pv.base ~= nil then pv.snapEv = { kind = "teleport", lag = z - pv.base, vz = vzNow, dz = stepZ or 0, dt = dt } end
      pv.base, pv.vb = z, 0
      pv.fastOn, pv.fastT, pv.fastN, pv.slowT, pv.slowN, pv.fastRamp, pv.ev = false, 0, 0, 0, 0, nil, 0
      pv.joOn, pv.joA, pv.joG, pv.joFb, pv.joLeft = nil, nil, nil, nil, nil
      pv.xyJ = nil
    elseif joNow or pv.joOn then
      -- the jump-out mode (armed by the vehicle exit hold before the launch, kept through the apex and the landing until the base sat on V)
      CM.jumpOutStep(pv, cfg, dt, playerPos, vzNow, joNow, jo ~= nil and jo.ray or nil)
    else
      local ref = lastZ or z -- the V height pv.base belongs to
      -- fast mode: armed by a sustained vertical speed, left after the speed was low for as long
      local fast = pv.fastOn == true
      if cfg.pivotFast ~= false and (cfg.pivotFastSmooth or 0) > 0 then
        local vth, armT = cfg.pivotFastVz or 3, cfg.pivotFastArm or 0.03
        if abs(vzNow) > vth then
          pv.fastT, pv.fastN, pv.slowT, pv.slowN = (pv.fastT or 0) + dt, (pv.fastN or 0) + 1, 0, 0
        else
          pv.fastT, pv.fastN, pv.slowT, pv.slowN = 0, 0, (pv.slowT or 0) + dt, (pv.slowN or 0) + 1
        end
        if not fast and pv.fastN >= 2 and pv.fastT >= armT and pv.slideMode == nil then -- not while a slide / its exit blend runs (P7 F2 keeps its own air spring)
          fast, pv.fastOn, pv.ev, pv.fastRamp = true, true, 0, nil
        elseif fast and pv.slowN >= 2 and pv.slowT >= armT then
          -- left: V's speed is the base's speed again (V stopped at the touchdown: the base keeps only the offset's own velocity)
          fast, pv.fastOn, pv.fastRamp, pv.vb = false, false, 0, vzNow + (pv.ev or 0)
        end
      elseif fast then
        fast, pv.fastOn, pv.fastRamp = false, false, nil
      end
      -- the lag bound: a continuous clamp instead of the snap, the base keeps V's vertical velocity. Spring: the distance to V of
      -- this frame (as the old snap measured it); fast mode: the offset to V of the last frame (the base moves with V)
      local lag = fast and (ref - pv.base) or (z - pv.base)
      if not oldSnap and abs(lag) > lagMax then
        pv.snapEv = { kind = "clamp", lag = lag, vz = vzNow, dz = stepZ or 0, dt = dt }
        local sgn = lag > 0 and 1 or -1
        if fast then pv.base = ref - sgn * lagMax else pv.base = z - sgn * lagMax end
        pv.vb, pv.ev = vzNow, 0
      end
      if fast then
        local e, ev = CM.smoothDamp(pv.base - ref, 0, pv.ev or 0, cfg.pivotFastSmooth, dt)
        pv.base, pv.ev, pv.vb = z + e, ev, vzNow + ev
      else
        local smooth = cfg.pivotSmooth
        local fastS = cfg.slideAirExitSmooth or 0
        if fastS > 0 and fastS < smooth then
          if air == true and pv.slideMode == "exit" then
            pv.airRamp, smooth = 0, fastS -- the exit blend runs in the air: the base follows the take-off
          elseif pv.airRamp ~= nil then
            pv.airRamp = pv.airRamp + dt -- after it: the spring relaxes back to pivotSmooth over slideAirExitRamp s
            local u = (cfg.slideAirExitRamp or 0) > 0 and clamp(pv.airRamp / cfg.slideAirExitRamp, 0, 1) or 1
            smooth = fastS + (smooth - fastS) * u
            if u >= 1 then pv.airRamp = nil end
          end
        end
        if pv.fastRamp ~= nil then -- after the fast mode: pivotFastSmooth -> pivotSmooth over pivotFastRamp s
          local fs = cfg.pivotFastSmooth or smooth
          pv.fastRamp = pv.fastRamp + dt
          local u = (cfg.pivotFastRamp or 0) > 0 and clamp(pv.fastRamp / cfg.pivotFastRamp, 0, 1) or 1
          if fs < smooth then smooth = math.min(smooth, fs + (cfg.pivotSmooth - fs) * u) end
          if u >= 1 then pv.fastRamp = nil end
        end
        pv.base, pv.vb = CM.smoothDamp(pv.base, z, pv.vb, smooth, dt)
      end
    end
    pv.z, pv.vz, pv.height, pv.hh = pv.base + hh, 0, hh, hh
    -- the pivot xy (the jump-out follower / its decay; V's xy otherwise)
    local pxy, pyy = playerPos.x, playerPos.y
    if pin ~= true and (pv.joOn == true or pv.xyJ ~= nil) then pxy, pyy = CM.jumpOutXYStep(pv, cfg, dt, playerPos, pv.joOn == true)
    else pv.xyLx, pv.xyLy = pxy, pyy end
    return v3(pxy, pyy, pv.z)
  end
  pv.base, pv.crouchW, pv.slideW, pv.aimS, pv.slideMode, pv.slideExit, pv.slideBase, pv.hh = nil, nil, nil, nil, nil, nil, nil, nil
  pv.lastPz, pv.pvz, pv.fastOn, pv.fastT, pv.fastN, pv.slowT, pv.slowN, pv.fastRamp, pv.ev = nil, nil, nil, nil, nil, nil, nil, nil, nil
  pv.joOn, pv.joA, pv.joG, pv.joFb, pv.joLeft = nil, nil, nil, nil, nil
  pv.xyJ, pv.xyLx, pv.xyLy = nil, nil, nil
  local h = (eyeZ ~= nil) and (eyeZ - playerPos.z) or nil
  if h == nil or h ~= h or h < cfg.eyeMin or h > cfg.eyeMax then h = cfg.eyeFallback end
  local target = playerPos.z + h
  if pv.z == nil or abs(target - pv.z) > cfg.pivotSnap then
    pv.z, pv.vz = target, 0
  else
    pv.z, pv.vz = CM.smoothDamp(pv.z, target, pv.vz, cfg.pivotSmooth, dt)
  end
  pv.height = h
  return v3(playerPos.x, playerPos.y, pv.z)
end

-- mount pivot continuity (cfg.mountBlend*). S = solver state (S.mshift = running offset or nil), prevPos = the
-- player position of the previous solve (nil = none), pivot = CM.pivotStep's result of this frame. arm (bool): a new jump
-- may start a blend (the mount window, the orbit shown, consecutive frames); keep (bool): a running blend continues
-- (false = abort: unmounted, exit, perspective change, feature off). Returns the pivot to use and an event table or nil:
-- { kind = "start" | "restart" | "end" | "abort", d = step m, why = string }.
-- Start: offset0 = (last shown offset) - step, so the shown pivot xy equals the last frame's exactly; shown pivot =
-- live + offset0 * (1 - smoothstep(t / mountBlendTime)) with t = 0 on the start frame; at t >= mountBlendTime the offset
-- is dropped and the pivot is the live one bit for bit. Without a running blend the pivot is returned unchanged.
function CM.mountShiftStep(S, cfg, dt, prevPos, playerPos, pivot, arm, keep)
  local ms = S.mshift
  local ev = nil
  if ms ~= nil and (not keep or cfg.mountBlend == false) then
    S.mshift, ms = nil, nil
    ev = { kind = "abort", why = cfg.mountBlend == false and "switched off" or "mount state left" }
  end
  if prevPos ~= nil and keep and cfg.mountBlend ~= false then
    local dx, dy = playerPos.x - prevPos.x, playerPos.y - prevPos.y
    local d = math.sqrt(dx * dx + dy * dy)
    if d > (cfg.teleportDist or 3) then
      if ms ~= nil then
        S.mshift, ms = nil, nil
        ev = { kind = "abort", d = d, why = "teleport" }
      end
    elseif arm and d > (cfg.mountBlendMinStep or 0.3) then
      local sx, sy = 0, 0
      if ms ~= nil then sx, sy = ms.sx, ms.sy end
      ev = { kind = ms ~= nil and "restart" or "start", d = d }
      ms = { x = sx - dx, y = sy - dy, sx = sx - dx, sy = sy - dy, t = 0, d = d }
      S.mshift = ms
    end
  end
  if ms == nil then return pivot, ev end
  local dur = cfg.mountBlendTime or 0
  local w = dur > 0 and CM.smoothstep(ms.t / dur) or 1
  if w >= 1 then
    S.mshift = nil
    return pivot, ev or { kind = "end", d = ms.d }
  end
  local k = 1 - w
  ms.sx, ms.sy = ms.x * k, ms.y * k
  ms.t = ms.t + (dt or 0)
  return v3(pivot.x + ms.sx, pivot.y + ms.sy, pivot.z), ev
end

-- ================================================================ rig
-- boom: pivot + R(yaw, pitch) * (side * s, -distance, height)
function CM.boomOffset(yawDeg, pitchDeg, side, distance, height)
  return CM.qrot(CM.qFromYawPitch(yawDeg, pitchDeg), v3(side, -distance, height))
end

-- ================================================================ collision (multi-ray, mocked offline)
-- rays: center pivot -> target; full set adds 4 rays shifted by the camera radius along the view right / up axes
function CM.collisionRays(pivot, target, yawDeg, pitchDeg, radius, full)
  local rays = { { from = pivot, to = target, kind = "center" } }
  if full then
    local qq = CM.qFromYawPitch(yawDeg, pitchDeg)
    local right, up = CM.qrot(qq, v3(1, 0, 0)), CM.qrot(qq, v3(0, 0, 1))
    for _, o in ipairs({ CM.scale(right, radius), CM.scale(right, -radius), CM.scale(up, radius), CM.scale(up, -radius) }) do
      rays[#rays + 1] = { from = CM.add(pivot, o), to = CM.add(target, o), kind = "edge" }
    end
  end
  return rays
end
-- body clearance from the pivot (cfg minArm, at least hardMin)
function CM.clearance(cfg) return math.max(cfg.hardMin or 0, cfg.minArm or 0) end
-- hits[i] = distance from the ray start to the hit (m) or nil; returns the allowed boom length, hit count and inside
-- floor = the clearance, lowered to (nearest raw hit - clearanceWallGap) when a surface is nearer (never
-- behind it), never above the boom of this frame (fullLen < clearance during the eye-start blend: as before), never below hardMin
-- (the previous floor, returned as before even above fullLen); inside = the boom (at most fullLen) is below the clearance
function CM.collisionLength(hits, nrays, fullLen, cfg)
  local allowed, count, nearest = fullLen, 0, nil
  for i = 1, nrays do
    local d = hits[i]
    if d ~= nil then
      count = count + 1
      allowed = math.min(allowed, d - cfg.margin)
      if nearest == nil or d < nearest then nearest = d end
    end
  end
  local clr = CM.clearance(cfg)
  local lo = clr
  if nearest ~= nil then lo = math.min(lo, nearest - (cfg.clearanceWallGap or 0)) end
  local placed = clamp(allowed, math.max(math.min(lo, fullLen), cfg.hardMin), fullLen)
  return placed, count, math.min(placed, fullLen) < clr - 1e-9
end
function CM.newCollision() return { len = nil, frame = 0, lastHit = -1e9, clock = 0, hits = 0, press = 0, inV = 0 } end
function CM.collisionPlan(cs, cfg)
  cs.frame = cs.frame + 1
  return (cs.clock - cs.lastHit) < cfg.fullHold or (cs.frame % math.max(1, cfg.fullEvery)) == 0
end
-- closer: immediately (no clipping); farther: exponential return with a dead zone
-- inRate (m/s, optional): limits the pull-in of the boom (nil / 0 = instant, as always). A vehicle mount hold that still
-- shows the orbit (before the pull-out) passes it - V next to a big car's body shortened the boom 1.15 m in one frame
-- (Columbus / Emperor parallel entries, 0.3 s after the mount)
-- inDelay (s, optional): with inRate, the pull-in starts after the boom was pressed for that long (leaky timer) and ramps up with
-- cfg.vehHoldColAccel to inRate (a swinging door crossing the boom for 0.25 s is no occluder); nil / 0 = the pull-in at inRate at once, as before
-- inMin (m, optional): with inRate, floor of the pull-in (cfg.vehHoldColMin): the boom never comes closer to the pivot than that
-- inLift (m/s, optional): with inRate, the boom lengthens by at most that rate, and a boom shorter than inMin (with an occluder: hitCount > 0) rises to it
function CM.collisionSmooth(cs, cfg, dt, allowed, hitCount, inRate, inDelay, inMin, inLift)
  cs.clock = cs.clock + dt
  cs.hits = hitCount or 0
  if (hitCount or 0) > 0 then cs.lastHit = cs.clock end
  -- inMin (m, optional): with inRate, the boom is not pulled in below this length (a boom already shorter keeps its length)
  local rawAllowed = allowed -- (the allowed length before the floor)
  local lift = (inRate and inRate > 0 and inLift and inLift > 0) and inLift or nil -- lift limit of the hold gate
  if cs.len ~= nil and inRate and inRate > 0 and inMin and inMin > 0 then
    if lift and (hitCount or 0) > 0 then allowed = math.max(allowed, inMin) -- the floor itself, a shorter boom rises to it (at most `lift` m/s, below)
    else allowed = math.max(allowed, math.min(inMin, cs.len)) end
  end
  if inRate and inRate > 0 and cfg.vehHoldColCarry ~= false then cs.inCarry = inRate end -- the rate of the gate that just ran (carried until the boom reached the allowed length)
  if cs.len == nil then
    cs.len = allowed
    cs.inCarry = nil
  elseif allowed < cs.len then
    if inRate and inRate > 0 then
      local delay, acc = inDelay or 0, cfg.vehHoldColAccel or 0 -- inDelay (s, nil / 0 = off): the delayed, ramped pull-in of a standing vehicle's mount hold
      if delay > 0 then cs.press = (cs.press or 0) + dt end
      if delay > 0 and cs.press < delay then
        cs.inV = 0 -- pressed for less than the delay: the boom stays where it is
      else
        cs.inV = (delay > 0 and acc > 0) and math.min(inRate, (cs.inV or 0) + acc * dt) or inRate
        cs.len = math.max(allowed, cs.len - cs.inV * dt)
      end
    elseif cs.inCarry then
      -- the gate dropped (no rate any more) with the boom still longer than the allowed length: the same rate limited pull-in, ramped as before, until it is there
      local acc = cfg.vehHoldColAccel or 0
      cs.inV = acc > 0 and math.min(cs.inCarry, (cs.inV or 0) + acc * dt) or cs.inCarry
      cs.len = math.max(allowed, cs.len - cs.inV * dt)
      if cs.len <= allowed then cs.inCarry, cs.inV = nil, 0 end
    else
      cs.len = allowed
    end
  else
    if rawAllowed >= cs.len then cs.inCarry = nil end -- the occluder is gone (the boom is at / inside the allowed length; held by the floor it is not)
    -- inDelay mode: while the occluder is still there (a hit, the boom at its allowed length within the dead zone) the press timer and the pull-in speed stay -
    -- a further intrusion (the boom turning into the body, a creeping surface) is no new door pulse and must not wait the delay again; they empty only when the hit clears
    local held = (inDelay or 0) > 0 and (hitCount or 0) > 0 and (cs.press or 0) > 0 and allowed - cs.len <= cfg.deadzone
    if not held then
      if cs.press and cs.press > 0 then cs.press = math.max(0, cs.press - 2 * dt) end
      cs.inV = 0
    end
    if allowed - cs.len > cfg.deadzone then
      local nl = CM.approach(cs.len, allowed, 3 / math.max(cfg.returnTime, 1e-3), dt)
      if lift then nl = math.min(nl, cs.len + lift * dt) end -- the lengthening of the mount hold is rate limited as well
      cs.len = nl
    end
  end
  return cs.len
end

-- ================================================================ perspective state machine
-- States: fpp -> rep_wait -> entering -> tpp -> leaving -> settle -> fpp. Weight w: 0 = camera in the eyes, 1 = boom.
-- env: enabled (UTPP camera allowed), blocked (reason|nil: forces first person), cut (blocked reason wants a cut),
--      freeze (menu / photo mode: nothing changes), canEnter (component present), repReady (representation ready)
-- Returns a list of commands: rep_on, cam_on, fpp_on, rep_off, cam_off (executed in this order by the adapter).
function CM.newTransition(useRep)
  return { state = "fpp", w = 0, epoch = 0, want = false, quick = false, timer = 0, useRep = useRep ~= false,
           instantEnter = false, enters = 0, exits = 0, last = "init" }
end
function CM.trRequest(T, want, why)
  if T.want ~= want then T.epoch = T.epoch + 1 end
  T.want = want and true or false
  T.last = why or (want and "request tpp" or "request fpp")
end
function CM.trQuick(T, down)
  T.quick = down and true or false
  T.epoch = T.epoch + 1
  if not down then T.instantEnter = true end
end
function CM.trStep(T, cfg, dt, env)
  local out = {}
  if env.freeze then return out end
  local blocked = (not env.enabled and "disabled") or env.blocked or (T.quick and "quick") or nil
  local cut = (not env.enabled) or env.cut or T.quick
  local wantTpp = T.want and not blocked
  local s = T.state
  if s == "fpp" then
    T.w = 0
    if wantTpp and env.canEnter then
      T.epoch = T.epoch + 1
      if T.useRep then
        out[#out + 1] = "rep_on"
        T.state, T.timer = "rep_wait", 0
      else
        out[#out + 1] = "cam_on"
        T.state = "entering"
        T.enters = T.enters + 1
      end
    end
  elseif s == "rep_wait" then
    T.timer = T.timer + dt
    if not wantTpp then
      out[#out + 1] = "rep_off"
      T.state = "fpp"
    elseif env.repReady and T.timer >= cfg.repWait or T.timer >= cfg.repWaitMax then
      out[#out + 1] = "cam_on"
      T.state = "entering"
      T.enters = T.enters + 1
    end
  elseif s == "entering" or s == "tpp" or s == "leaving" then
    if wantTpp then
      if T.instantEnter then T.w = 1 end
      T.w = math.min(1, T.w + dt / math.max(T.enterOverride or cfg.enterTime, 1e-3))
      T.state = (T.w >= 1) and "tpp" or "entering"
    elseif cut then
      T.w = 0
    else
      T.w = math.max(0, T.w - dt / math.max(cfg.exitTime, 1e-3))
      T.state = "leaving"
    end
    if not wantTpp and T.w <= 0 then
      out[#out + 1] = "fpp_on"
      out[#out + 1] = "rep_off"
      T.state = "settle"
      T.exits = T.exits + 1
    end
  elseif s == "settle" then
    out[#out + 1] = "cam_off" -- one frame after the FPP camera took over (design 2.5 step 4)
    T.state = "fpp"
  end
  -- the instant re-entry of a quick-FPP release survives the settle/fpp frames (release one frame after the press)
  if T.state == "tpp" then T.instantEnter = false end
  -- per-transition enter time (return from aiming) only for this entry
  if T.state == "tpp" or not T.want then T.enterOverride = nil end
  T.blocked = blocked
  return out
end
function CM.trActive(T) return T.state == "entering" or T.state == "tpp" or T.state == "leaving" end

-- ================================================================ ragdoll camera (cfg.ragdollCam / rag*)
-- inp.ragdoll of CM.solve (nil = no ragdoll natives / no episode input: everything below stays out of the way):
--   state (0 idle / 1 ragdoll owns the body / 2 blend-out, Game.UTPPRagdollState), knock (bool: PSM knockdown get-up still playing),
--   pelvis (the Hips in V's model space, Game.UTPPRagdollPelvis) + fresh (bool: w = 1, i.e. younger than the DLL's window),
--   noLook (bool: a vehicle hold owns the look; only the pivot / boom are overridden).
function CM.newRagdoll()
  return { on = false, t = 0, endT = 0, bw = 0, out = nil, p = nil, v = nil, model = nil, staleT = 0, blocked = false, snap = false,
           noLook = false }
end
-- Episode state machine, once per solve before the look. Returns phase ("on" | "out" | nil) and an event {kind = "enter" | "exit"}.
-- Enter: state >= 1 only. Held while state >= 1 or the PSM knockdown (r.knock) lasts; ends ragEndHold s after both are over (or at
-- ragMaxTime, or when the natives / the switch go away) and is followed by the exit blend (phase "out") of ragExitBlend s.
function CM.ragdollPhase(S, cfg, dt, r, teleport)
  local R = S.rag
  if R == nil then
    if r == nil or cfg.ragdollCam == false then return nil, nil end
    R = CM.newRagdoll()
    S.rag = R
  end
  local enabled = cfg.ragdollCam ~= false and r ~= nil
  local st = r and tonumber(r.state) or 0
  local held = enabled and (st >= 1 or r.knock == true)
  if r ~= nil then R.noLook = r.noLook == true end
  local ev
  if not held then R.blocked = false end
  if R.on then
    R.t = R.t + dt
    if held then R.endT = 0 else R.endT = R.endT + dt end
    if teleport then R.p, R.snap = nil, true end -- the player was moved far (respawn / load): the spring restarts on its target
    local capped = R.t >= (cfg.ragMaxTime or 12)
    if not enabled or capped or R.endT >= (cfg.ragEndHold or 0.1) then
      R.on = false
      if capped and held then R.blocked = true end -- a state stuck at 1 / 2 does not start a second episode
      local ex = (cfg.ragExitBlend or 0) > 0 -- (also when the natives / the switch went away: a blend, never a cut)
      R.out = ex and { t = 0 } or nil
      ev = { kind = "exit", t = R.t, why = (not enabled) and "natives gone / switched off" or (capped and "time cap") or "get-up over",
             blend = ex and cfg.ragExitBlend or 0 }
    end
  elseif enabled and st >= 1 and not R.blocked then
    R.on, R.t, R.endT, R.out, R.p, R.v, R.snap, R.staleT = true, 0, 0, nil, nil, nil, teleport == true, 0
    R.ret, R.rw = nil, 0
    ev = { kind = "enter", state = st }
  end
  -- early return inside the running episode (see cfg.ragReturnEarly): latched at the trigger, weight R.rw = smoothstep(t / ragReturnTime)
  if R.on and ev == nil and enabled and cfg.ragReturnEarly ~= false and (cfg.ragReturnTime or 0) > 0 then
    if R.ret == nil then
      local g = r.getup
      if type(g) == "table" then
        local gs = tonumber(g.state) or 0
        local fr = tonumber(g.frac)
        if gs == 3 or (gs == 2 and fr ~= nil and fr >= (cfg.ragReturnFrac or 0.45)) then
          R.ret, R.rw = { t = 0 }, 0
          ev = { kind = "return", state = gs, frac = fr, t = R.t, time = cfg.ragReturnTime }
        end
      end
    else
      R.ret.t = R.ret.t + dt
      R.rw = CM.smoothstep(R.ret.t / cfg.ragReturnTime)
    end
  end
  if R.out ~= nil and ev == nil then
    R.out.t = R.out.t + dt
    if teleport or R.out.t >= (cfg.ragExitBlend or 0) then R.out = nil end -- (a far move ends the crossfade: the capsule pivot at once)
  end
  -- the boom weight (distance / pitch bias): in while the episode runs, out over ragBlendOut afterwards
  if R.on then
    local bi = cfg.ragBlendIn or 0
    R.bw = bi > 0 and math.min(1, R.bw + dt / bi) or 1
    if R.ret ~= nil then R.bw = math.min(R.bw, 1 - (R.rw or 0)) end -- early return: the boom weight falls with the return weight
  else
    local bo = cfg.ragBlendOut or 0
    R.bw = bo > 0 and math.max(0, R.bw - dt / bo) or 0
  end
  return (R.on and "on") or (R.out ~= nil and "out") or nil, ev
end
-- The ragdoll pivot of this frame (phase "on" / "out" of CM.ragdollPhase). capsule = the normal pivot (CM.pivotStep, mount shift).
-- World pelvis = player position + player rotation * (model pelvis) + (0, 0, ragLift). A stale pelvis keeps its last model-space point
-- (moved with the entity) and rises to ragStandPelvis over ragRise s. Phase "on": the spring's position; phase "out": a crossfade from
-- it to the capsule pivot (the spring keeps tracking meanwhile).
function CM.ragdollPivot(S, cfg, dt, phase, r, playerPos, playerQ, capsule)
  local R = S.rag
  local fresh = r ~= nil and r.fresh == true and r.pelvis ~= nil
  if fresh then
    R.model, R.staleT = v3(r.pelvis.x, r.pelvis.y, r.pelvis.z), 0
  elseif R.model ~= nil then
    R.staleT = R.staleT + dt
  end
  local tgt
  if R.model ~= nil then
    local mz = R.model.z
    if not fresh and R.staleT > 0 then
      local rise = cfg.ragRise or 0
      local u = rise > 0 and CM.smoothstep(R.staleT / rise) or 1
      mz = mz + ((cfg.ragStandPelvis or 1.0) - mz) * u
    end
    local w = CM.qrot(playerQ or CM.QID, v3(R.model.x, R.model.y, mz))
    tgt = v3(playerPos.x + w.x, playerPos.y + w.y, playerPos.z + w.z + (cfg.ragLift or 0))
  else
    tgt = capsule -- never a fresh pelvis: the normal pivot
  end
  -- the world pelvis of this frame for the framing constraint (CM.ragdollFrame); nil without a model pelvis (the target is the capsule pivot then)
  R.pelvisW = R.model ~= nil and v3(tgt.x, tgt.y, tgt.z - (cfg.ragLift or 0)) or nil
  -- early return: the target crosses from the pelvis point to the capsule pivot (in the exit blend too: the spring keeps tracking it)
  if R.ret ~= nil and (R.rw or 0) > 0 then tgt = CM.vlerp(tgt, capsule, R.rw) end
  if R.p == nil then
    local lp = S.lastPivot
    if R.snap or lp == nil then
      R.p, R.v = v3(tgt.x, tgt.y, tgt.z), v3(0, 0, 0)
    else
      local lv = S.lastPivotV or v3(0, 0, 0)
      R.p, R.v = v3(lp.x, lp.y, lp.z), v3(lv.x, lv.y, lv.z)
    end
    R.snap = false
  end
  local T = (r ~= nil and tonumber(r.state) == 1) and (cfg.ragSmoothActive or 0.15) or (cfg.ragSmoothGetUp or 0.25)
  -- blend-in: a slower spring at the start of the episode, easing to T over ragEnterBlend s
  local eb0, es0 = cfg.ragEnterBlend or 0, cfg.ragEnterSmooth or 0
  if phase == "on" and eb0 > 0 and es0 > T and R.t < eb0 then
    local u = CM.smoothstep(R.t / eb0)
    T = es0 + (T - es0) * u
  end
  local p, v = R.p, R.v
  p.x, v.x = CM.smoothDamp(p.x, tgt.x, v.x, T, dt)
  p.y, v.y = CM.smoothDamp(p.y, tgt.y, v.y, T, dt)
  p.z, v.z = CM.smoothDamp(p.z, tgt.z, v.z, T, dt)
  if phase == "out" and R.out ~= nil then
    local eb = cfg.ragExitBlend or 0
    local w = eb > 0 and CM.smoothstep(R.out.t / eb) or 1
    return CM.vlerp(p, capsule, w), tgt
  end
  return v3(p.x, p.y, p.z), tgt
end

-- framing constraint (cfg.ragFrame, see DEFAULTS): the world shift of the pivot that keeps V's head / pelvis inside the safe band of
-- the camera of this frame. cam = pivot + off, q = the shown orientation, fovV = vertical FOV (deg). nil = no shift (inside the band, or no usable
-- point). Per axis the shift is (largest overshoot to one side) + (largest overshoot to the other side): continuous, a body too tall for the band
-- gets the compromise. Points more than 0.2 m behind the image plane are skipped.
function CM.ragdollFrame(cfg, pivot, off, q, fovV, pelvisW)
  if pelvisW == nil or type(fovV) ~= "number" or fovV <= 1 then return nil end
  local cam = CM.add(pivot, off)
  local right, fwd, up = CM.qrot(q, v3(1, 0, 0)), CM.qrot(q, v3(0, 1, 0)), CM.qrot(q, v3(0, 0, 1))
  local margin = cfg.ragFrameMargin or 0.75
  local tv = math.tan(rad(fovV * 0.5)) * margin
  local th = tv * (cfg.ragFrameAspect or (16 / 9))
  local pts = { v3(pelvisW.x, pelvisW.y, pelvisW.z + (cfg.ragHeadUp or 0.75)), pelvisW, v3(pelvisW.x, pelvisW.y, pelvisW.z - (cfg.ragLowDown or 0.25)) }
  local rPos, rNeg, uPos, uNeg = 0, 0, 0, 0
  for _, pt in ipairs(pts) do
    local rel = CM.sub(pt, cam)
    local f = CM.dot(rel, fwd)
    if f > 0.2 then
      local r, u = CM.dot(rel, right), CM.dot(rel, up)
      rPos = math.max(rPos, r - th * f); rNeg = math.min(rNeg, r + th * f)
      uPos = math.max(uPos, u - tv * f); uNeg = math.min(uNeg, u + tv * f)
    end
  end
  local sr, su = rPos + rNeg, uPos + uNeg
  if sr == 0 and su == 0 then return nil end
  return CM.add(CM.scale(right, sr), CM.scale(up, su))
end

-- early return: the yaw increment (deg, added to this frame's mouse yaw) that eases the free orbit onto V's facing over the return time.
-- Decided once at the first call after the trigger: only when the orbit is more than ragRecenterMin deg off bodyYaw. Each frame takes the fraction
-- (e - ePrev) / (1 - ePrev) of the remaining angle, e = the return weight: the profile is the smoothstep of the angle, frame rate independent,
-- no overshoot. Off unless cfg.ragRecenter == true; look input (dyaw / dpitch, deg of this frame) above cfg.ragRecenterInput cancels it for the
-- rest of the return (latched: the player's view is never pulled back afterwards).
function CM.ragdollRecenter(S, cfg, bodyYaw, dyaw, dpitch)
  local R = S.rag
  if R == nil or R.ret == nil or cfg.ragRecenter ~= true or type(bodyYaw) ~= "number" then return 0 end
  local cur = S.look.freeYaw
  if type(cur) ~= "number" then return 0 end
  local rt = R.ret
  local thr = cfg.ragRecenterInput or 0
  if math.abs(dyaw or 0) > thr or math.abs(dpitch or 0) > thr then rt.rc = false end -- look input: the recenter is over for this return
  if rt.rc == nil then
    rt.rc = math.abs(CM.wrapDeg(bodyYaw - cur)) > (cfg.ragRecenterMin or 15)
    rt.ePrev = R.rw or 0
  end
  if not rt.rc then return 0 end
  local e = R.rw or 0
  local d = 0
  if rt.ePrev < 1 and e > rt.ePrev then d = CM.wrapDeg(bodyYaw - cur) * (e - rt.ePrev) / (1 - rt.ePrev) end
  rt.ePrev = e
  return d
end

-- ================================================================ solver (one frame)
-- S = CM.newSolver(cfg); in = { dt, playerPos, playerQ, bodyYaw, eyeZ, fppPitch, dyaw, dpitch, freeHeld, moving,
--   profile (blended), side (shoulder s), w (transition weight), zoom (m), raycast(from, to) -> distance|nil,
--   crouch (PSM crouch state after CM.stanceFilterStep, pivotSource "capsule"), stanceFast (the last change was a clean in-place
--   edge: CM.pivotStep crouchDelay / crouchBlend / standDelay / standBlend), air (the player is in the air: CM.pivotStep slideAirExit*), slide (the seated slide, also crouch = true; own pivot
--   height cfg.slideHeight, scaled in by aimW: the free camera keeps the crouch pivot), followSnap (follow limiter off: vehicle hold),
--   aim (forced follow, CM.lookStep env.aim), aimW (aim weight), fovDirect (FOV = target, no fovRate lag),
--   decoupled (free world yaw), refYaw (aim reference: the game's look yaw, nil = body yaw),
--   lookYaw (the game's look yaw while synced: free / own aim yaw source, nil = own mouse integration),
--   zoomOnly (the aim pose shown is the unarmed zoom: zoomBoomPitchScale / zoomBoomFrame), lookEcho
--   (fppPitch echoes the game look written last frame: CM.lookStep env.lookEcho), lookWriteOn (the game's look is
--   written this frame: own aim / zoom pitch and yaw, CM.lookStep env.lookWriteOn; adjustPending: the game's look
--   of this frame is utpp_bodyturn's adjustTransform override, CM.lookStep env.adjustPending), zoomShare (0..1: share of the
--   unarmed zoom in the aim pose shown, ZU.w / (A.w + ZU.w); the boom lag of both rule sets blended by it; nil = zoomOnly),
--   jumpOut (bool: the vehicle exit hold is on, the pivot base follows V with the jump-out landing mode, CM.jumpOutStep; raycast is its
--   ground probe) }
-- returns { world = pos, q = quat, localPos, localQ, fov, yaw, pitch, arm, full, hits, pivot, flip (event|nil), teleport }
function CM.newSolver(cfg)
  return { cfg = cfg or CM.config(), look = CM.newLook(), pivot = CM.newPivot(), col = CM.newCollision(), fov = nil }
end
function CM.solve(S, inp)
  local cfg, dt = S.cfg, inp.dt
  local p = inp.profile
  local teleport = S.lastPos ~= nil and CM.dist(S.lastPos, inp.playerPos) > (cfg.teleportDist or 3)
  local prevPos = S.lastPos
  S.lastPos = inp.playerPos
  -- ragdoll camera: the episode phase ("on" | "out" | nil) before the look; during it the orbit keeps its world yaw (holdYaw),
  -- has its own pitch (ownPitch) and does not read the game's look (lookYaw nil) - unless a vehicle hold owns the look (r.noLook)
  local ragPhase, ragEv
  if inp.ragdoll ~= nil or S.rag ~= nil then ragPhase, ragEv = CM.ragdollPhase(S, cfg, dt, inp.ragdoll, teleport) end
  local holdYawIn, ownPitchIn, lookYawIn = inp.holdYaw, inp.ownPitch, inp.lookYaw
  if ragPhase ~= nil and not S.rag.noLook then holdYawIn, ownPitchIn, lookYawIn = true, true, nil end
  local recInc = 0
  if ragPhase == "on" and S.rag.ret ~= nil and not S.rag.noLook then recInc = CM.ragdollRecenter(S, cfg, inp.bodyYaw, inp.dyaw, inp.dpitch) end -- early return
  local yaw, pitch = CM.lookStep(S.look, cfg, dt, (inp.dyaw or 0) + recInc, inp.dpitch or 0,
    { bodyYaw = inp.bodyYaw, fppPitch = inp.fppPitch, freeHeld = inp.freeHeld, moving = inp.moving, teleport = teleport,
      decoupled = inp.decoupled, shooterMode = inp.shooterMode, refYaw = inp.refYaw, lookYaw = lookYawIn,
      snap = inp.followSnap, aim = inp.aim, aimW = inp.aimW, crouch = inp.crouch, lookEcho = inp.lookEcho,
      lookWriteOn = inp.lookWriteOn, adjustPending = inp.adjustPending,
      holdYaw = holdYawIn, ownPitch = ownPitchIn })
  local flip = S.look.flip
  S.look.flip = nil
  local ew = CM.smoothstep(inp.w or 1)
  -- orientation: at w = 0 it equals the FPP view (body yaw, FPP pitch) so the cut to the FPP camera is invisible
  local fppPitch = inp.fppPitch or pitch
  local oyaw = CM.wrapDeg(inp.bodyYaw + CM.wrapDeg(yaw - inp.bodyYaw) * ew)
  local opitch = fppPitch + (pitch - fppPitch) * ew
  local aw = clamp(inp.aimW or (inp.aim and 1 or 0), 0, 1)
  local pivot = CM.pivotStep(S.pivot, cfg, dt, inp.playerPos, inp.eyeZ, inp.crouch, inp.slide, aw, inp.stanceFast, inp.air,
    ragPhase == "on", -- (pinned during a ragdoll episode)
    (inp.jumpOut ~= nil or S.pivot.joOn) and ragPhase ~= "on" and { on = inp.jumpOut == true, ray = inp.raycast } or nil) -- jump-out landing mode
  -- mount pivot continuity (inp.mountArm / inp.mountKeep, see CM.mountShiftStep); nil inputs = no blend
  local mshEv
  pivot, mshEv = CM.mountShiftStep(S, cfg, dt, prevPos, inp.playerPos, pivot, inp.mountArm == true and ragPhase ~= "on",
    inp.mountKeep == true and ragPhase ~= "on")
  -- ragdoll camera: the pivot follows the world pelvis (phase "on") / crosses over to the capsule pivot (phase "out")
  local ragTarget
  if ragPhase ~= nil then pivot, ragTarget = CM.ragdollPivot(S, cfg, dt, ragPhase, inp.ragdoll, inp.playerPos, inp.playerQ, pivot) end
  local ragW = (S.rag ~= nil and S.rag.bw > 0) and CM.smoothstep(S.rag.bw) or 0
  local dist = clamp(p.distance + (inp.zoom or 0), cfg.distMin, cfg.distMax)
  if ragW > 0 then dist = dist + (cfg.ragDistAdd or 0) * ragW end -- the boom is longer while the body lies (after the clamp: distMax does not eat it)
  -- clearance blend (cfg clearanceBlend): the boom geometry below scales with ewB = c + (1 - c) * ew, c = the
  -- clearance / the unscaled boom length (rotation-invariant), instead of ew: w = 0 is pivot + boom direction * clearance (outside
  -- the head), not the FPP eye point (inside the shown TPP head), and the boom length grows monotonically from there. The
  -- orientation and the FOV keep ew (w = 0 still looks exactly like the FPP view). ew = 1 or clearanceBlend off (default): ewB = ew.
  local ewB = ew
  local clrBlend = cfg.clearanceBlend == true
  if clrBlend and ew < 1 then
    local sd = p.side * (inp.side or 1)
    local Lu = math.sqrt(sd * sd + dist * dist + p.height * p.height)
    local c = Lu > 1e-6 and clamp(CM.clearance(cfg) / Lu, 0, 1) or 1
    ewB = c + (1 - c) * ew
  end
  -- FOV of this frame (vertical; before the boom: the aim framing limit below uses it)
  local fovTarget = p.fov
  if inp.fppFov and ew < 1 then fovTarget = inp.fppFov + (p.fov - inp.fppFov) * ew end
  if S.fov == nil or inp.fovDirect then S.fov = fovTarget else S.fov = CM.approach(S.fov, fovTarget, cfg.fovRate, dt) end
  -- (aiming up / down during the zoom / with a weapon is a drama): while aiming / zooming (and
  -- through the exit tween) the boom POSITION orbits with only aimBoomPitchScale of the pitch (looking down: ground in
  -- front over the shoulder, not V's head from above; looking up: camera near shoulder height), the ORIENTATION keeps
  -- the full look pitch (qq below; shots and crosshair come from the active camera, so the screen
  -- centre stays on the shot line). Not aiming: aw = 0 -> boomPitch = opitch (unchanged).
  -- framing limit (review: at 0.35 V left the frame past about -40 / +48): the lag (look pitch - boom pitch)
  -- is limited so the pivot (V's shoulder line) stays within aimBoomFrame of half the vertical FOV of this frame from
  -- the screen centre - at any aim pitch and zoom step; beyond that the boom orbits with the look (V held at that
  -- screen offset). b0 = the pivot's screen angle without lag (height / distance of the pose); 0 lag always allowed.
  -- Unarmed zoom (review: with the narrow zoom FOV the limit fell back to ~the full orbit, camera over V's head
  -- looking down): the unarmed zoom (inp.zoomOnly: zoom pose shown, no weapon aim) uses
  -- zoomBoomPitchScale (0.25) and zoomBoomFrame (0 = no limit: the pure partial orbit); the weapon aim keeps
  -- aimBoomPitchScale (0.35) / aimBoomFrame. A frame value <= 0 = no limit.
  -- aimBoomFrame default 0 (review: at 0.8 the weapon boom followed the look 1:1 past about -35 / +30); the two
  -- rule sets are blended by the zoom's share of the pose (inp.zoomShare) - overlapping aim / zoom tweens switched the
  -- rule set in one frame (0.4-0.8 m camera jumps at |pitch| >= 50)
  local boomPitch = opitch
  if aw > 0 then
    local zs = tonumber(inp.zoomShare)
    zs = zs and clamp(zs, 0, 1) or (inp.zoomOnly == true and 1 or 0)
    local function lagOf(scale, frame)
      local lag = opitch * (1 - (scale or 1)) * aw
      if (frame or 0) > 0 then
        local lim = frame * 0.5 * (S.fov or 60)
        local b0 = -deg(atan2(p.height * ewB, math.max(dist * ewB, 1e-3)))
        lag = clamp(lag, math.min(0, b0 - lim), math.max(0, b0 + lim))
      end
      return lag
    end
    local lag
    if zs <= 0 then lag = lagOf(cfg.aimBoomPitchScale, cfg.aimBoomFrame)
    elseif zs >= 1 then lag = lagOf(cfg.zoomBoomPitchScale, cfg.zoomBoomFrame)
    else lag = lagOf(cfg.aimBoomPitchScale, cfg.aimBoomFrame) * (1 - zs) + lagOf(cfg.zoomBoomPitchScale, cfg.zoomBoomFrame) * zs end
    boomPitch = opitch - lag
  end
  -- ragdoll camera: the boom looks further down at the body (shown pitch only; the look state L.pitch is untouched)
  local shownPitch = opitch
  if ragW > 0 and (cfg.ragPitchBias or 0) ~= 0 then
    local floorP = math.min(cfg.pitchMin or -80, opitch)
    shownPitch = math.max(opitch + cfg.ragPitchBias * ragW, floorP)
    boomPitch = boomPitch + (shownPitch - opitch)
  end
  -- (review: aimed slide, aim settings 1.5 m / 0.5 / boomFrame 0.45): the slide pivot is only 0.62 m over the
  -- floor, and looking up past ~30 deg the framing limit orbits the boom with the look - the camera goes to the floor, the floor
  -- ray pulls the arm in (1.45 m at +30, 1.13 at +35, 0.93 at +40; the head 0.55 m away, out of the frame). The boom pitch is
  -- limited so that the boom end stays cfg.radius + cfg.margin over the floor under the player (flat-floor model, z of the
  -- player's origin): R sin(pitch - atan2(h, d)) <= room. Weighted by how far the pivot of the moment is below the crouch height
  -- (0 at the crouch / stand pivots, 1 at the full slide height, aim weight and blend progress included): continuous with the
  -- pivot, also when a slide is interrupted, re-entered during its exit or the stance changes in the exit (review: the blend
  -- parameter pv.slideW restarts with each blend, the limit jumped 0.07-0.46 m in one frame); the free camera, the crouch and
  -- the stance changes are untouched (no pivot below the crouch height there).
  local slH, lw = cfg.slideHeight or cfg.crouchHeight, 0
  if S.pivot.hh ~= nil and slH < cfg.crouchHeight then
    lw = clamp((cfg.crouchHeight - S.pivot.hh) / (cfg.crouchHeight - slH), 0, 1)
  end
  if cfg.slideArmLimit ~= false and lw > 1e-9 then
    local d, h = dist * ewB, p.height * ewB
    local R = math.sqrt(d * d + h * h)
    if R > 1e-3 then
      -- the floor under the player as the pivot knows it: the smoothed base (pivotSmooth), not the raw z of the player (a step /
      -- curb / capsule step-off of 0.15 m moved the camera 0.15 m in one frame while the pivot moved 0.005 m)
      local room = pivot.z - ((S.pivot.base or inp.playerPos.z) + (cfg.radius or 0) + (cfg.margin or 0))
      local pmax = deg(math.asin(clamp(room / R, -1, 1)) + atan2(h, d))
      if boomPitch > pmax then boomPitch = boomPitch + (pmax - boomPitch) * lw end
    end
  end
  local off = CM.boomOffset(oyaw, boomPitch, p.side * (inp.side or 1) * ewB, dist * ewB, p.height * ewB)
  -- ragdoll framing constraint: V's head / pelvis stay inside the safe band of this frame's camera (pivot shift only, see CM.ragdollFrame)
  if ragPhase ~= nil and cfg.ragFrame ~= false and ew >= 0.999 and S.rag ~= nil and S.rag.pelvisW ~= nil then
    local sh = CM.ragdollFrame(cfg, pivot, off, CM.qFromYawPitch(oyaw, shownPitch), S.fov or p.fov, S.rag.pelvisW) or v3(0, 0, 0)
    -- the shift follows the wanted one at <= ragFrameRate m/s (a jump of the body data cannot cut the view; V's own speed is far below it)
    local cur, R = S.rag.shift or v3(0, 0, 0), S.rag
    local dx, dy, dz = sh.x - cur.x, sh.y - cur.y, sh.z - cur.z
    local dl = math.sqrt(dx * dx + dy * dy + dz * dz)
    local mx = (cfg.ragFrameRate or 8) * dt
    if (cfg.ragFrameRate or 8) > 0 and dl > mx then local k = mx / dl; dx, dy, dz = dx * k, dy * k, dz * k end
    cur = v3(cur.x + dx, cur.y + dy, cur.z + dz)
    R.shift = cur
    if cur.x ~= 0 or cur.y ~= 0 or cur.z ~= 0 then pivot = CM.add(pivot, cur) end
  elseif S.rag ~= nil then
    S.rag.shift = nil
  end
  local target = CM.add(pivot, off)
  local fullLen = CM.len(off)
  local arm, full, hits = fullLen, false, 0
  if cfg.collision and inp.raycast and fullLen > 1e-3 then
    full = CM.collisionPlan(S.col, cfg)
    local rays = CM.collisionRays(pivot, target, oyaw, boomPitch, cfg.radius, full) -- rays follow the actual boom
    local res = {}
    for i, r in ipairs(rays) do res[i] = inp.raycast(r.from, r.to) end
    local allowed
    allowed, hits = CM.collisionLength(res, #rays, fullLen, cfg)
    arm = CM.collisionSmooth(S.col, cfg, dt, allowed, hits, inp.colInRate, inp.colInDelay, inp.colInMin, inp.colInLift)
    arm = math.min(arm, fullLen)
  else
    S.col.len = nil
  end
  local world = pivot
  if fullLen > 1e-6 then world = CM.add(pivot, CM.scale(off, arm / fullLen)) end
  -- start the boom from the real eye point when it is known (w -> 0 converges to the FPP camera position);
  -- not with the opt-in clearanceBlend (w = 0 is the clearance point there, see ewB above)
  if not clrBlend and inp.eyePos and ew < 1 then world = CM.vlerp(inp.eyePos, world, ew) end
  -- jump-out landing framing (cfg.jumpOutFrame, CM.jumpOutFrameStep): the shown orientation only (the boom, res.pitch and the look state
  -- are untouched); res.joPitch = the tilt of this frame for the vehicle hold's own poses (utpp_camera VehicleCompose)
  local joPitch = 0
  if S.joFr ~= nil or S.pivot.joOn then
    if S.joFr == nil or teleport then S.joFr = {} end
    joPitch = CM.jumpOutFrameStep(S.joFr, S.pivot, cfg, dt, world, inp.playerPos, ragPhase == nil and ew * (1 - aw) or 0)
    if joPitch ~= 0 then shownPitch = shownPitch + joPitch elseif not S.pivot.joOn then S.joFr = nil end
  end
  local qq = CM.qFromYawPitch(oyaw, shownPitch)
  local lp, lq = CM.toLocal(inp.playerPos, inp.playerQ or CM.QID, world, qq)
  -- inside = the boom placed this frame is shorter than the body clearance (a near surface, or the eye-point
  -- blend); insideWall = a surface shortened it (arm below the boom of this frame), not only the blend. utpp_camera restores the
  -- vanilla near-camera head hide while inside (on foot)
  local inside = arm < CM.clearance(cfg) - 1e-9
  -- the shown pivot and its velocity (CM.ragdollPivot starts the spring from them)
  do
    local lpv = S.lastPivot
    if lpv ~= nil and dt > 1e-4 then
      local function cl(x) return clamp(x, -30, 30) end
      S.lastPivotV = v3(cl((pivot.x - lpv.x) / dt), cl((pivot.y - lpv.y) / dt), cl((pivot.z - lpv.z) / dt))
    else
      S.lastPivotV = v3(0, 0, 0)
    end
    S.lastPivot = v3(pivot.x, pivot.y, pivot.z)
  end
  return { world = world, q = qq, localPos = lp, localQ = lq, fov = S.fov, yaw = oyaw, pitch = opitch, arm = arm,
           fullLen = fullLen, full = full, hits = hits, pivot = pivot, flip = flip, teleport = teleport, boomPitch = boomPitch,
           shownPitch = shownPitch, -- the pitch of res.q / res.localQ (opitch + the ragdoll boom bias + joPitch); res.pitch stays the look pitch
           joPitch = joPitch, -- the jump-out landing framing tilt included in shownPitch (deg, 0 = none)
           mountShift = mshEv, inside = inside, insideWall = inside and arm < fullLen - 1e-9,
           ragdoll = (ragPhase ~= nil or ragEv ~= nil or ragW > 0) and { phase = ragPhase, ev = ragEv, w = ragW, target = ragTarget } or nil }
end

return CM

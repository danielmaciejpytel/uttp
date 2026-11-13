-- UTPP body turn executor: body yaw follows the motion matching animation.
--
-- The module selects its own executor (one log line):
--   * every CET frame while UTPP Locomotion is active (Game.UTPPLocomotionActive(); the MM target is then camera yaw + raw stick,
--     utpp.reds UTPPOwnMoveTarget) the native body turn is on; while Game.UTPPMMBodyTurnActive() the yaw from
--     Game.UTPPMMBodyYawCommand(body, body) (animation-led step + bounded fallback <= 180 deg/s, steering hysteresis - all
--     native, as in 0.3.x) is clamped to maxRate (900 deg/s) and sent to scripts/utpp_bodyturn.reds, which applies it in the
--     next player locomotion update as a vanilla AdjustTransformWithDurations rotation and moves the capsule along the
--     camera heading (LocomotionParameters.SetUseCameraHeadingForMovement);
--   * UTPP Locomotion inactive (weapon, aiming, FPP, vehicle, ...) -> native body turn off (matcher keeps the original input);
--   * camera: the camera is attached to the body with a local yaw, so the body may only turn under a camera that does not
--     turn with it (otherwise camera + stick turns with the body and V spins). While the executor drives, utpp_camera's
--     free-look mode is held (GetMod("utpp_camera").FreeLook: heading lock + own world yaw), released 0.5 s after the executor
--     stops. Guard: if the camera yaw follows the body yaw for 60 deg of executor turning, the executor pauses 5 s (after 3
--     trips: for the session), one log line each;
--   * UTPP Locomotion is switched on at selection (exec.enableLocomotion) and off again by SetExecutor(false).
-- SetExecutor(false), or utpp_bodyturn.reds not loaded -> UTPPSetMMBodyTurn(false) (the matcher keeps the original input).
-- Kill switch: Game.UTPPSetMMBodyTurn(false). Console: GetMod("utpp_bodyturn").SetExecutor(false|true),
-- print(GetMod("utpp_bodyturn").Status()), print(Game.UTPPBodyTurnExecStatus()).
--
-- setting bodyturn.weaponTurn (default ON; UTPP tab, section "Body turn"): body turn with a weapon in
-- hand. Locomotion blocks while an item sits in WeaponRight (heartbeat bit kWeapon = 1<<13 = 8192). Default ON because the
-- heartbeat weapon bit misfires (in game: reasons=0x2000 although WeaponRight was empty, Locomotion
-- never active; with the mask 0x5FBF it ran and the body turned on the run arcs). The setting sends
-- Game.UTPPLocomotionSetBlockMask(0x5FBF) (= 0x7FBF & ~8192; aiming bit 4096 still blocks) when on, 0x7FBF when off. Applied live,
-- once the executor is selected (SelectExecutor), when Locomotion turns active again and after every loading screen /
-- missing player; only with a player; on failure retried every 2 s. Console: GetMod("utpp_bodyturn").SetWeaponTurn(true|false).

local M = { VERSION = "0.6.3" }

local function norm(v)
  local n = (v + 180) % 360
  if n < 0 then n = n + 360 end
  return n - 180
end

-- ---------------------------------------------------------------- body turn executor (pure; offline-testable)
-- api (in game: gameExecApi below; tests: a mock). Every call is pcall'd.
--   arm(on) -> true when utpp_bodyturn.reds answered      setNative(on)          Game.UTPPSetMMBodyTurn
--   locomotionEnable(on)  Game.UTPPLocomotionSetEnabled        locomotionActive() -> bool  locomotionReasons() -> int
--   active() -> bool   Game.UTPPMMBodyTurnActive          bodyYaw() / cameraYaw() -> deg | nil
--   command(cur) -> deg   Game.UTPPMMBodyYawCommand(cur, cur)                apply(yaw) -> bool (reds request)
--   camHeld() -> bool | nil (nil = no utpp_camera)       camHold(on)            utpp_camera FreeLook
--   lookSynced() -> bool  utpp_camera LookSynced (the camera shows the game's look, whose world
--                         heading stays while V turns -> the aim align cannot chase it; the spin guard is not needed)
--   lookStep(step)        utpp_camera OnBodyLookStep (after every sent apply / applyAim request: V's exact turn of
--                         the coming tick for the game look write, UTPPSetGameLook entityYawAheadDeg)
--   vehicleHeld() -> bool | nil (nil = no utpp_camera / no VehicleHeld)   utpp_camera VehicleHeld: its end starts the
--                         E.vehicleSettle (0.3 s) pause of the aim align
-- execStats.adjSeq: +1 per sent adjustTransform request (apply / applyAim; lastAdjStep = its step). The game
-- overrides its FPP look in the ticks such a turn lands (look pulled onto V's yaw,
-- pitch ~0), so utpp_camera does not read the game's look back in those frames (adjustPending) and marks them (camTrace adj).
local E = {
  mode = "pending",     -- pending | executor | disabled
  enabled = true,       -- kill switch (SetExecutor)
  cameraHold = true,    -- hold utpp_camera's free-look (decoupled) camera while the executor turns the body
  enableLocomotion = true, -- switch UTPP Locomotion on when the executor is selected
  maxRate = 900,        -- deg/s safety clamp on top of the native controller (its total cap is 900 as well)
  minStep = 0.05,       -- deg: smaller steps are not sent
  linger = 0.5,         -- s the camera hold stays after the executor stops
  coupledLimit = 60,    -- deg of body turn with the camera following before the executor pauses
  coupledPause = 5,     -- s
  coupledMaxTrips = 3,  -- then paused for the session (SetExecutor(true) re-arms)
  -- the guard compared V's turn with the game's
  -- look yaw (api.cameraYaw), which FOLLOWS the body while the game look write is off (world camera failed) -> a false
  -- trip while running, and the 5 s pause froze V's yaw at 8.6 deg while Locomotion kept moving V (3.5 m/s, 'moonwalk').
  -- guardFix (setting bodyturn.guardFix, default on; off = the plain guard): the guard reads the camera shown by
  -- utpp_camera (api.viewYaw) instead of the game's look when available; with the game's look it is skipped while the
  -- look is not written (api.lookWritten == false); it never runs and a pause never holds while V moves faster than
  -- guardMoveSpeed m/s (api.speed; the executor keeps turning V, the pause resumes once V stands); the trip line carries
  -- bd / cd / score / src.
  guardFix = true,
  guardMoveSpeed = 1.0, -- m/s
  aimAlign = true,      -- turn V onto the game's look yaw while aiming (Locomotion blocked by the aim bit only)
  aimAlignRate = 1080,  -- deg/s (720 lagged behind fast mouse turns)
  aimSmooth = true,     -- bounded/eased turns, stable aim admission and episode cooldown
  aimSmoothRate = 540, aimSmoothStart = 35, aimSmoothGain = 10,
  aimStableTime = 0.1, aimCooldown = 0.3, aimStable = 0, aimUpperPrevious = false,
  aimAlignNear = 10,    -- deg: within this the spin guard counter restarts
  aimAlignDeadband = 2, -- deg: the align stops within this
  aimAlignStart = 12,   -- deg: the align starts only beyond this (hysteresis) - every adjustTransform tick
                        -- overrides the game's FPP look for that tick (pitch -> ~0, yaw -> V), so V is turned only when
                        -- the camera is clearly away; the DLL AimOffset (30 deg) covers the arm in between
  aimAlignMaxSum = 540, -- deg turned in one aim without reaching the look -> stop (spin guard)
  -- melee strike turn (setting bodyturn.meleeStrikeTurn, default off): while a fists strike runs and the game has a tracked target,
  -- V turns toward the direction the game's hit will take (api.meleeStrikeYaw, utpp_melee.reds UTPPMeleeStrikeYaw)
  strikeTurn = false,
  strikeMax = 50,       -- deg: V ends within this of the facing it had when the episode (a strike series) started; the rest stays with the tilt
  strikeRate = 540,     -- deg/s cap
  strikeGain = 20,      -- 1/s ease-out gain (time constant 0.05 s)
  strikeStartDeg = 8,   -- deg: a turn starts beyond this error ...
  strikeDeadband = 4,   -- deg: ... and ends within this one (hysteresis)
  strikeGap = 0.5,      -- s without a signal that ends an episode
  strikeMaxSpeed = 1.0, -- m/s: no strike turn in a series that starts while V moves faster (the legs are motion matching then;
                        -- the tilt covers it); the speed is V's speed before the strike, not the game's lunge during it
  strikeSum = 0, strikeStart = nil, strikeSeen = -1e9, strikeLogged = false, strikeTurning = false, strikeHeld = false,
  -- vehicle hand-over: the aim align (the body turn onto the camera) waits this long after utpp_camera ends a
  -- vehicle hold (api.vehicleHeld true -> false), so it does not land on the same frames as the release end and the aim pose;
  -- afterwards it starts through the normal hysteresis / rate. Never armed without a vehicle hold (api.vehicleHeld nil / false).
  vehicleSettle = 0.3,  -- s
  vehicleHeldPrev = false, vehicleSettleUntil = nil,
  -- runtime
  nativeOn = nil, holdOwned = false, holdTimer = 0, score = 0, pausedUntil = nil, trips = 0, clock = 0,
  prevBody = nil, prevCam = nil, prevStep = 0, locomotionOwned = false, why = "-", selected = false,
}
M.exec = E
local xs = { frames = 0, applied = 0, deadband = 0, rejected = 0, clamped = 0, cmdErrors = 0, toggles = 0, holds = 0, armFails = 0,
             lastCur = 0, lastCmd = 0, lastStep = 0, lastCam = 0, adjSeq = 0, lastAdjStep = 0 }
M.execStats = xs
-- Hipfire uses the same PSM.Weapon Shoot=8 as utpp_motion.reds (game 2.5: PSM.RightWeapon, LeftWeapon while dual wielding). This is a
-- body-heading lease only: never writes ADS, camera zoom, or the native PSM.
-- Mirrors HipFirePoseHold (mm/HipFirePose.hpp), so the lease and the DLL hip pose share one clock (the fixed 0.75 s lease
-- dropped 50 ms before the next shot of a 1 s cadence and switched the native body turn and the backward-run exclusion
-- on and off): an isolated shot leases hipFireIsolatedHold (1.2 s) after its last Shoot frame, a Shoot run that starts within
-- hipFireSustainLink (2.5 s) of the previous Shoot frame leases hipFireSustainedHold (2 s). A sprint (sig.sprint, PSM
-- LocomotionDetailed) cuts both to what is left of hipFireSprintHold (0.75 s) after the last Shoot frame; an unreadable sprint
-- state (sig.sprint nil) does the same, i.e. the former fixed 0.75 s lease. In the air / a landing / a climb, vault or ladder
-- (sig.air, PSM LocomotionDetailed; the DLL hold's grounded rule, combat probe shoot_jump) nothing is leased after a Shoot
-- frame and the engagement is forgotten. A reload / native Ready-type state keeps the engagement memory (as the DLL); unsafe/missing
-- state clears lease and memory.
E.hipFireRemaining = 0
E.hipFireIsolatedHold, E.hipFireSustainedHold, E.hipFireSustainLink, E.hipFireSprintHold = 1.2, 2.0, 2.5, 0.75
E.hipFireSince, E.hipFireInShot, E.hipFireSustained = 1e9, false, false -- s since the last Shoot frame (any attribution)
-- ADS ownership a Shoot=8 that belongs to ADS (UpperBody==6) must not start the
-- hipfire lease. The lease aligned the capsule onto the camera at 1080 deg/s (unsmoothed) for 0.75 s after the RMB release
-- while V stood still, so the capsule turned 200-250 deg away from the visible body (the offset the native root hides until
-- the next reload reveals it). Shoot=8 outlasts the RMB release by 0.015-0.18 s, so the tail of an ADS shot is attributed to
-- ADS for hipFireAdsTail seconds after the last ADS sample; a new Shoot=8 run (action left 8 in between) is hipfire again.
E.hipFireAdsShot, E.hipFireAdsAge, E.hipFireAdsTail = false, 0, 0.4
local function hipFireForget() E.hipFireSince, E.hipFireInShot, E.hipFireSustained = 1e9, false, false end
-- engagement memory of HipFirePoseHold: a Shoot run that starts within hipFireSustainLink of the previous Shoot frame is sustained
local function hipFireMemory(shot, dt)
  if shot then
    if not E.hipFireInShot then
      E.hipFireSustained = E.hipFireSince <= E.hipFireSustainLink
      E.hipFireInShot = true
    end
    E.hipFireSince = 0
  else
    E.hipFireInShot = false
    if type(dt) == "number" and dt == dt and dt > 0 then E.hipFireSince = math.min(1e9, E.hipFireSince + dt) end
  end
end
function M.HipFireStep(sig, reasons, dt)
  local safe = type(reasons) == "number" and reasons == reasons and reasons >= 0 and reasons < 1e6 and reasons % 1 == 0
  local rest = reasons or -1
  if safe then
    for _, bit in ipairs({4096, 8192}) do
      if math.floor(rest / bit) % 2 == 1 then rest = rest - bit end
    end
    if E.alignHeld and math.floor(rest / 16384) % 2 == 1 then rest = rest - 16384 end
  end
  -- ADS in first person (perspective.aiming / aimingScoped = fpp): Locomotion reports kNotTpp (65536) there, so the shot is not
  -- admitted below, but the Shoot=8 tail runs on after utpp_perspective is back in third person. The latch is armed from
  -- the FPP aim samples too (review); the lease itself stays off in FPP.
  local fppAds = safe and rest == 65536 and type(sig) == "table" and sig.weaponDrawn == true and sig.upper == 6
  if fppAds then
    E.hipFireRemaining = 0
    E.hipFireAdsShot, E.hipFireAdsAge = sig.action == 8, 0
    hipFireMemory(sig.action == 8, dt) -- the DLL hold counts every Shoot frame
    return false
  end
  local unsafe = not safe or rest ~= 0 or type(sig) ~= "table" or sig.weaponDrawn ~= true
  if unsafe
    -- Native ranged PSM may leave Shoot through Ready/Safe/NoAmmo/Overheat
    -- (or Charging for the next shot). Keep the same bounded tail as HipFirePoseHold;
    -- these states never start/refresh it. Reload and unknown actions still clear.
    or (sig.action ~= 0 and sig.action ~= 1 and sig.action ~= 4 and sig.action ~= 5
        and sig.action ~= 6 and sig.action ~= 7 and sig.action ~= 8) then
    E.hipFireRemaining = 0
    E.hipFireAdsShot, E.hipFireAdsAge = false, 0
    if not unsafe and sig.action == 2 then -- a reload ends the lease, not the engagement (HipFirePoseHold)
      hipFireMemory(false, dt)
      E.hipFireSustained = false
    else
      hipFireForget()
    end
  elseif sig.upper == 6 then
    -- ADS steers V through the smoothed aim path; a hipfire lease never runs on top of it (also clears a lease that was
    -- started by a hipfire shot just before the RMB press)
    E.hipFireRemaining = 0
    E.hipFireAdsShot, E.hipFireAdsAge = sig.action == 8, 0
    hipFireMemory(sig.action == 8, dt)
  elseif sig.air == true then
    -- HipFirePoseHold with grounded false: no hold, the engagement is forgotten; a hipfire Shoot frame in the air leases only itself
    local hip = false
    if sig.action == 8 then
      if E.hipFireAdsShot then
        E.hipFireAdsAge = E.hipFireAdsAge + dt
        if E.hipFireAdsAge > E.hipFireAdsTail then E.hipFireAdsShot = false end
      end
      hip = not E.hipFireAdsShot
    else
      E.hipFireAdsShot, E.hipFireAdsAge = false, 0
    end
    E.hipFireRemaining = 0
    hipFireForget()
    E.hipFireInShot = sig.action == 8
    return hip
  elseif sig.action == 8 then
    if E.hipFireAdsShot then
      E.hipFireAdsAge = E.hipFireAdsAge + dt
      if E.hipFireAdsAge > E.hipFireAdsTail then E.hipFireAdsShot = false end
    end
    hipFireMemory(true, dt)
    E.hipFireRemaining = E.hipFireAdsShot and 0 or (E.hipFireSustained and E.hipFireSustainedHold or E.hipFireIsolatedHold)
  else
    E.hipFireAdsShot, E.hipFireAdsAge = false, 0
    -- a sprint (or an unreadable sprint state) keeps only what is left of the 0.75 s after the last Shoot frame
    local cap = math.max(0, E.hipFireSprintHold - E.hipFireSince)
    if sig.sprint ~= false and (E.hipFireSustained or E.hipFireRemaining > cap + 1e-4) then
      E.hipFireSustained = false
      E.hipFireRemaining = math.min(E.hipFireRemaining, cap)
    end
    E.hipFireRemaining = math.max(0, E.hipFireRemaining - dt)
    hipFireMemory(false, dt)
  end
  return E.hipFireRemaining > 0
end
M.api = nil
M.maskApi = nil -- in game: { hasPlayer, setBlockMask } (works before the executor api exists)
M.log = function(msg) print(msg) end

local function finite(v) return type(v) == "number" and v == v and v > -1e6 and v < 1e6 end
local function try(fn, ...)
  if type(fn) ~= "function" then return false, nil end
  return pcall(fn, ...)
end

-- Only on change (or forced): the native UTPPSetMMBodyTurn also resets the body-yaw controller and the target latch
-- (MotionHost.cpp SetBodyTurn), so a per-frame call would break the hysteresis / yaw-rate smoothing.
local function setNative(api, on, force)
  if E.nativeOn == on and not force then return end
  if E.nativeOn ~= on then xs.toggles = xs.toggles + 1 end
  E.nativeOn = on
  try(api.setNative, on)
end

local function releaseHold(api, dt)
  if not E.holdOwned then return end
  E.holdTimer = E.holdTimer - (dt or 0)
  if E.holdTimer > 0 then return end
  try(api.camHold, false)
  E.holdOwned = false
end

local function engageHold(api)
  E.holdTimer = E.linger
  if not E.cameraHold then return end
  local ok, held = try(api.camHeld)
  if not ok or held == nil or held == true then return end -- no utpp_camera, or already free (ours or the player's key)
  local okH, r = try(api.camHold, true)
  if okH and r ~= false then
    if not E.holdOwned then xs.holds = xs.holds + 1 end
    E.holdOwned = true
  end
end

-- ---------------------------------------------------------------- body turn with a weapon (Locomotion block mask)
M.MASK_DEFAULT = 0x7FBF     -- all heartbeat bits block
M.MASK_WEAPON_TURN = 0x5FBF -- 0x7FBF & ~8192 (kWeapon); the aim bit (4096) still blocks
local W = { on = true, -- default ON (= the setting default; the stock mask would leave Locomotion dead)
  sent = nil, logged = nil, dirty = true, retryAt = 0, prevActive = nil, calls = 0, fails = 0, noPlayer = 0 }
M.weapon = W

-- Sends the mask for the current setting. api.hasPlayer() -> bool, api.setBlockMask(mask) (Game.UTPPLocomotionSetBlockMask).
-- Only while the executor is selected; only with a player; the log line appears once per change.
-- Returns true when the DLL got the mask (now or earlier, unchanged).
function M.ApplyBlockMask(api, force)
  if E.mode ~= "executor" then return false end
  api = api or M.api or M.maskApi or {}
  local mask = W.on and M.MASK_WEAPON_TURN or M.MASK_DEFAULT
  if not force and not W.dirty and W.sent == mask then return true end
  local okP, has = try(api.hasPlayer)
  if not okP or has ~= true then
    W.noPlayer, W.dirty, W.sent = W.noPlayer + 1, true, nil
    return false
  end
  local ok = try(api.setBlockMask, mask)
  if not ok then
    W.fails, W.dirty, W.sent, W.retryAt = W.fails + 1, true, nil, E.clock + 2
    return false
  end
  W.calls, W.dirty, W.sent = W.calls + 1, false, mask
  if W.logged ~= mask then
    W.logged = mask
    M.log(string.format("[utpp_bodyturn] Locomotion block mask -> 0x%X (weapon turn %s)", mask, W.on and "on" or "off"))
  end
  return true
end

-- Setting bodyturn.weaponTurn / console. Returns the switch value.
function M.SetWeaponTurn(on)
  W.on = on == true
  W.dirty = true
  M.ApplyBlockMask(nil, true)
  return W.on
end

-- turn clip rate (natives UTPPSetMMTurnRate / UTPPSetMMTurnRateParams; with
-- a cap of 1.1 it still looked slow): TR.on / TR.max sent whenever a setting changes; missing natives (older DLL) are
-- logged once and the settings stay inert.
local TR = { on = true, max = 1.5, min = 0.9, sent = nil, calls = 0, fails = 0, missing = false }
M.turnRate = TR
function M.ApplyTurnRate()
  if Game == nil or type(Game.UTPPSetMMTurnRate) ~= "function" or type(Game.UTPPSetMMTurnRateParams) ~= "function" then
    if not TR.missing then
      TR.missing = true
      M.log("[utpp_bodyturn] turn rate natives missing (UTPP.dll < 0.6.4) - bodyturn.turnRate* settings inert")
    end
    return false
  end
  TR.calls = TR.calls + 1
  local ok1 = try(Game.UTPPSetMMTurnRate, TR.on == true)
  local ok2, accepted = try(Game.UTPPSetMMTurnRateParams, TR.min, TR.max)
  if not (ok1 and ok2 and accepted ~= false) then
    TR.fails = TR.fails + 1
    M.log(string.format("[utpp_bodyturn] turn rate apply FAILED (on=%s max=%.2f accepted=%s)", tostring(TR.on), TR.max,
      tostring(accepted)))
    return false
  end
  TR.sent = { on = TR.on, max = TR.max }
  M.log(string.format("[utpp_bodyturn] turn clip rate -> %s, max %.2f (min %.2f)", TR.on and "on" or "off", TR.max, TR.min))
  return true
end

-- B2 aim strafe S1 (UTPP.dll natives UTPPSetMMStrafe / UTPPSetMMStrafeParams): while V aims with a weapon in hand
-- the legs strafe (side / back steps) and the hips / feet keep facing the aim. SF.on / SF.unarmed sent whenever a setting changes
-- (tuning params keep the native defaults 10 / 30 / 0.15 / 0); missing natives (older DLL) are logged once, the settings stay inert.
-- (jog while aiming): SF.jog / SF.jogSpeed (settings bodyturn.aimJog / bodyturn.aimJogSpeed) -> jogF = jogSpeed (3.3..3.85 m/s
-- forward; sides 0.70x, back 0.60x) or 0 (jog off = the vanilla aim walk speed; the reds MaxSpeed multiplier is removed).
-- Default ON (jog while aiming), speed 3.3 = native minimum (run loop rate
-- floor 0.6 x 5.46 m/s; back 0.60 x 3.3 = 1.98 m/s stays above the vanilla aim walk 1.65).
local SF = { on = true, unarmed = true, rootTurn = 10.0, rootWarp = 30.0, exitHold = 0.15, jogF = 0.0, jog = true, jogSpeed = 3.3,
             sent = nil, calls = 0, fails = 0, missing = false }
M.strafe = SF
function M.ApplyStrafe()
  if Game == nil or type(Game.UTPPSetMMStrafe) ~= "function" or type(Game.UTPPSetMMStrafeParams) ~= "function" then
    if not SF.missing then
      SF.missing = true
      M.log("[utpp_bodyturn] aim strafe natives missing (older UTPP.dll) - bodyturn.aimStrafe* settings inert")
    end
    return false
  end
  SF.calls = SF.calls + 1
  local js = tonumber(SF.jogSpeed) or 3.3
  if js ~= js then js = 3.3 end
  js = math.max(3.3, math.min(3.85, js))
  SF.jogF = (SF.jog == true) and js or 0.0
  local ok1 = try(Game.UTPPSetMMStrafe, SF.on == true)
  local ok2, accepted = try(Game.UTPPSetMMStrafeParams, SF.unarmed == true, SF.rootTurn, SF.rootWarp, SF.exitHold, SF.jogF)
  if not (ok1 and ok2 and accepted ~= false) then
    SF.fails = SF.fails + 1
    M.log(string.format("[utpp_bodyturn] aim strafe apply FAILED (on=%s unarmed=%s accepted=%s)", tostring(SF.on),
      tostring(SF.unarmed), tostring(accepted)))
    return false
  end
  SF.sent = { on = SF.on, unarmed = SF.unarmed, jogF = SF.jogF }
  M.log(string.format("[utpp_bodyturn] aim strafe -> %s (unarmed %s, jog %.2f)", SF.on and "on" or "off", SF.unarmed and "on" or "off",
    SF.jogF))
  return true
end

-- B2 aim strafe S2 chest align (UTPP.dll native UTPPSetMMChestAlign): while strafing, the chest twist of the side /
-- back loops is turned back towards the forward-walk chest (spine only). CA.on sent whenever the setting changes (tuning params
-- keep the native defaults swayKeep 0.5 / chestMaxDeg 60: console UTPPSetMMChestAlignParams); a missing native (older DLL) is
-- logged once, the setting stays inert.
local CA = { on = true, sent = nil, calls = 0, fails = 0, missing = false }
M.chest = CA
function M.ApplyChestAlign()
  if Game == nil or type(Game.UTPPSetMMChestAlign) ~= "function" then
    if not CA.missing then
      CA.missing = true
      M.log("[utpp_bodyturn] chest align native missing (older UTPP.dll) - bodyturn.aimChestAlign inert")
    end
    return false
  end
  CA.calls = CA.calls + 1
  if not try(Game.UTPPSetMMChestAlign, CA.on == true) then
    CA.fails = CA.fails + 1
    M.log(string.format("[utpp_bodyturn] chest align apply FAILED (on=%s)", tostring(CA.on)))
    return false
  end
  CA.sent = { on = CA.on }
  M.log(string.format("[utpp_bodyturn] aim chest align -> %s", CA.on and "on" or "off"))
  return true
end

-- ---------------------------------------------------------------- Shooter mode (setting bodyturn.shooterMode)
-- Default OFF = nothing below runs (the only OFF difference: one UTPPSetMotionShooterMode(false) at registration). ON (and effective):
-- the DLL (UTPPSetMotionShooterMode) strafes on the ground like motion set (camera-relative movement, motion set side / back speeds through the
-- reds MaxSpeed multiplier, sprint only forward, a standing V keeps her visible direction while the camera orbits and turns
-- with motion set clips when she moves / aims / shoots). This module keeps the native body turn and UTPP Locomotion OFF for as long as
-- the mode is effective, so V's capsule follows the mouse (utpp_camera follow mode; idle orbit, decouple hold and the free-look
-- key are inactive there). The align (ShooterModeAlignStep) turns the capsule onto the shown camera only where it can differ from
-- it: when the mode is switched on and when an aim ends. Effective = setting on AND the DLL accepted the native AND Animation
-- motion is enabled AND the DLL reports Shooter mode live (UTPPMotionShooterModeLive); polled once per second, so a late motion start or a motion
-- fallback makes the mode inert again (Locomotion / body turn come back, one log line each).
-- SM.nativeState: "-" (never called) | "ok" | "missing" (older DLL) | "failed".
local SM = { on = false, effective = false, nativeOk = nil, nativeState = "-", missingLogged = false, motionOk = true, motionAt = 0,
             align = false, alignSum = 0, alignStopped = false, prevAim = false, assertAt = 0, assertLogged = false,
             calls = 0, fails = 0, prevCur = nil, sentPrev = false, stall = 0 }
M.shooterMode = SM
function M.ShooterModeActive() return SM.effective == true end

-- UTPPSetMotionHipLease(on) (DLL H2 safeguard): the CET hip-fire lease (HipFireStep) is the reason the native body turn is off, so
-- the DLL keeps H2 closed. Sent only when the lease state CHANGES (start / end; the refresh of a running lease sends nothing),
-- independent of Shooter mode; E.hipLeaseSent = the value the DLL has (nil = unknown -> re-synced by the next ExecStep). A missing
-- native (older DLL) is silent and counts as sent (no retry every frame); a failing native is retried next frame, silently.
E.hipLeaseSent = nil
local hipLeaseCalls = 0
local function setHipLease(on)
  on = on == true
  if E.hipLeaseSent == on then return end
  if Game == nil or type(Game.UTPPSetMotionHipLease) ~= "function" then
    E.hipLeaseSent = on
    return
  end
  local ok, accepted = try(Game.UTPPSetMotionHipLease, on)
  if ok and accepted ~= false then
    hipLeaseCalls = hipLeaseCalls + 1
    E.hipLeaseSent = on
  end
end
function M.HipLeaseCalls() return hipLeaseCalls end

-- UTPPSetMotionShooterMode(on): idempotent, never resets matcher state. false when the native is missing / failed.
local function shooterModeNative(on)
  if Game == nil or type(Game.UTPPSetMotionShooterMode) ~= "function" then
    SM.nativeOk, SM.nativeState = false, "missing"
    if on == true and not SM.missingLogged then
      SM.missingLogged = true
      M.log("[utpp_bodyturn] Shooter mode native missing (older UTPP.dll) - setting inert")
    end
    return false
  end
  SM.calls = SM.calls + 1
  local ok, accepted = try(Game.UTPPSetMotionShooterMode, on == true)
  if not ok or accepted == false then
    SM.fails = SM.fails + 1
    SM.nativeOk, SM.nativeState = false, "failed"
    return false
  end
  SM.nativeOk, SM.nativeState = true, "ok"
  return true
end

-- Motion set enabled (CET setting) and live in the DLL (UTPPMotionShooterModeLive). Only an explicit false blocks: a missing
-- api entry / native (older build) or an unreadable answer counts as "unknown = fine".
local function motionok(api)
  api = api or {}
  if type(api.motionEnabled) == "function" then
    local ok, v = try(api.motionEnabled)
    if ok and v == false then return false end
  end
  if type(api.shooterModeLive) == "function" then
    local ok, v = try(api.shooterModeLive)
    if ok and v == false then return false end
  end
  return true
end

local function shooterModeEffective()
  return SM.on == true and SM.nativeOk == true and SM.motionOk ~= false
end

-- Releases the module's own camera holds at once (no linger).
local function releaseOwnHolds(api)
  if E.holdOwned then try(api.camHold, false); E.holdOwned = false end
  E.holdTimer = 0
  if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
end

-- Re-evaluates the effective flag (setting + native + motion) and, in executor mode, switches Locomotion / native body turn.
-- Order ON: native body turn off, Locomotion off (ALWAYS: the DLL flag outlives a CET reload), own holds released, align armed.
-- Order OFF: align off, own hold released, Locomotion back on, block mask re-sent (the native body turn returns in ExecStep).
function M.ApplyShooterModeLive(api)
  api = api or M.api or {}
  local eff = shooterModeEffective()
  if eff == SM.effective then return end
  SM.effective = eff
  if E.mode ~= "executor" then return end -- SelectExecutor applies the state
  if eff then
    E.trips, E.pausedUntil, E.score = 0, nil, 0 -- a stale camera-guard pause must not block the ADS align (ExecStep skips the guard)
    setNative(api, false)
    try(api.locomotionEnable, false)
    E.locomotionOwned = false
    releaseOwnHolds(api)
    E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
    SM.align, SM.alignSum, SM.alignStopped, SM.prevCur, SM.stall = true, 0, false, nil, 0
    SM.assertAt = E.clock + 1 -- Locomotion was just switched off here: the re-assert (which reads a state from before) starts one second later
    M.log("[utpp_bodyturn] Shooter mode ON: native body turn off, UTPP Locomotion off, camera follows V; V is turned onto the camera "
      .. "once (<= " .. E.aimSmoothRate .. " deg/s)")
  else
    SM.align, SM.prevCur, SM.stall = false, nil, 0
    releaseOwnHolds(api)
    if E.enableLocomotion then
      local okE = try(api.locomotionEnable, true)
      if okE then E.locomotionOwned = true end
    end
    W.dirty = true
    M.ApplyBlockMask(api, true)
    M.log(string.format("[utpp_bodyturn] Shooter mode OFF%s: UTPP Locomotion and the native body turn are back",
      SM.on and " (inert: Motion set not enabled / not live, or the native is not accepted)" or ""))
  end
end

-- Setting bodyturn.shooterMode / console. ON: DLL flag first, then CET; OFF: CET first, DLL flag last.
function M.SetShooterMode(on)
  SM.on = on == true
  if SM.on then
    shooterModeNative(true)
    SM.motionOk = motionok(M.api)
    SM.motionAt = E.clock + 1
    M.ApplyShooterModeLive(nil)
  else
    M.ApplyShooterModeLive(nil)
    shooterModeNative(false)
  end
  return SM.on
end

-- One CET frame while Shooter mode is effective (ExecStep, after the ADS aim align). The capsule normally follows the mouse
-- (follow camera), so there is nothing to do; when it can differ from the shown camera (mode switched on, an aim ended) V is
-- turned onto the camera heading once, with utpp_camera's free look held so the camera does not turn with her (alignHold
-- only locks the heading while aiming): <= aimSmoothRate deg/s with ease-out, as the smoothed aim align. Reads the shown
-- camera only (api.viewYaw, no look fallback) and only in third person (Locomotion reasons without kNotTpp 65536 / kStale
-- 131072), otherwise "shooterMode align waiting" without holds or requests. The progress counts the MEASURED body turn (a paused
-- game applies no request: the sum must not run away); no measured body motion for 3+ sent frames is also "waiting".
-- Returns the reason (also exec.why): shooterMode | shooterMode align | shooterMode align waiting | shooterMode aligned.
function M.ShooterModeAlignStep(api, dt)
  api = api or M.api or {}
  dt = (finite(dt) and dt > 0 and dt < 1) and dt or (1 / 60)
  E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
  if not SM.align then
    SM.prevCur, SM.sentPrev, SM.stall = nil, false, 0
    if E.holdOwned then E.holdTimer = 0; releaseHold(api, 0) end
    E.why = "shooterMode"
    return E.why
  end
  local okR, reasons = try(api.locomotionReasons)
  local tpp = okR and finite(reasons) and reasons >= 0 and reasons % 1 == 0
    and math.floor(reasons / 65536) % 2 == 0 and math.floor(reasons / 131072) % 2 == 0
  local okA, armed, okB, cur, okC, cam = false, nil, false, nil, false, nil
  if tpp then
    okA, armed = try(api.arm, true)
    okB, cur = try(api.bodyYaw)
    okC, cam = try(api.viewYaw)
  end
  if not (tpp and okA and armed == true and okB and finite(cur) and okC and finite(cam) and math.abs(cam) <= 360) then
    SM.prevCur, SM.sentPrev, SM.stall = nil, false, 0
    releaseHold(api, dt) -- a short gap keeps the hold (the linger), a long one (vehicle, FPP, menu) lets go
    E.why = "shooterMode align waiting"
    return E.why
  end
  -- measured progress of the body since the last frame
  local moved = SM.prevCur ~= nil and math.abs(norm(cur - SM.prevCur)) or 0
  if SM.prevCur ~= nil and SM.sentPrev then
    if moved < 0.05 then SM.stall = SM.stall + 1 else SM.stall = 0 end
  end
  SM.prevCur = cur
  local err = norm(cam - cur)
  if math.abs(err) < E.aimAlignDeadband then
    SM.align, SM.prevCur, SM.sentPrev, SM.stall = false, nil, false, 0
    E.holdTimer = 0
    releaseHold(api, 0) -- no linger: follow mode starts with the yaw offset ~ 0
    E.why = "shooterMode aligned"
    return E.why
  end
  if SM.stall < 3 then SM.alignSum = SM.alignSum + moved end
  if SM.alignSum > 720 then
    if not SM.alignStopped then
      SM.alignStopped = true
      M.log(string.format("[utpp_bodyturn] Shooter align STOPPED: %.0f deg turned without reaching the camera (camera turns with V?)",
        SM.alignSum))
    end
    SM.align, SM.prevCur, SM.sentPrev, SM.stall = false, nil, false, 0
    E.holdTimer = 0
    releaseHold(api, 0)
    E.why = "shooterMode aligned"
    return E.why
  end
  engageHold(api) -- FreeLook keeps the camera's world yaw (refuses to own a hold the player already holds; the align still works)
  local turnDt = math.min(dt, 0.1)
  local lim = E.aimSmoothRate * turnDt
  local step = math.max(-lim, math.min(lim, err * (1 - math.exp(-E.aimSmoothGain * turnDt))))
  local okP, res = try(api.applyAim or api.apply, norm(cur + step))
  local sent = okP and res == true
  SM.sentPrev = sent
  if sent then
    xs.adjSeq, xs.lastAdjStep, xs.lastStep = (xs.adjSeq or 0) + 1, step, step
    try(api.bodyStep, step) -- the camera rewrites its local transform with this step (ran earlier in the CET frame)
    try(api.lookStep, step)
  else
    xs.rejected = xs.rejected + 1
  end
  E.why = (sent and SM.stall < 3) and "shooterMode align" or "shooterMode align waiting"
  return E.why
end

-- Melee strike turn. While a fists strike runs (api.meleeStrikeYaw returns the world yaw of the game's hit direction, 1000 = none:
-- no tracked target, other weapon, first person) V is turned toward it so the visible fist and the hit agree. The goal is that
-- yaw limited to E.strikeMax degrees around V's facing at the start of the episode (an episode ends E.strikeGap s after the last
-- signal); a turn starts beyond E.strikeStartDeg and ends within E.strikeDeadband (hysteresis); no turn in a series that starts
-- while V moves. The same request path as the aim align (applyAim, FPP heading lock, game look step); the goal is computed from
-- V's measured yaw every frame, so late or dropped requests only slow the turn. The heading lock has one owner at a time: the
-- strike turn takes over a lock the aim align holds (E.alignHeld -> E.strikeHeld) and releases its own lock when it stops
-- turning, when the signal ends and on every frame another path owns (strikeIdle). Returns a why string when it owns this frame,
-- nil when not applicable.
-- The speed gate uses V's speed on the last frame without a strike signal (the game's melee lunge moves V fast while the strike
-- runs; that must not cancel the turn); with no such sample, the speed at the episode start.
-- Frames on which the strike step does not run (Locomotion active, camera guard pause, Shooter mode): V's speed is sampled for
-- the gate and the strike turn's heading lock is released. reset = also forget the episode (executor selection / disarm).
local function strikeIdle(api, reset)
  if reset then
    E.strikeStart, E.strikeSum, E.strikeSeen, E.strikeLogged, E.strikeTurning = nil, 0, -1e9, false, false
    E.strikeIdleSpeed = nil
  else
    local okR, reasons = try(api.locomotionReasons)
    if not (okR and finite(reasons) and math.floor(reasons / 2048) % 2 == 1) then
      local okS, sp = try(api.speed)
      E.strikeIdleSpeed = (okS and finite(sp)) and sp or nil
    end
  end
  if E.strikeHeld then
    if not E.alignHeld then try(api.alignHold, false) end
    E.strikeHeld = false
  end
end

function M.StrikeTurnStep(api, dt)
  if not E.strikeTurn then return nil end
  local okR, reasons = try(api.locomotionReasons)
  reasons = (okR and finite(reasons)) and reasons or 0
  local function release()
    if E.strikeHeld then
      if not E.alignHeld then try(api.alignHold, false) end
      E.strikeHeld = false
    end
  end
  -- melee attack state (bit 2048) in third person (bit 65536 clear); anything else: no signal, the episode ends after the gap
  local signal = nil
  if math.floor(reasons / 2048) % 2 == 1 and math.floor(reasons / 65536) % 2 == 0 and type(api.meleeStrikeYaw) == "function" then
    local okY, y = try(api.meleeStrikeYaw)
    if okY and finite(y) and math.abs(y) <= 360 then signal = y end
  end
  if signal ~= nil and (E.strikeStart == nil or E.clock - E.strikeSeen > E.strikeGap) then
    -- a new episode (also after a long stretch on which this step did not run): fresh start facing; no episode while V moves
    E.strikeStart, E.strikeSum, E.strikeLogged, E.strikeTurning = nil, 0, false, false
    local speed = E.strikeIdleSpeed
    if speed == nil then
      local okS, sp = try(api.speed)
      speed = (okS and finite(sp)) and sp or nil
    end
    if speed ~= nil and speed > E.strikeMaxSpeed then signal = nil end
  end
  if signal == nil then
    if E.clock - E.strikeSeen > E.strikeGap then
      E.strikeStart, E.strikeSum, E.strikeLogged, E.strikeTurning = nil, 0, false, false
    end
    if math.floor(reasons / 2048) % 2 == 0 then
      local okS, sp = try(api.speed)
      E.strikeIdleSpeed = (okS and finite(sp)) and sp or nil
    end
    release()
    return nil
  end
  E.strikeSeen = E.clock
  local okB, cur = try(api.bodyYaw)
  local okA, armed = try(api.arm, true)
  if not (okB and finite(cur) and okA and armed == true) then release(); return nil end
  if E.strikeStart == nil then E.strikeStart = cur end
  E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
  local off = norm(signal - E.strikeStart)
  local goal = E.strikeStart + math.max(-E.strikeMax, math.min(E.strikeMax, off))
  local err = norm(goal - cur)
  local atLimit = math.abs(off) > E.strikeMax
  E.strikeSum = math.abs(norm(cur - E.strikeStart))
  if E.strikeTurning then
    if math.abs(err) <= E.strikeDeadband then E.strikeTurning = false end
  elseif math.abs(err) > E.strikeStartDeg then
    E.strikeTurning = true
  end
  if not E.strikeTurning then
    release()
    E.why = atLimit and "strike hold" or "strike aligned"
    return E.why
  end
  local turnDt = math.min(math.max(dt, 1 / 240), 0.1)
  local lim = E.strikeRate * turnDt
  local step = math.max(-lim, math.min(lim, err * (1 - math.exp(-E.strikeGain * turnDt))))
  if not E.strikeHeld then
    -- one owner of the heading lock at a time: a lock the aim align holds is taken over (and released by the strike turn)
    if E.alignHeld then E.alignHeld = false else try(api.alignHold, true) end
    E.strikeHeld = true
  end
  local okP, res = try(api.applyAim or api.apply, norm(cur + step))
  if okP and res == true then
    xs.strikeTurns = (xs.strikeTurns or 0) + 1
    xs.adjSeq, xs.lastAdjStep = (xs.adjSeq or 0) + 1, step
    xs.lastAimStep = step
    try(api.lookStep, step)
    if not E.strikeLogged then
      E.strikeLogged = true
      M.log(string.format("[utpp_bodyturn] melee strike turn: hit direction %.0f deg from V's facing - turning V toward the tracked target (<= %d deg per series)",
        norm(signal - cur), E.strikeMax))
    end
    E.why = "strike turn"
  else
    xs.rejected = xs.rejected + 1
    E.why = "strike turn rejected"
  end
  return E.why
end

function M.SetStrikeTurn(on)
  E.strikeTurn = on == true
  strikeIdle(M.api or {}, true)
  return E.strikeTurn
end

-- Changing the policy releases its camera lock and starts a fresh admission/align episode.
function M.SetAimSmooth(on)
  E.aimSmooth = on == true
  E.hipFireRemaining, E.hipFireAdsShot, E.hipFireAdsAge = 0, false, 0
  hipFireForget()
  E.aimStable, E.aimUpperPrevious, E.aimLastStart = 0, false, nil
  E.aimAligning, E.aimAlignLogged = false, false
  E.aimAlignSum, E.aimAlignStopped = 0, false
  if E.alignHeld then try((M.api or {}).alignHold, false); E.alignHeld = false end
  return E.aimSmooth
end

-- Registers the UTPP-tab setting (utpp_core public API). Called from onInit; offline tests pass a mock core.
function M.RegisterSettings(core)
  M.core = core
  core.RegisterSection("bodyturn", "Body turn")
  core.Register({ key = "bodyturn.weaponTurn", section = "bodyturn", type = "bool", default = true,
    label = "Body turn with a weapon in hand (Locomotion)",
    desc = "Lets the character turn toward the movement direction while holding a ranged/melee weapon; aiming still blocks. Keep it on: the weapon check of Locomotion can misfire and then block all body turn.",
    apply = function(v) M.SetWeaponTurn(v == true) end })
  core.Register({ key = "bodyturn.turnRate", section = "bodyturn", type = "bool", default = true,
    label = "Faster turn clips (match the run speed)",
    desc = "Plays the run turn / pivot clips at the rate that matches the capsule speed instead of the run loop rate (the turns looked like slow motion).",
    apply = function(v) TR.on = v == true; M.ApplyTurnRate() end })
  core.Register({ key = "bodyturn.turnRateMax", section = "bodyturn", type = "float", default = 1.5, min = 1.0, max = 2.0,
    step = 0.05, label = "Turn clip rate: maximum",
    desc = "Upper cap of the turn clip playback rate (1.0 = clip speed). Raise if the turns still look slow, lower if the feet slide.",
    apply = function(v) TR.max = tonumber(v) or 1.5; M.ApplyTurnRate() end })
  core.Register({ key = "bodyturn.aimStrafe", section = "bodyturn", type = "bool", default = true,
    label = "Strafe while aiming",
    desc = "While aiming with a weapon the legs step sideways / backwards / diagonally in the stick direction and the hips and feet keep facing the aim (no forward running legs or turn steps). Off = legs as before.",
    apply = function(v) SF.on = v == true; M.ApplyStrafe() end })
  core.Register({ key = "bodyturn.aimStrafeUnarmed", section = "bodyturn", type = "bool", default = true,
    label = "Strafe while aiming: also with empty hands",
    desc = "Also strafe on the right-mouse zoom with empty hands (no weapon). Off = strafe only while aiming with a weapon.",
    apply = function(v) SF.unarmed = v == true; M.ApplyStrafe() end })
  core.Register({ key = "bodyturn.aimChestAlign", section = "bodyturn", type = "bool", default = true,
    label = "Strafe while aiming: keep the chest facing the aim",
    desc = "While strafing, turns the chest back from the twist of the side / back walk animations so the aimed upper body stays on the aim (spine only; hips and legs unchanged). Off = the chest twists with the step animation.",
    apply = function(v) CA.on = v == true; M.ApplyChestAlign() end })
  core.Register({ key = "bodyturn.aimJog", section = "bodyturn", type = "bool", default = true,
    label = "Strafe while aiming: jog",
    desc = "While aiming with a weapon V jogs instead of walking (strafe run steps; forward at the jog speed below, sideways 70 %, backwards 60 % of it). Weapon slowdowns keep their effect. Off = the vanilla aim walk speed.",
    apply = function(v) SF.jog = v == true; M.ApplyStrafe() end })
  core.Register({ key = "bodyturn.aimJogSpeed", section = "bodyturn", type = "float", default = 3.3, min = 3.3, max = 3.85,
    step = 0.05, label = "Strafe while aiming: jog speed (m/s)",
    desc = "Forward jog speed while aiming (default 3.3, 3.3 to 3.85 = vanilla run; vanilla aim walk is 1.65). Sideways and backwards scale with it (0.70 / 0.60).",
    apply = function(v) SF.jogSpeed = tonumber(v) or 3.3; M.ApplyStrafe() end })
  core.Register({ key = "bodyturn.guardFix", section = "bodyturn", type = "bool", default = true,
    label = "Body turn guard: shown camera, never while moving",
    desc = "The guard that pauses the body turn when the camera turns with V reads the camera you see (not the game's look, "
      .. "which follows V when it is not written) and never fires or holds a pause while V moves (> 1 m/s). Off = the old guard "
      .. "(false 5 s pauses with a frozen body direction while running).",
    apply = function(v) E.guardFix = v == true end })
  core.Register({ key = "bodyturn.aimSmooth", section = "bodyturn", type = "bool", default = true,
    label = "Aim: smooth body alignment",
    desc = "Turns the body at up to 540 deg/s with ease-out. Starts beyond 35 deg, at least 0.3 s between episodes; "
      .. "requires a weapon in hand or UpperBody Aim stable for 0.1 s. Off = legacy 1080 deg/s alignment.",
    apply = function(v) M.SetAimSmooth(v); M.log("[utpp_bodyturn] aim smooth -> " .. tostring(E.aimSmooth)) end })
  core.Register({ key = "bodyturn.meleeStrikeTurn", section = "bodyturn", type = "bool", default = false,
    label = "Fists: turn toward the target when striking",
    desc = "In third person the game's fist hit goes toward the target it tracks, while V's body keeps its own facing (the "
      .. "upper-body tilt covers only 25 deg). With this on, V turns toward that target during a fists strike, eased, at most 50 deg "
      .. "per series of strikes. Only with a tracked target; other weapons and first person are unchanged.",
    apply = function(v) M.SetStrikeTurn(v == true) end })
  -- registered LAST (the apply of the stored value runs at once and sends UTPPSetMotionShooterMode(v), even false: the DLL state is
  -- unknown after a CET reload and the native is idempotent)
  core.Register({ key = "bodyturn.shooterMode", section = "bodyturn", type = "bool", default = false, label = "Shooter mode",
    desc = "Third-person-shooter style movement relative to the camera. Standing still: V does not turn when you turn the camera "
      .. "(you orbit around her). When V moves, aims or shoots she turns with the start / turn / pivot animations (one smooth "
      .. "turn, no spin) until she faces where the camera looks; she then runs forward, side-steps (70 % of the run speed) or "
      .. "backpedals (60 %) with the strafe animations. Sprint works only roughly forward. "
      .. "Off = V turns toward the movement direction (body turn).",
    apply = function(v) M.SetShooterMode(v == true) end })
end

-- Called once at startup (and again by SetExecutor(true)). Returns the log line.
function M.SelectExecutor(api)
  api = api or M.api or {}
  E.selected = true
  if not E.enabled then
    setNative(api, false, true)
    E.hipLeaseSent = nil
    setHipLease(false)
    E.mode, E.why = "disabled", "switch off"
    return "UTPP body turn disabled (UTPPSetMMBodyTurn(false); executor switched off by SetExecutor(false))"
  end
  -- probe only: in the main menu there is no game session (the scriptable system answers later); ExecStep re-arms every
  -- frame and keeps the native body turn off for as long as the executor does not answer (= disabled as before)
  local ok, armed = try(api.arm, true)
  local answers = ok and armed == true
  E.mode, E.why = "executor", "selected"
  E.trips, E.pausedUntil, E.score = 0, nil, 0
  E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
  E.viewSrc = false
  strikeIdle(api, true)
  if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
  E.hipFireRemaining, E.hipFireAdsShot, E.hipFireAdsAge = 0, false, 0
  hipFireForget()
  E.hipLeaseSent = nil
  setHipLease(false) -- no lease at selection (re-sync of the DLL flag)
  E.aimStable, E.aimUpperPrevious, E.aimLastStart = 0, false, nil
  E.aimAligning, E.aimAlignLogged = false, false
  E.aimAlignSum, E.aimAlignStopped = 0, false
  setNative(api, false, true) -- switched on while UTPP Locomotion is active (ExecStep, on change only)
  local freeMove = "Locomotion setting untouched"
  if SM.on then -- Shooter mode: re-read the live state with this api (a late motion start / a fallback keeps the mode inert)
    SM.motionOk = motionok(api)
    SM.motionAt = E.clock + 1
  end
  SM.effective = shooterModeEffective()
  if SM.effective then
    try(api.locomotionEnable, false) -- ALWAYS: the DLL flag outlives a CET reload
    E.locomotionOwned = false
    freeMove = "UTPP Locomotion off (Shooter mode)"
    SM.align, SM.alignSum, SM.alignStopped, SM.prevCur, SM.stall = true, 0, false, nil, 0
  elseif E.enableLocomotion then
    local okE = try(api.locomotionEnable, true)
    if okE then E.locomotionOwned = true; freeMove = "UTPP Locomotion enabled" end
  end
  W.dirty = true
  M.ApplyBlockMask(api, true) -- Locomotion (re)enabled: send the block mask (deferred without a player: ExecStep)
  local okR, reasons = try(api.locomotionReasons)
  return string.format("body turn executor: adjustTransform via utpp_bodyturn.reds (answers now: %s) "
    .. "+ camera-heading movement while UTPP Locomotion is active (%s, reasons=%s), camera hold=%s, maxRate=%d; kill switch "
    .. "GetMod(\"utpp_bodyturn\").SetExecutor(false)", answers and "yes" or "not yet", freeMove,
    (okR and finite(reasons)) and string.format("0x%X", reasons) or "?", tostring(E.cameraHold), E.maxRate)
end

function M.DisarmExecutor(api, why)
  api = api or M.api or {}
  SM.align, SM.prevAim, SM.prevCur, SM.stall = false, false, nil, 0
  strikeIdle(api, true)
  if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
  E.hipFireRemaining, E.hipFireAdsShot, E.hipFireAdsAge = 0, false, 0
  hipFireForget()
  E.hipLeaseSent = nil
  setHipLease(false)
  E.aimStable, E.aimUpperPrevious, E.aimLastStart = 0, false, nil
  E.aimAligning, E.aimAlignLogged = false, false
  E.aimAlignSum, E.aimAlignStopped = 0, false
  try(api.arm, false)
  setNative(api, false, true)
  if E.holdOwned then try(api.camHold, false); E.holdOwned = false end
  if E.locomotionOwned then try(api.locomotionEnable, false); E.locomotionOwned = false end
  if W.sent ~= nil and W.sent ~= M.MASK_DEFAULT then try(api.setBlockMask, M.MASK_DEFAULT) end -- back to the stock mask
  W.sent, W.logged, W.dirty = nil, nil, true
  E.mode, E.why = "disabled", why or "disarmed"
  E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
end

-- One CET frame. Returns a short reason string (also kept in exec.why).
function M.ExecStep(api, dt)
  if E.mode ~= "executor" then return "off" end
  api = api or M.api or {}
  dt = (finite(dt) and dt > 0 and dt < 1) and dt or (1 / 60)
  E.clock = E.clock + dt
  xs.frames = xs.frames + 1
  if E.pausedUntil ~= nil and E.clock >= E.pausedUntil then E.pausedUntil, E.score = nil, 0 end
  -- vehicle hold edge (utpp_camera VehicleHeld; nil = unknown / no camera module: nothing changes)
  do
    local okVh, vh = try(api.vehicleHeld)
    if okVh and vh == true then
      E.vehicleHeldPrev, E.vehicleSettleUntil = true, nil
    elseif okVh and vh == false and E.vehicleHeldPrev then
      E.vehicleHeldPrev = false
      E.vehicleSettleUntil = E.clock + (E.vehicleSettle or 0.3)
    end
  end
  -- guardFix: V moving faster than guardMoveSpeed -> the guard is off and a pause does not hold (no frozen yaw)
  local speed = 0
  if E.guardFix then
    local okSp, sp = try(api.speed)
    if okSp and finite(sp) and sp > 0 then speed = sp end
  end
  -- Resolve the guard source before pause admission. Reading last frame's
  -- viewSrc here made pause clear it, resume restore it, and reset native yaw
  -- history every few frames. A world/free camera has independent heading;
  -- correlated mouse and body motion while running is not feedback evidence.
  local gcam, gsrc = nil, "look"
  local independentView = false
  if E.guardFix then
    local okV, vy = try(api.viewYaw)
    if okV and finite(vy) and math.abs(vy) <= 360 then
      gcam, gsrc = vy, "view"
      local okI, independent = try(api.viewIndependent)
      independentView = okI and independent == true
    else
      local okL, lw = try(api.lookWritten)
      if okL and lw == false then gsrc = "look-unwritten" end
    end
    if E.guardFrameSource ~= gsrc then
      E.prevBody, E.prevCam, E.prevStep, E.score = nil, nil, 0, 0
      E.guardFrameSource = gsrc
    end
    E.viewSrc = gsrc == "view"
  end
  local moving = E.guardFix and speed > (E.guardMoveSpeed or 1.0)
    and (gsrc ~= "view" or independentView)
  if moving and E.pausedUntil ~= nil and not E.pauseMoving then
    E.pauseMoving = true
    xs.pauseMoves = (xs.pauseMoves or 0) + 1
    M.log(string.format("[utpp_bodyturn] camera guard pause suspended: V moves %.1f m/s (> %.1f) - the executor turns V with the "
      .. "movement, the pause holds again when V stands", speed, E.guardMoveSpeed or 1.0))
  elseif not moving and E.pauseMoving then
    E.pauseMoving = false
  end
  local paused = E.pausedUntil ~= nil and not moving
  local okM, freeMove = try(api.locomotionActive)
  local active360 = okM and freeMove == true
  if active360 and W.prevActive == false then W.dirty = true end -- Locomotion active again -> re-send the mask
  W.prevActive = active360
  -- Shooter mode: once per second re-read motion enabled / live, retry a failed native, re-evaluate the effective state
  if SM.on and E.clock >= SM.motionAt then
    SM.motionAt = E.clock + 1
    if SM.nativeOk ~= true then
      local was = SM.nativeState
      if shooterModeNative(true) and was == "failed" then
        M.log("[utpp_bodyturn] Shooter mode native accepted after a retry")
      end
    end
    SM.motionOk = motionok(api)
    M.ApplyShooterModeLive(api)
  end
  -- Locomotion must stay off while Shooter mode is effective (another module / a CET reload may switch it on): re-assert <= 1/s
  if SM.effective and active360 and E.clock >= SM.assertAt then
    SM.assertAt = E.clock + 1
    try(api.locomotionEnable, false)
    E.locomotionOwned = false
    if not SM.assertLogged then
      SM.assertLogged = true
      M.log("[utpp_bodyturn] Shooter mode: UTPP Locomotion reported active - switched off again (re-checked once per second)")
    end
  end
  if W.dirty and E.clock >= W.retryAt then M.ApplyBlockMask(api) end
  local okSignals, aimSignals = try(api.aimSignals)
  local okReasons, headingReasons = try(api.locomotionReasons)
  local hipFire = M.HipFireStep(E.aimAlign and okSignals and aimSignals or nil,
                               okReasons and headingReasons or nil, dt)
  setHipLease(hipFire) -- DLL H2 exclusion: only on a change (start / end)
  local want = active360 and not paused and not hipFire and not SM.effective
  local missing = false
  if want then
    -- idempotent; a new game session starts with a disarmed executor
    local okR, armed = try(api.arm, true)
    if not okR or armed ~= true then
      want, missing = false, true
      xs.armFails = xs.armFails + 1
    end
  end
  setNative(api, want)
  local function idle(why)
    E.why = why
    E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
    releaseHold(api, dt)
    return why
  end
  if not want and not paused and E.strikeTurn and not SM.effective then
    local strikeWhy = M.StrikeTurnStep(api, dt)
    if strikeWhy then return strikeWhy end
  elseif E.strikeTurn then
    strikeIdle(api) -- another owner this frame: the strike turn keeps no heading lock
  end
  if not want and not paused and E.aimAlign then
    -- aiming while standing after V had turned toward the movement left V facing away from
    -- the game's look (body -16 deg, look 166 deg): the game does not recentre that offset when the aim starts, the aim
    -- offset then reported a 180 deg error and V faced the camera. While Locomotion is blocked only by aiming (reason bit
    -- 4096, third person) the executor turns V onto the game's look yaw (utpp_camera AlignHold = FPP heading lock while
    -- it turns, <= aimAlignRate deg/s; without the lock the game's camera turned with V: 541 deg chase).
    local okR, reasons = try(api.locomotionReasons)
    reasons = (okR and finite(reasons)) and reasons or 0
    local aiming = hipFire or (math.floor(reasons / 4096) % 2 == 1 and math.floor(reasons / 65536) % 2 == 0)
    if SM.effective then -- an aim that ended arms the face-the-camera align (the capsule may differ from the camera)
      if SM.prevAim and not aiming then SM.align, SM.alignSum, SM.alignStopped, SM.prevCur, SM.stall = true, 0, false, nil, 0 end
      SM.prevAim = aiming
    end
    if aiming and E.vehicleSettleUntil ~= nil then
      if E.clock < E.vehicleSettleUntil then
        -- right after a vehicle hold ended: no body turn yet (the camera release / aim pose are still settling)
        E.aimAligning, E.aimAlignLogged = false, false
        E.aimAlignSum, E.aimAlignStopped = 0, false
        if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
        E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
        E.why = "aim align vehicle settle"
        return E.why
      end
      E.vehicleSettleUntil = nil
    end
    if aiming then
      if E.aimSmooth then
        local okSig, sig = okSignals, aimSignals
        local upper = okSig and type(sig) == "table" and sig.upper == 6
        E.aimStable = upper and (E.aimUpperPrevious and (E.aimStable + dt) or 0) or 0
        E.aimUpperPrevious = upper
        local weapon = okSig and type(sig) == "table" and sig.weaponDrawn == true
        if not weapon and E.aimStable + 1e-9 < E.aimStableTime then
          E.aimAligning, E.aimAlignLogged = false, false
          E.aimAlignSum, E.aimAlignStopped = 0, false
          if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
          E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
          E.why = "aim align waiting"
          return E.why
        end
      end
      local okArm, armed = try(api.arm, true)
      local okB, cur = try(api.bodyYaw)
      -- target = the yaw of the camera shown by utpp_camera (world camera, keeps its own yaw while aiming);
      -- the game's look (api.cameraYaw) only without utpp_camera
      local okC, cam = try(api.viewYaw)
      if not okC or not finite(cam) then okC, cam = try(api.cameraYaw) end
      if okArm and armed == true and okB and finite(cur) and okC and finite(cam) and math.abs(cam) <= 360 then
        local err = norm(cam - cur)
        -- (fast turns while aiming left V facing elsewhere): the guard counted every degree turned in
        -- the whole aim, so long aims with fast mouse turns hit 540 deg and stopped the align for good. Now it counts only
        -- the degrees turned since V was last within aimAlignNear deg of the camera (a real chase never gets close).
        if math.abs(err) <= E.aimAlignNear then E.aimAlignSum = 0 end
        -- guard: the look turning with V would make V spin -> stop for the rest of this aim (logged once).
        -- Not while utpp_camera reports the game look sync (the look keeps its world heading; 542 deg were turned
        -- legitimately by fast mouse turns and the stop left V aiming away)
        local okS, synced = try(api.lookSynced)
        synced = okS and synced == true
        if not synced and (E.aimAlignSum or 0) > E.aimAlignMaxSum then
          if not E.aimAlignStopped then
            E.aimAlignStopped = true
            M.log(string.format("[utpp_bodyturn] aim align STOPPED: %.0f deg turned without reaching the look (camera turns with V?)",
              E.aimAlignSum))
          end
          if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
          E.why = "aim align stopped"
          return E.why
        end
        local start = hipFire and E.aimAlignStart or (E.aimSmooth and E.aimSmoothStart or (E.aimAlignStart or E.aimAlignDeadband))
        local cooling = not hipFire and E.aimSmooth and E.aimLastStart ~= nil and E.clock - E.aimLastStart < E.aimCooldown
        if E.aimAligning ~= true and math.abs(err) >= start and not cooling then
          E.aimAligning = true
          if E.aimSmooth then E.aimLastStart = E.clock end
        end
        if E.aimAligning == true and math.abs(err) >= E.aimAlignDeadband then
          if not E.alignHeld then try(api.alignHold, true); E.alignHeld = true end
          local rate = hipFire and E.aimAlignRate or (E.aimSmooth and E.aimSmoothRate or E.aimAlignRate)
          local turnDt = E.aimSmooth and math.min(dt, 0.1) or math.min(math.max(dt, 1 / 240), 0.1)
          local lim = rate * turnDt
          local desired = E.aimSmooth and not hipFire and err * (1 - math.exp(-E.aimSmoothGain * turnDt)) or err
          local step = math.max(-lim, math.min(lim, desired))
          -- (body yaw never moved - the plain request is dropped while Locomotion is inactive)
          local okP, res = try(api.applyAim or api.apply, norm(cur + step))
          local sent = okP and res == true
          if sent then xs.aimAligned = (xs.aimAligned or 0) + 1; E.aimAlignSum = (E.aimAlignSum or 0) + math.abs(step)
            xs.adjSeq, xs.lastAdjStep = (xs.adjSeq or 0) + 1, step -- an adjustTransform turn of V was requested
          else xs.rejected = xs.rejected + 1 end
          -- the game look write gets V's exact turn of the coming tick (utpp_camera OnBodyLookStep -> native ahead)
          if sent then
            xs.lastAimStep = step
            try(api.lookStep, step)
          end
          if math.abs(err) >= 45 and not E.aimAlignLogged then
            E.aimAlignLogged = true
            M.log(string.format("[utpp_bodyturn] aim align: V faces %.0f deg away from the look - turning V onto the aim (<= %d deg/s)",
              err, rate))
          end
          E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
          E.why = sent and "aim align" or "aim align rejected"
          return E.why
        end
        E.aimAlignLogged = false
        if math.abs(err) < E.aimAlignDeadband then E.aimAligning = false end
        E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
        if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
        E.why = cooling and math.abs(err) >= start and "aim align cooldown" or "aim aligned"
        return E.why
      end
    end
  end
  E.aimAlignSum, E.aimAlignStopped = 0, false -- not aiming (or Locomotion active): the next aim starts a fresh align
  E.aimAligning, E.aimAlignLogged = false, false -- interrupted aim must re-enter through the start hysteresis
  E.aimStable, E.aimUpperPrevious = 0, false
  if E.alignHeld then try(api.alignHold, false); E.alignHeld = false end
  if SM.effective then return M.ShooterModeAlignStep(api, dt) end
  if not want then return idle(paused and "paused" or (missing and "executor missing" or "locomotion inactive")) end
  local okA, active = try(api.active)
  if not okA or active ~= true then return idle("native inactive") end
  local okB, cur = try(api.bodyYaw)
  if not okB or not finite(cur) then return idle("no body yaw") end
  local okC, cam = try(api.cameraYaw)
  if not okC or not finite(cam) or math.abs(cam) > 360 then cam = nil end
  -- guardFix: guard input = the camera shown by utpp_camera (world camera / utppCamera: keeps its own yaw) when
  -- available, else the game's look - but not while that look is not written from the camera (it follows V then)
  if not E.guardFix then
    gcam, gsrc = cam, "look"
  elseif gsrc == "look" then
    gcam = cam
  end
  if moving then gcam, gsrc = nil, "moving" end
  -- camera coupling guard: while the executor turns the body, a camera that turns by the same amount means the target
  -- (camera + stick) turns with the body -> V would spin
  if E.prevBody ~= nil and E.prevCam ~= nil and gcam ~= nil and math.abs(E.prevStep) >= 0.5 then
    local bd, cd = norm(cur - E.prevBody), norm(gcam - E.prevCam)
    if math.abs(bd) >= 0.5 then
      if math.abs(cd - bd) <= 0.25 * math.abs(bd) then E.score = E.score + math.abs(bd)
      else E.score = math.max(0, E.score - math.abs(bd)) end
    end
    if E.score >= E.coupledLimit then
      E.trips = E.trips + 1
      local forever = E.trips >= E.coupledMaxTrips
      E.pausedUntil = forever and math.huge or (E.clock + E.coupledPause)
      local score = E.score
      E.score = 0
      setNative(api, false)
      M.log(string.format("[utpp_bodyturn] camera turns with the body (%d deg) - body turn paused %s (trip %d)%s",
        E.coupledLimit, forever and "for this session (SetExecutor(true) re-arms)" or string.format("%.0f s", E.coupledPause),
        E.trips, E.guardFix and string.format(" [bd=%.2f cd=%.2f score=%.1f src=%s speed=%.1f]", bd, cd, score, gsrc, speed) or ""))
      return idle("paused")
    end
  elseif E.guardFix and gcam == nil then
    E.score = 0 -- guard skipped (moving / look not written): no stale score
  end
  if E.guardFix then xs.guardSrc = gsrc end
  engageHold(api)
  local okK, cmd = try(api.command, cur)
  if not okK or not finite(cmd) then
    xs.cmdErrors = xs.cmdErrors + 1
    E.prevBody, E.prevCam, E.prevStep = cur, cam, 0
    E.why = "no command"
    return E.why
  end
  local step = norm(cmd - cur)
  local lim = E.maxRate * math.min(math.max(dt, 1 / 240), 0.1)
  if math.abs(step) > lim then
    step = (step >= 0) and lim or -lim
    xs.clamped = xs.clamped + 1
  end
  local sent = false
  if math.abs(step) >= E.minStep then
    local okP, res = try(api.apply, norm(cur + step))
    sent = okP and res == true
    if sent then xs.applied = xs.applied + 1 else xs.rejected = xs.rejected + 1 end
    if sent then xs.adjSeq, xs.lastAdjStep = (xs.adjSeq or 0) + 1, step end -- adjustTransform turn requested
    -- the camera rewrites its local transform with this exact step (it ran earlier in this CET frame)
    if sent then try(api.bodyStep, step) end
    -- the game look write gets the same step as V's turn of the coming tick (native ahead)
    if sent then try(api.lookStep, step) end
    E.why = sent and "applied" or "rejected"
  else
    xs.deadband = xs.deadband + 1
    E.why = "deadband"
  end
  E.prevBody, E.prevCam, E.prevStep = cur, (E.guardFix and gcam ~= nil) and gcam or cam, sent and step or 0
  xs.lastCur, xs.lastCmd, xs.lastStep, xs.lastCam = cur, cmd, step, cam or 0
  return E.why
end

-- Kill switch for the executor. Returns the switch value.
function M.SetExecutor(on)
  on = on ~= false
  E.enabled = on
  if E.mode == "executor" and not on then
    M.DisarmExecutor(M.api, "switch off")
    M.log("[utpp_bodyturn] body turn executor switched off - UTPP body turn disabled (UTPPSetMMBodyTurn(false))")
  elseif E.mode == "disabled" and on and E.selected then
    M.log("[utpp_bodyturn] " .. M.SelectExecutor(M.api))
  elseif E.mode == "executor" and on and (E.pausedUntil ~= nil or E.trips > 0) then
    E.trips, E.pausedUntil, E.score = 0, nil, 0
    M.log("[utpp_bodyturn] body turn executor re-armed (camera guard reset)")
  end
  return E.enabled
end

function M.Status()
  return string.format("utpp_bodyturn %s | exec=%s enabled=%s why=%s native=%s "
                       .. "frames=%d applied=%d deadband=%d rejected=%d clamped=%d cmdErrors=%d armFails=%d toggles=%d hold=%s holds=%d "
                       .. "trips=%d paused=%s locomotionOwned=%s lastCur=%.1f lastCmd=%.1f lastStep=%.2f lastCam=%.1f guardFix=%s guardSrc=%s pauseMoves=%d | weaponTurn=%s mask=%s "
                       .. "maskCalls=%d maskFails=%d maskNoPlayer=%d | turnRate=%s max=%.2f sent=%s calls=%d fails=%d"
                       .. " | aimStrafe=%s unarmed=%s sent=%s calls=%d fails=%d aimJog=%s jogF=%.2f"
                       .. " | aimChestAlign=%s sent=%s calls=%d fails=%d",
                       M.VERSION,
                       E.mode, tostring(E.enabled), tostring(E.why), tostring(E.nativeOn), xs.frames, xs.applied, xs.deadband,
                       xs.rejected, xs.clamped, xs.cmdErrors, xs.armFails, xs.toggles, tostring(E.holdOwned), xs.holds, E.trips,
                       tostring(E.pausedUntil ~= nil), tostring(E.locomotionOwned), xs.lastCur, xs.lastCmd, xs.lastStep, xs.lastCam,
                       tostring(E.guardFix), tostring(xs.guardSrc or "-"), xs.pauseMoves or 0,
                       tostring(W.on), W.sent and string.format("0x%X", W.sent) or "-", W.calls, W.fails, W.noPlayer,
                       tostring(TR.on), TR.max, TR.sent and "yes" or (TR.missing and "missing" or "no"), TR.calls, TR.fails,
                       tostring(SF.on), tostring(SF.unarmed), SF.sent and "yes" or (SF.missing and "missing" or "no"), SF.calls,
                       SF.fails, tostring(SF.jog), SF.jogF, tostring(CA.on), CA.sent and "yes" or (CA.missing and "missing" or "no"), CA.calls, CA.fails)
    .. string.format(" | aimSmooth=%s aimStable=%.3f aimStart=%.0f aimRate=%.0f", tostring(E.aimSmooth), E.aimStable,
      E.aimSmooth and E.aimSmoothStart or E.aimAlignStart, E.aimSmooth and E.aimSmoothRate or E.aimAlignRate)
    .. string.format(" | shooterMode=%s eff=%s native=%s align=%s motion=%s", tostring(SM.on), tostring(SM.effective),
      tostring(SM.nativeState), tostring(SM.align), tostring(SM.motionOk))
    .. string.format(" | strikeTurn=%s turned=%.1f turns=%d", tostring(E.strikeTurn), E.strikeSum, xs.strikeTurns or 0)
end

-- ---------------------------------------------------------------- in-game wiring (skipped by the offline test)
-- fix: the events used to be registered only when type(GetMod) == "function" at FILE LOAD; in CET that was false, so
-- no events were registered (CET scripting.log: only "Mod utpp_bodyturn loaded!"). Events are now registered whenever
-- registerForEvent exists; GetMod is resolved lazily in the handlers.
if type(registerForEvent) == "function" then
  local tries = 0
  local done = false
  local announced = false
  -- no file system paths in the log: a Lua error text names the script file with its full path (user name included); a path in
  -- the CET mods folder is cut to "mods\...", the game folder prefix is dropped, a user profile path becomes "~" (same rules as
  -- utpp_core M.ScrubPaths)
  local function scrubPaths(s)
    s = tostring(s)
    if not string.find(s, "%a:[\\/]") then return s end
    s = string.gsub(s, "%a:[\\/][^\r\n\"']-([Mm]ods[\\/])", "%1")
    s = string.gsub(s, "%a:[\\/][^\r\n\"']-Cyberpunk 2077[\\/]", "")
    s = string.gsub(s, "%a:[\\/][Uu]sers[\\/][^\\/%s\"']+", "~")
    return (string.gsub(s, "%a:[\\/][^%s\"']*", "<path>"))
  end
  -- CET prints do not reach a file in game; status lines also go to the UTPP red4ext log.
  local function sink(msg)
    print(msg)
    if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(msg)) end) end
  end
  M.log = sink

  -- Locomotion block mask (weapon turn)
  local function gameMaskApi()
    return {
      hasPlayer = function()
        local ok, p = pcall(Game.GetPlayer)
        return ok and p ~= nil
      end,
      setBlockMask = function(mask) Game.UTPPLocomotionSetBlockMask(mask) end,
    }
  end
  M.maskApi = gameMaskApi()

  -- executor api. Script globals first, the UTPPBodyTurnExec scriptable system as fallback (CET builds that do not
  -- resolve script globals, as in utpp_body).
  local function gameExecApi()
    local function sys()
      local ok, s = pcall(function() return Game.GetScriptableSystemsContainer():Get("UTPPBodyTurnExec") end)
      return ok and s or nil
    end
    local function viaGlobalOrSystem(global, method, ...)
      local args = { ... }
      local ok, r = pcall(function() return Game[global](unpack(args)) end)
      if ok and r ~= nil then return r end
      local s = sys()
      if s == nil then return nil end
      local okS, rS = pcall(function() return s[method](s, unpack(args)) end)
      if okS then return rS end
      return nil
    end
    local camMod = nil
    local function cam()
      if camMod == nil and type(GetMod) == "function" then
        local ok, m = pcall(GetMod, "utpp_camera")
        camMod = (ok and type(m) == "table" and type(m.FreeLook) == "function") and m or false
      end
      return camMod or nil
    end
    local mask = gameMaskApi()
    return {
      hasPlayer = mask.hasPlayer,
      setBlockMask = mask.setBlockMask,
      arm = function(on) return viaGlobalOrSystem("UTPPBodyTurnExecArm", "Arm", on) end,
      apply = function(yaw) return viaGlobalOrSystem("UTPPBodyTurnExecRequest", "Request", yaw) end,
      -- aim align request, applied also while Locomotion is blocked by the aim (utpp_bodyturn.reds RequestAim)
      applyAim = function(yaw) return viaGlobalOrSystem("UTPPBodyTurnExecRequestAim", "RequestAim", yaw) end,
      cameraYaw = function()
        local v = Game.UTPPBodyTurnHeldCameraYaw()
        if v == nil then local s = sys(); v = s and s:HaveYaws() and s:HeldCameraYaw() or nil end
        return v
      end,
      setNative = function(on) Game.UTPPSetMMBodyTurn(on) end,
      locomotionEnable = function(on) Game.UTPPLocomotionSetEnabled(on) end,
      locomotionActive = function() return Game.UTPPLocomotionActive() end,
      locomotionReasons = function() return Game.UTPPLocomotionReasons() end,
      aimSignals = function()
        local p = Game.GetPlayer()
        if p == nil then return nil end
        -- Read live state only here, while aim alignment is requested; never retain game handles.
        local okW, item = pcall(function()
          return Game.GetTransactionSystem():GetItemInSlot(p, TweakDBID.new("AttachmentSlots.WeaponRight"))
        end)
        local okU, upper, action, detailed = pcall(function()
          local d = GetAllBlackboardDefs().PlayerStateMachine
          local bb = Game.GetBlackboardSystem():GetLocalInstanced(p:GetEntityID(), d)
          if not bb then return nil, nil, nil end
          local ub = bb:GetInt(d.UpperBody)
          -- Game 2.5 split PSM.Weapon into RightWeapon / LeftWeapon (dual wield); 2.31 fallback kept.
          local act = bb:GetInt(d.RightWeapon or d.Weapon)
          -- While dual wielding (UpperBody DualWield 7 / IsDualWielding) a left-hand Shoot or Reload counts too.
          if d.LeftWeapon ~= nil and (ub == 7 or (d.IsDualWielding ~= nil and bb:GetBool(d.IsDualWielding) == true)) then
            local l = bb:GetInt(d.LeftWeapon)
            if act ~= 8 and l == 8 then act = 8 elseif act ~= 8 and act ~= 2 and l == 2 then act = 2 end
          end
          -- LocomotionDetailed for the sprint demotion of the hipfire lease (own pcall: a missing field keeps ub / act)
          local det = nil
          pcall(function() det = bb:GetInt(d.LocomotionDetailed) end)
          return ub, act, det
        end)
        -- gamePSMDetailedLocomotionStates Sprint 4 / CrouchSprint 30 (adapter::PsmSprint); unreadable = nil (HipFireStep: the 0.75 s lease).
        -- air: SlideFall 6, Fall .. DodgeAir 14-22, Regular / Hard / VeryHard land 23-25, Climb / Vault 8-9, ladder 10-13 (adapter::PsmAir,
        -- PsmLand, PsmTraversal and the ladder range of the DLL hold's grounded rule)
        local sprint, air = nil, nil
        if okU and type(detailed) == "number" then
          sprint = detailed == 4 or detailed == 30
          air = detailed == 6 or (detailed >= 8 and detailed <= 25)
        end
        return {weaponDrawn = okW and item ~= nil, upper = okU and upper or nil, action = okU and action or nil, sprint = sprint, air = air}
      end,
      active = function() return Game.UTPPMMBodyTurnActive() end,
      command = function(cur) return Game.UTPPMMBodyYawCommand(cur, cur) end,
      -- melee strike turn: world yaw of the game's fist hit direction (1000 = none; utpp_melee.reds UTPPMeleeStrikeYaw)
      meleeStrikeYaw = function() return Game.UTPPMeleeStrikeYaw() end,
      bodyYaw = function()
        local p = Game.GetPlayer()
        if p == nil then return nil end
        local f = p:GetWorldForward()
        if f == nil or (f.x * f.x + f.y * f.y) < 1e-8 then return nil end
        return math.deg(math.atan2(-f.x, f.y))
      end,
      camHeld = function()
        local c = cam()
        if c == nil or type(c.rt) ~= "table" then return nil end
        return c.rt.freeHeld == true
      end,
      camHold = function(on)
        local c = cam()
        if c == nil then return false end
        c.FreeLook(on)
        return true
      end,
      bodyStep = function(step)
        local c = cam()
        if c == nil or type(c.OnBodyStep) ~= "function" then return false end
        return c.OnBodyStep(step)
      end,
      -- V's exact turn of the coming tick for the game look write (utpp_camera re-sends UTPPSetGameLook with it)
      lookStep = function(step)
        local c = cam()
        if c == nil or type(c.OnBodyLookStep) ~= "function" then return false end
        return c.OnBodyLookStep(step)
      end,
      viewYaw = function()
        local c = cam()
        if c == nil or type(c.ShownYaw) ~= "function" then return nil end
        return c.ShownYaw()
      end,
      -- utpp_camera holds the camera for a vehicle (nil = no camera module / no such API)
      vehicleHeld = function()
        local c = cam()
        if c == nil or type(c.VehicleHeld) ~= "function" then return nil end
        return c.VehicleHeld() == true
      end,
      viewIndependent = function()
        local c = cam()
        return c ~= nil and type(c.ShownYaw) == "function" and finite(c.ShownYaw())
          and c.G ~= nil and c.G.wroteWorld == true
          and c.S ~= nil and c.S.look ~= nil and c.S.look.free == true
      end,
      alignHold = function(on)
        local c = cam()
        if c == nil or type(c.AlignHold) ~= "function" then return false end
        c.AlignHold(on)
        return true
      end,
      lookSynced = function()
        local c = cam()
        if c == nil or type(c.LookSynced) ~= "function" then return false end
        return c.LookSynced() == true
      end,
      -- guardFix: the game look write state (utpp_camera.LookWritten; nil = unknown -> guard as before) and V's
      -- horizontal speed (m/s)
      lookWritten = function()
        local c = cam()
        if c == nil or type(c.LookWritten) ~= "function" then return nil end
        return c.LookWritten() -- true / false / nil (unknown)
      end,
      -- Shooter mode: Motion set enabled in CET (utpp_motion setting; a missing module counts as off) and live in the DLL
      motionEnabled = function()
        local ok, m = pcall(GetMod, "utpp_motion")
        return ok and type(m) == "table" and type(m.want) == "table" and m.want.enabled == true
      end,
      shooterModeLive = function()
        if Game == nil or type(Game.UTPPMotionShooterModeLive) ~= "function" then return nil end
        local ok, r = pcall(Game.UTPPMotionShooterModeLive)
        if ok and type(r) == "boolean" then return r end
        return nil
      end,
      speed = function()
        local p = Game.GetPlayer()
        if p == nil then return nil end
        local v = p:GetVelocity()
        if v == nil then return nil end
        return math.sqrt((v.x or 0) ^ 2 + (v.y or 0) ^ 2)
      end,
    }
  end

  -- The executor is selected once, after startDelay CET frames (the game systems are up by then).
  local startDelay = 300
  -- true while the Game Animations pack is in force (utpp_core): the executor (UTPP Locomotion, body turn) stays off from the
  -- start; the pack switch turns it on again through SetExecutor(true)
  local function gamePackInForce()
    if type(GetMod) ~= "function" then return false end
    local ok, core = pcall(GetMod, "utpp_core")
    if not (ok and type(core) == "table" and type(core.PackIsGame) == "function") then return false end
    local okG, game = pcall(core.PackIsGame)
    return okG and game == true
  end
  local function startExecutor(final)
    if final then
      M.api = gameExecApi()
      if gamePackInForce() then E.enabled = false end
      sink("[utpp_bodyturn] " .. M.SelectExecutor(M.api))
    end
    return final
  end

  registerForEvent("onInit", function()
    if not announced then
      announced = true
      sink("[utpp_bodyturn] onInit: events registered")
    end
    done = startExecutor(false)
    local okC, core = false, nil
    if type(GetMod) == "function" then okC, core = pcall(GetMod, "utpp_core") end
    if okC and type(core) == "table" and type(core.Register) == "function" then
      local okR, err = pcall(M.RegisterSettings, core)
      if not okR then sink("[utpp_bodyturn] settings registration FAILED: " .. tostring(err)) end
    else
      sink("[utpp_bodyturn] utpp_core not found - bodyturn.weaponTurn defaults to on (console: SetWeaponTurn(false))")
    end
  end)

  local execErrors = 0
  registerForEvent("onUpdate", function(dt)
    if not done then
      tries = tries + 1
      done = startExecutor(tries >= startDelay)
      return
    end
    if E.mode == "executor" then
      local okP, player = pcall(Game.GetPlayer)
      if not okP or player == nil then -- no player (menu / loading / exit): no game calls, no stale yaw history
        E.hipFireRemaining, E.hipFireAdsShot, E.hipFireAdsAge = 0, false, 0
        hipFireForget()
        E.hipLeaseSent = nil -- new session: the lease flag is re-synced (false) by the first ExecStep with a player
        E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
        E.strikeStart, E.strikeIdleSpeed = nil, nil -- no strike episode across a load (a held lock is released by the next ExecStep)
        W.dirty, W.sent = true, nil -- new session: the mask is sent again once the player exists
        return
      end
      local ok, err = pcall(M.ExecStep, M.api, dt)
      if not ok then
        execErrors = execErrors + 1
        if execErrors == 1 then sink("[utpp_bodyturn] executor error: " .. tostring(err)) end
      end
    end
  end)

  registerForEvent("onShutdown", function()
    -- game exit: player gone -> no game call (arm / camera hold go through script systems); the DLL disarms on its own
    local okP, p = pcall(Game.GetPlayer)
    local alive = okP and p ~= nil and (pcall(function() return p:GetEntityID() end))
    if not alive then
      sink("[utpp_bodyturn] shutdown: player gone - game calls skipped")
      E.holdOwned, E.locomotionOwned, E.nativeOn = false, false, nil
      E.prevBody, E.prevCam, E.prevStep = nil, nil, 0
      return
    end
    if E.mode == "executor" then pcall(M.DisarmExecutor, M.api, "shutdown") end
  end)
end

return M

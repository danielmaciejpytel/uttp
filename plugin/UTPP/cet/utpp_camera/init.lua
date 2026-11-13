-- UTPP utpp_camera - third-person camera.
-- unarmed ZOOM state (separate from the weapon aim). RMB with EMPTY hands in third person
--        (camera.zoomUnarmed.enabled, PSM UpperBody = 6 for camera cfg zoomUnarmedEnterDelay 0.1 s, nothing in WeaponRight,
--        no melee block / gadget / arm cyberware, no dodge / slide at the entry, no vehicle hold / pose override): over the
--        shoulder like the aim (body behind the camera, free look / idle orbit off, profile zoom masked), pose by step
--        k = 0..N-1 (camera.zoomUnarmed.steps 3): distance camera.zoomUnarmed.distance 0.9 -> lastDistance 0.6 m, FOV
--        camera.zoomUnarmed.fov 45 -> lastFov 28 (45 / 35.5 / 28), blend 0.15 s (enter, exit and every step). Wheel: the
--        game's own ZoomIn / ZoomOut actions (input context "Aiming" while UpperBody = Aim maps IK_MouseWheelUp / Down and
--        the pad d-pad to them) read by the PlayerPuppet.OnAction observer (pressed edge from ListenerAction.GetType, else
--        a value edge). ZoomIn past step N-1
--        (camera.zoomUnarmed.fppHandoff) = "fpp handoff": utpp_perspective.RequestFppZoom(true) (rule "zoom" -> first
--        person, instant) or, without utpp_perspective, the own block rule "unarmed zoom"; the game's Kiroshi zoom is shown.
--        RMB release (UpperBody leaves Aim) = exit: request cleared, third person again (fast return like after aiming),
--        profile / zoom unchanged. Logs "cam zoomUnarmed enter / step k/N / fpp handoff / exit"; Status() zoomUnarmed=.
--        API: ZoomUnarmed() (state copy), ZoomWheel(dir) (console step).
-- world camera ("the camera is a separate object in the world, not part of the character";
--        camera.worldCamera, default on): utppCamera lives in the player entity space, so utpp_bodyturn turning V AFTER
--        the camera in the CET frame carried the camera along for one frame (shimmer on direction changes). Now a
--        separate entity is spawned once per session in gameplay (Codeware StaticEntitySystem.SpawnEntity, fallback CET
--        exEntitySpawner.Spawn) from the template utpp\camera\utpp_player_camera.ent (the same archive: entEntity + one
--        gameCameraComponent "utppCamera", no parent) at the world origin with the identity orientation; it never moves,
--        so the component's local transform IS the world pose: the solver's world pose (res.world / res.q, FOV res.fov) is
--        written as is, independent of V's orientation. It is activated like utppCamera (CameraComponent.Activate(0,
--        audio), like the vanilla device camera take-over deviceBase.script TakeControlOverCamera) and handed back with
--        the FPP camera's Activate + Deactivate as before. The camera in use is chosen per third-person entry (G.use =
--        "world" | "comp"); a world camera that becomes ready in third person takes over at the shown pose. Any failure
--        (no spawner, spawn timeout, space check, entity lost, write / Activate error, watchdog: not rendering) logs
--        "world camera: FAILED <why> -> component camera" and the component path runs unchanged (the prediction /
--        OnBodyStep code only runs for the component). No game handle is cached: the entity id (a value) is kept and the
--        component re-resolved every frame; session end / shutdown without a player forget the id without game calls
--        (Codeware disposes its static entities on world detach). Logs "world camera: spawned <how> entity=<id>",
--        "world camera: ready ...", "world camera: active", "world camera: deactivated (...)", "world camera: released
--        (...)"; camTrace wcam=0|1; Status() wcam=...; API WorldCamera().
-- (1) aim / unarmed zoom keep the shown yaw and
--        pitch and align onto the game's look direction (FPP camera world yaw, fallback body yaw; FPP pitch) at <= 300 deg/s
--        (camera_math CM.alignStep) instead of snapping onto the body yaw within the 0.12 s pose tween (the body faces the
--        movement direction under Locomotion and is turned by the game when the heading lock ends); (2) Locomotion inactive for
--        < camera cfg decoupleHold 0.35 s keeps the free camera (no free/follow flips on flickers); (3) free camera: the
--        entity yaw step of the coming game tick (utpp_bodyturn executor turning V) is predicted from the last body step
--        and taken out of the local transform (camera.predictBodyTurn; the CET frame runs before the game tick, the camera
--        component lives in the entity space); (4) mouse scale calibration: no samples while aiming / zooming / free and
--        0.6 s after the heading lock ends, outliers rejected (CM.calibrate band); (5) camera.trace (default on for the test
--        period): 10 Hz "camTrace ..." line in third person, "aim offset status: ..." (Game.UTPPAimOffsetStatus) at the aim
--        start and every 2 s while aiming.
-- Game object handles (every ExitGame crashed, access violation read 0x8,
--        right after "shutdown: player gone - game calls skipped"; without the muzzle occlusion it exited cleanly):
--        the WeaponObject for the muzzle is no longer cached (it used to be kept in G.aimObj across frames, also after the aim
--        exit): muzzlePos() re-resolves it from the WeaponRight slot on every muzzle sample and drops the local at once;
--        nothing but plain values (G.aimWeapon: record / type / fov) is cached across frames. dropHandles() clears the
--        remaining cached game objects (G.comp, G.fpp, G.player; the G.aimObj field is gone) and runs in
--        resetSession, in both onShutdown paths and when the player is nil; onShutdown also stops the frame loop (until
--        the next onInit) and runs a full GC while Lua still owns the wrappers.
-- TPP signal for utpp.reds: every change of "camera in third person" (entering / tpp / leaving) is
--        reported with Game.UTPPSetTppOverride(on) (fallback: scriptable system UTPPPerspectiveState), one log line per
--        change; this opens the motion matching gate (bit 0x8) and the standalone target facing. Also sent on the
--        first frame (stale value after a CET reload), on a session reset and on shutdown. Pose nudge (camera.poseNudge):
--        0.4 s and 0.9 s after the full body is requested, standing still on the ground, the vanilla "StandEnter"
--        animation event is pushed again (what the game pushes on every stand entry, e.g. after landing) against the
--        T-pose seen after loading until the first jump.
-- Standalone vehicle path (earlier: "exit epoch=N (utpp_perspective: vehicle):
--        camera.Activate ok" at every mount = cut to the game's FPP camera, re-entry from the eyes after the exit).
--        utpp_vehiclecam now drives utppCamera through mount / ride / exit with a small logged API:
--        HoldForVehicle(on, why[, "game"]), SetPoseOverride(pos, yaw, pitch), ReleaseToOrbit(blendS), HandToVehicle(blendS),
--        TakeFromVehicle(blendS), SetVehiclePriority(on), VehicleReady(), VehicleHeld(), VehicleExitWanted(), OrbitPose(),
--        SetVehicleDelegate(fn, name). Vehicle API (M.VehicleApiVersion 2): HoldForVehicle(on, why, mode, blendS) ("game"
--        also before a hand-over), TakeForVehicleExit(pos, yaw, pitch, blendS, why[, fov]) (exit take-over from the game's
--        vehicle camera), SetPoseOverride(pos, yaw, pitch, why[, fov]), optional VehicleJumpOut(on, why) (jump-out landing mode of the pivot); no FPP camera activation while the vehicle
--        camera renders in third person for the driver. While held the "vehicle" FPP decision is ignored (no camera.Activate, no FPP
--        handoff), the camera stays in its TPP state (TPP signal ON), the watchdog / calibration / heading lock are off and
--        the written pose is the override (world position, yaw, pitch) or the release blend to the orbit. A vehicle FPP
--        decision while in TPP and not held asks the registered delegate (utpp_vehiclecam) first - one logged call.
-- Camera behaviour:
--        (1) pivot height from the capsule: player position + stance height (stand 1.55 m / crouch 1.05 m from the PSM
--            Locomotion crouch states, blended 0.25 s), no longer the animated FPP camera (it drops in the game's FPP
--            melee / block / vehicle-exit sets = "the camera lowers as if V crouched"). camera.pivotSource = capsule|fpp;
--            log "pivot source -> ..." and "pivot stance -> crouch|stand (...)".
--        (2) follow mode yaw limiter: the mouse part of V's turn passes at once, the rest of a body turn (motion matching /
--            target facing, recentering after the idle orbit) at <= camera.followMaxRate deg/s with ease-in; a body flip
--            (> 120 deg within 0.3 s without the mouse) holds the world yaw 0.6 s (camera.followHoldOnFlip), log
--            "follow: body turned ... -> camera holds its world yaw ...".
--        (3) return from aiming: an FPP -> TPP request after the aiming rule (utpp_perspective rule "weapon": aiming /
--            gadget / arm cyberware / weapon drawn; own rule "weapon") enters over camera.aimReturnBlend (0.35 s) with the
--            orbit yaw reset behind the back (camera.aimReturn), log "aim return (...)".
--        (4) OrbitPoseAt(yaw, pitch): orbit pose around V for a given yaw / pitch (utpp_vehiclecam jump-out follow).
--        (5) pose nudge also 0.1 s and 0.5 s after a game menu closes (UI_System.IsInMenu / pause -> off) in third person
--            (camera.poseNudge, same guards), log "menu closed after ... s: pose nudge scheduled ...".
-- over-the-shoulder AIM state. In 2.31 the
--        player's shots and GetDefaultCrosshairData use the ACTIVE camera (probe), so the camera forward is the
--        aim direction. Enter: camera.aim.enabled, perspective.aiming = tpp (utpp_core), third person (entering / tpp,
--        wanted), PSM UpperBody = 6 (Aim), a ranged item in WeaponRight (not fists / melee type / PSM MeleeWeapon 5-10),
--        no vehicle hold / pose override (they win). While aiming: forced "body behind the camera" (follow mode: free
--        look / idle orbit off, yaw / pitch offset onto the FPP aim with the blend in, no follow limiter lag, pitch
--        from the FPP camera as before, wider pitch range), pose over the live profile: camera.aim.distance 0.8 m behind
--        the capsule pivot, camera.aim.side 0.45 m on the current shoulder, FOV = weapon stat AimFOV (StatsSystem,
--        accepted 20-90) or camera.aim.fov 42, blend in 0.12 s / out 0.20 s (FOV follows the tween), zoom keys masked,
--        collision rays as usual; camera.aim.autoSwap: shoulder-side ray (0.5 m) blocked for 0.3 s -> shoulder swap
--        when the other side is clearer. Exit when any condition drops: back to the profile over 0.20 s, the orbit stays
--        behind the back (the first-person aim return is unchanged). Logs "cam aim enter weapon=... fov=...
--        side=L|R", "cam aim exit (<reason>) after <s> s", "cam aim swap -> L|R (wall <d> m)"; Status() aim= aimT=.
--        Aim occlusion, level A (detection only - the shot itself is unchanged): while the aim
--        weight > 0.5, ray 1 along the camera forward of this frame from the muzzle plane t0 = max(0, dot(muzzle - cam,
--        fwd)) to camera.aim.range (60 m) -> aim point P; ray 2 muzzle (WeaponObject.GetMuzzleSlotWorldTransform of the
--        WeaponRight object read by the aim observer) -> P; blocked = ray 2 hit more than 0.15 m before P (P less than
--        0.1 m in front of the muzzle = not blocked). Same collision groups / raycast as the camera collision. At most
--        every 2nd frame and 30 Hz, hysteresis 0.1 s on both edges; camera.aim.occlusion (default on). API: AimBlocked(),
--        AimBlockInfo(); Status() aimBlocked= aimBlockD=; logs "aim blocked d=<m> (muzzle->P hit at <m> of <m>)" (d = the
--        muzzle -> obstacle distance) / "aim clear after <s> s".
--
--   * camera component "utppCamera" added to the 4 player entities by the ArchiveXL patch utpp_camera.archive(.xl)
--     (UTPP archive folder); no parent binding, so it lives in the player root space;
--   * orbit: V follows the camera (the game turns V with the mouse; pitch from the FPP camera so aiming and
--     interactions keep working) or free look / idle orbit (own yaw/pitch from mouse and right stick, V does not turn);
--   * distance / zoom, shoulder offset + swap, height, FOV of one fixed camera profile (the five stored
--     profiles stay in utpp_settings.txt, only profile 2 is used);
--   * collision: 1-5 rays per frame (SpatialQueriesSystem.SyncRaycastByCollisionGroup), closer at once, back smoothly;
--   * transitions FPP <-> TPP: the boom grows from / shrinks into the eyes, then the FPP camera takes over;
--   * body: the game's own TPP representation (ActivateTPPRepresentationEvent + anim feature) behind a setting;
--   * "Use Ultimate Third Person" ON/OFF switch (camera.impl);
--   * no key bindings of its own (the UTPP key-binding system was removed); toggle / swap / zoom / free look
--     stay as console and module APIs;
--   * consumers of the parallel modules when installed: utpp_perspective (decides TPP/FPP)
--     and utpp_body (SetTppBody: full body
--     with ownership); without them the camera uses its own toggle and the plain TPP representation.
-- Pure math: camera_math.lua.
--
-- Console: local c = GetMod("utpp_camera")
--   c.Toggle()  c.Enter()  c.Exit()  c.Swap()  c.Zoom(1|-1)  c.FreeLook(true|false)  (c.Profile / c.NextProfile: fixed 2)
--   print(c.Status())  c.Diag(5) (5 s of per-frame values in the UTPP log)  c.SetEnabled(true|false|nil) (test override)
--   c.NudgePose() (push "StandEnter" now, same conditions as the automatic nudge)
--   c.OrbitPoseAt(yaw, pitch) (orbit pose around V for the given yaw / pitch; nil outside third person)
--   print(c.VehicleStatus())  (vehicle hold: held / handed / priority / override / release + counters)
--   print(c.Status())  ... aim=on|off aimT=<s> (aim state, time in the current aim state)
--   c.AimBlocked()  c.AimBlockInfo()  (aim occlusion: muzzle blocked while the camera ray is clear)
--   c.ZoomUnarmed()  c.ZoomWheel(1|-1)  (unarmed zoom: state copy / one wheel step while the state is on)

local M = { VERSION = "0.1.6" }

-- ================================================================ math module
local CM
do
  local ok, mod = pcall(require, "camera_math")
  if not ok or type(mod) ~= "table" then ok, mod = pcall(dofile, "camera_math.lua") end
  if (not ok or type(mod) ~= "table") and type(UTPP_CAMERA_MATH_PATH) == "string" then ok, mod = pcall(dofile, UTPP_CAMERA_MATH_PATH) end
  if ok and type(mod) == "table" then CM = mod else error("utpp_camera: camera_math.lua not found: " .. tostring(mod)) end
end
M.CM = CM

-- ================================================================ state
M.cfg = CM.config()
M.profiles = {}
for i, p in ipairs(CM.DEFAULT_PROFILES) do M.profiles[i] = CM.copyProfile(p) end
M.rt = { profileIndex = 2, profileCount = 3, zoom = 0, freeHeld = false, override = nil, body = true, audio = true,
         fppWithWeapon = true, mouseScale = 0, startTpp = false, profileBlend = 0.5, poseNudge = true,
         aimReturn = true, aimReturnBlend = 0.35,
         trace = false, -- camera.trace (10 Hz camTrace lines in third person)
         worldCamera = true, -- camera.worldCamera (separate camera entity in world space)
         worldCameraRetry = true, -- a failed world camera is spawned again (WC_FAIL_RETRY s, WC_FAIL_MAX times)
         -- "Vehicle" added - the vault over a parked car put the camera inside the body (hits=0 in camTrace);
         -- with the group the boom shortens (hits 2-3, 2.8 -> 1.8 m) and returns after the landing
         groups = { "Static", "Terrain", "Destructible", "Dynamic", "Vehicle" } }
M.T = CM.newTransition(true)
M.S = CM.newSolver(M.cfg)
M.blend = CM.newBlend(M.profiles[M.rt.profileIndex])
M.shoulder = CM.newShoulder(1)
M.cal = CM.newCalibration()
M.core = nil
M.stats = { frames = 0, writes = 0, rays = 0, errors = 0, reactivations = 0, cmds = {}, aimReturns = 0, flips = 0,
            menuNudges = 0, aimSwaps = 0, lockRelocks = 0, lockSkips = 0 }
M.aim = CM.newAim()   -- aim state (CM.aimStep)
M.aimFov = nil        -- FOV of the current aim (weapon AimFOV stat or camera.aim.fov)
M.occ = CM.newAimOcc() -- aim occlusion flag (CM.aimOccStep)
M.apt = CM.newAimPoint() -- crosshair point P of the native aim / camera targets (CM.aimPointStep)
M.zoomU = CM.newZoomUnarmed() -- unarmed zoom state (CM.zoomUnarmedStep)
-- unarmed zoom inputs: wheel = pending wheel steps (+ = closer), held = s of UpperBody 6, requested = first person
-- handoff requested (route), seen = game zoom actions observed (logged once each)
M.zuIn = { wheel = 0, src = nil, held = 0, requested = false, route = nil, seen = {}, gameZoom = nil }
-- world camera: template (in utpp_camera.archive), tag (leftovers of an earlier CET instance), state.
-- state: off | spawning | verify | ready | failed. id = EntityID value (no handle), route = static | ex.
M.WC_TEMPLATE = "utpp\\camera\\utpp_player_camera.ent"
M.WC_TAG = "UTPPWorldCamera"
M.WC_SPAWN_TIMEOUT = 5.0   -- s of gameplay (no menu) until the spawned entity must resolve
M.WC_READY_TIMEOUT = 10.0  -- s of retries while the spawner reports "not ready"
M.WC_CHECK_TIME = 1.0      -- s for the space check (component world position = written local position)
-- the world camera spawned 1.5 s after a save
-- load, the watchdog saw the game's active camera 3.76 m away (the loading camera was still shown) and FAILED it for
-- good after 2 re-activations -> component camera and "game look write OFF (world camera not shown)" for the rest of the
-- session. camera.worldCameraRetry (default on; off = the old permanent failure): a failed world camera is spawned
-- again WC_FAIL_RETRY s later, at most WC_FAIL_MAX times per game session; a camera ready in third person takes over at
-- the shown pose as before and the game look write restarts with it. Status: failRetries= retryIn=.
M.WC_FAIL_RETRY = 8.0      -- s after a failure before the next spawn attempt
M.WC_FAIL_MAX = 3          -- spawn attempts after failures per game session
local function newWc()
  return { state = "off", id = nil, idStr = nil, route = nil, how = nil, since = 0, waited = 0, check = nil, why = nil,
           firstTry = nil, retryAt = nil, reacts = 0, spawns = 0, fails = 0, activations = 0, switches = 0, releases = 0,
           failRetries = 0, failAt = nil, failRetryAt = nil }
end
M.wc = newWc()
function M.WorldCamera()
  local W = M.wc
  return { state = W.state, use = M.G and M.G.use or nil, route = W.route, how = W.how, id = W.idStr, why = W.why,
           spawns = W.spawns, fails = W.fails, activations = W.activations, switches = W.switches, releases = W.releases,
           enabled = M.rt.worldCamera ~= false }
end
function M.WorldCameraStatus()
  local W = M.wc
  local retryIn = "-"
  if W.state == "failed" and M.rt.worldCameraRetry ~= false and W.failRetries < M.WC_FAIL_MAX and W.failRetryAt and M.G then
    retryIn = string.format("%.1f", math.max(0, W.failRetryAt - (M.G.clock or 0)))
  end
  return string.format("wcam=%s use=%s route=%s entity=%s spawns=%d fails=%d activations=%d switches=%d releases=%d%s",
    W.state, tostring(M.G and M.G.use or "-"), tostring(W.route or "-"), tostring(W.idStr or "-"), W.spawns, W.fails,
    W.activations, W.switches, W.releases, W.why and (" why=" .. tostring(W.why)) or "")
    .. string.format(" failRetries=%d retryIn=%s", W.failRetries or 0, retryIn) -- (append-only)
end

-- ================================================================ logging
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
M.logTail = {}
function M.Log(msg)
  local line = "[utpp_camera] " .. tostring(msg)
  M.logTail[#M.logTail + 1] = line
  if #M.logTail > 80 then table.remove(M.logTail, 1) end
  if M.core and M.core.Get and M.core.Get("core.log") == false then return end
  if M.sink then M.sink(line) return end
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
end
local once = {}
function M.LogOnce(key, msg)
  if once[key] then return end
  once[key] = true
  M.Log(msg)
end

-- ================================================================ enabled switch
function M.Enabled()
  if M.rt.override ~= nil then return M.rt.override end
  if M.core and M.core.Effective then
    local v = M.core.Effective("camera.impl")
    if v ~= nil then return v and true or false end
  end
  return true
end
function M.SetEnabled(v)
  M.rt.override = v
  M.Log("enabled override = " .. tostring(v) .. " -> " .. tostring(M.Enabled()))
  return M.Enabled()
end

-- ================================================================ TPP signal for utpp.reds
-- M.tppSender(on) -> ok, route|error  (installed by the game adapter; tests may replace it).
-- Sent only on a change; a failed send keeps the old value and is retried after retryS (one log line per failure streak).
M.tppSig = { sent = nil, route = nil, changes = 0, failures = 0, failLogged = false, retryAt = -1, retryS = 1.0, last = nil }
function M.SignalTpp(on, why, clock)
  on = on and true or false
  if not on then M.PoseWriteTpp(false) end
  local s = M.tppSig
  if s.sent == on then return false end
  clock = tonumber(clock) or 0
  if s.retryAt >= 0 and clock < s.retryAt and s.pending == on then return false end
  s.pending = on
  local ok, info = false, "no sender (game adapter not loaded)"
  if type(M.tppSender) == "function" then
    local okC, r1, r2 = pcall(M.tppSender, on)
    if okC then ok, info = r1 == true, r2 else info = "error " .. tostring(r1) end
  end
  if ok then
    local initial = s.sent == nil
    s.sent, s.route, s.changes, s.failLogged, s.retryAt, s.last = on, tostring(info), s.changes + 1, false, -1, why
    M.Log(string.format("TPP signal -> %s (%s%s) via %s", on and "ON" or "OFF", tostring(why), initial and ", initial sync" or "",
      tostring(info)))
    return true
  end
  s.failures = s.failures + 1
  s.retryAt = clock + s.retryS
  if not s.failLogged then
    s.failLogged = true
    M.Log(string.format("TPP signal -> %s FAILED (%s): %s - utpp.reds 0.6.1 installed? (retry every %.1f s)",
      on and "ON" or "OFF", tostring(why), tostring(info), s.retryS))
  end
  return false
end
-- camera states that count as third person for the game side (utppCamera is the rendering camera)
function M.TppSignalState(T) return CM.trActive(T or M.T) end

-- ================================================================ head hide while the camera is inside V
-- The native HeadGuard sets the game's near-camera threshold (0.45 m squared = 0.2025) to its TPP value 0 ("never near") while
-- third person is on, so the vanilla near-camera hide of the head / hair never fires there. While the camera is within the body
-- clearance (CM.solve res.inside: a near surface pushed the boom inside it, or the start / end of the enter / exit blend near the
-- eye point), the TPP value goes back to the vanilla 0.2025 (Game.UTPPHeadGuardSetValue) - on the first inside frame; back to 0
-- (HeadGuard's default) after releaseDelay s outside (no flapping) and at once on every exit (camera inactive, session reset,
-- shutdown). Only a threshold changes: the blend timing and the camera pose are not touched. First person is not affected (the
-- guard restores the original constant there). The TPP signal (MM, locomotion, item plane) is untouched. On foot only: the
-- caller passes inside = false under a vehicle hold, mounted and within vehicleQuiet s after a hold (hand-over / take-over).
-- Log: one line per edge caused by a surface (res.insideWall); blend-only edges (every enter / exit) only with camera.trace. A
-- missing native (older DLL) = one "unavailable" line, nothing else; a native answering false = one FAILED line.
-- M.headGuardValue(v) -> ok, info, missing (installed by the game adapter; tests may replace it). INSIDE_HIDE.enabled = false:
-- never set.
M.INSIDE_HIDE = { enabled = true, insideValue = 0.2025, outsideValue = 0, releaseDelay = 0.25, vehicleQuiet = 1.0 }
M.insideHide = { on = false, outSince = nil, failLogged = false, edges = 0, loud = false, missing = false }
local function insideHideSet(v)
  if type(M.headGuardValue) ~= "function" then return false, "no HeadGuard route (game adapter not loaded)", true end
  local okC, r1, r2, r3 = pcall(M.headGuardValue, v)
  if okC then return r1 == true, r2, r3 == true end
  return false, "error " .. tostring(r1)
end
local function insideHideOff(why)
  local s, cfg = M.insideHide, M.INSIDE_HIDE
  local ok, info = insideHideSet(cfg.outsideValue)
  s.on, s.outSince = false, nil
  if s.loud or not ok then
    M.Log(string.format("camera out of the body clearance (%s) -> near-camera head hide back to the TPP value %.4f%s", tostring(why),
      cfg.outsideValue, ok and "" or (" FAILED: " .. tostring(info))))
  end
  return true
end
-- inside (bool) of this frame, why = the boom length (m) or a reason text, wall = a surface caused it (res.insideWall);
-- -> true when the HeadGuard value changed
function M.InsideHideStep(inside, clock, why, wall)
  local s, cfg = M.insideHide, M.INSIDE_HIDE
  clock = tonumber(clock) or 0
  if inside == true and cfg.enabled ~= false then
    s.outSince = nil
    if s.on or s.missing then return false end
    if type(why) == "number" then why = string.format("boom %.2f m%s", why, wall and "" or ", enter / exit blend") end
    local ok, info, missing = insideHideSet(cfg.insideValue)
    if ok then
      s.on, s.edges, s.loud = true, s.edges + 1, wall == true or M.rt.trace == true
      if s.loud then
        M.Log(string.format("camera inside the body clearance (%s) -> vanilla near-camera head hide ON (HeadGuard TPP value %.4f)",
          tostring(why), cfg.insideValue))
      end
      return true
    end
    if missing then -- older UTPP.dll / scripts: not an error, the feature is simply absent for this session
      s.missing = true
      M.Log("near-camera head hide while the camera is inside V: unavailable (" .. tostring(info) .. ") - older UTPP.dll?")
    elseif not s.failLogged then
      s.failLogged = true
      M.Log("camera inside the body clearance: near-camera head hide FAILED (" .. tostring(info) .. ") (logged once)")
    end
    return false
  end
  if not s.on then return false end
  if cfg.enabled == false then return insideHideOff("disabled") end
  if s.outSince == nil then s.outSince = clock end
  if clock - s.outSince < (cfg.releaseDelay or 0) then return false end
  return insideHideOff(type(why) == "number" and string.format("boom %.2f m", why) or why)
end
-- immediate release (camera inactive / FPP / session reset / shutdown); nothing when the hide is not on
function M.InsideHideRelease(why)
  if not M.insideHide.on then return false end
  return insideHideOff(why)
end

-- Separate from the debounced script TPP state: a per-frame native write veto.
-- No cached acknowledgement: false must always revoke a stale positive heartbeat.
function M.PoseWriteTpp(on)
  if on ~= true then M.FeedCameraTarget(nil) end
  local ok, accepted = pcall(function() return Game.UTPPSetPoseWriteTpp(on == true) end)
  return ok and accepted == true
end
-- Solved displayed-camera ray for native TPP pose targeting, independent of ADS.
-- This heartbeat changes no camera/FOV settings; the legacy ADS target is separate.
-- d (optional) = distance of the crosshair point P along this ray (M.AimPointUpdate); the native target is P
-- then (CM.aimPointRay), nil = the camera ray as before
function M.FeedCameraTarget(res, d)
  local enabled = res ~= nil
  if not enabled then CM.aimPointReset(M.apt) end
  if not enabled and not M.cameraTargetSent then return true end
  local org = enabled and res.world or {x=0,y=0,z=0}
  local fwd = enabled and CM.forward(res.yaw, res.pitch) or {x=0,y=1,z=0}
  if enabled and d ~= nil then org, fwd = CM.aimPointRay(org, fwd, d) end
  local ok, err = pcall(function()
    Game.UTPPSetCameraTarget(Vector4.new(org.x,org.y,org.z,1), Vector4.new(fwd.x,fwd.y,fwd.z,0), enabled)
  end)
  if ok then M.cameraTargetSent = enabled
  elseif not M.cameraTargetUnavailable then
    M.cameraTargetUnavailable = true
    M.Log("camera target: UTPPSetCameraTarget unavailable (" .. tostring(err) .. ") - older DLL")
  end
  -- Disable is best effort once; the native heartbeat also expires on its own.
  if not enabled then M.cameraTargetSent = false end
  return ok
end
-- crosshair point P for the native aim / camera targets (CM.AIM_POINT): once per frame before both feeds.
-- res = the camera solved this frame, eye = FPP camera position (nil = unknown), ray = raycast (nil = none), armed =
-- predicate for the own ray (weapon drawn / aiming; nil = always). Distance source: the parallax look ray of the last
-- frame (G.lwpDist) when fresh and past the eye plane, else an own camera forward ray from rayPastEye m past the eye
-- plane to camera.aim.range m (at most rayHz). Returns the filtered distance of P along the camera ray or nil (no P).
function M.AimPointUpdate(res, eye, ray, dt, armed)
  local ap, st, G = CM.AIM_POINT, M.apt, M.G or {}
  if ap.enabled ~= true or res == nil or res.world == nil then
    CM.aimPointReset(st)
    if M.aimPointOn == true then
      M.aimPointOn = false
      M.Log("aim point: off - camera / eye ray targets")
    end
    return nil
  end
  local clock = G.clock or 0
  local fwd = CM.forward(res.yaw, res.pitch)
  local range = math.max(tonumber(M.cfg.aimRange) or 60, 1)
  local farD = math.max(range, ap.targetDist)
  local known = eye ~= nil or st.ahead ~= nil
  local ahead = st.ahead or 0
  if eye then ahead = math.max(0, CM.dot(CM.sub(eye, res.world), fwd)) end
  local start = math.max(CM.clamp(tonumber(M.cfg.aimLookNear) or 0.3, 0, range * 0.5), ahead + ap.rayPastEye)
  local raw = nil
  if not known then
    -- eye plane never known: an own ray could start on V / behind her (hipfire boom) -> no P (the old targets)
  elseif G.lwpDist and G.lwpAt and G.lwpAt ~= st.lookAt and clock - G.lwpAt <= math.max(1 / ap.rayHz, dt or 0) + 1e-6
      and G.lwpDist >= start then
    st.lookAt = G.lwpAt
    raw = G.lwpDist >= range - 1e-6 and farD or G.lwpDist
  elseif type(ray) == "function" and start < range and (st.rayAt == nil or clock - st.rayAt >= 1 / ap.rayHz - 1e-6) then
    st.rayAt = clock
    if armed == nil or armed() then
      local hit = ray(CM.add(res.world, CM.scale(fwd, start)), CM.add(res.world, CM.scale(fwd, range)))
      raw = hit and math.min(farD, start + math.max(0, hit)) or farD
    end
  end
  if known then st.ahead = ahead end
  local d = CM.aimPointStep(st, ap, dt, clock, raw, ahead, tonumber(M.cfg.aimLookMinAhead) or 1.0, farD)
  st.p = d and CM.add(res.world, CM.scale(fwd, d)) or nil
  if (d ~= nil) ~= (M.aimPointOn == true) then
    M.aimPointOn = d ~= nil
    M.Log(d and string.format("aim point: native targets = crosshair point P (%.2f m along the camera ray)", d)
      or "aim point: no crosshair point (no ray result) - camera / eye ray targets")
  end
  return d
end
-- last crosshair point P of the native targets: world point (copy) and distance along the camera ray, or nil (probe / API)
function M.AimPoint()
  local st = M.apt
  if st == nil or st.p == nil then return nil end
  return { x = st.p.x, y = st.p.y, z = st.p.z }, st.d
end
-- public switch of the crosshair point feed (CM.AIM_POINT.enabled, shipped false). utpp_motion drives it together with the DLL
-- barrel loop (mm.aimAtCrosshair): P alone (open-loop aim offset from the fixed chest) is never wanted. Off: the next AimPointUpdate
-- clears the filter and the targets are exactly the old camera / eye rays. Returns the new state.
function M.SetAimPoint(on)
  CM.AIM_POINT.enabled = on == true
  return CM.AIM_POINT.enabled
end
function M.AimPointEnabled() return CM.AIM_POINT.enabled == true end
-- held = vehicle camera hold: it vetoes while V is mounted (the game's seated pose is wanted), not after the unmount
-- (onFoot, optional, default unchanged): then the MM pose write is allowed again during the rest of the exit hold.
function M.PoseWriteTppWanted(T, freeze, camOK, res, held, commands, onFoot)
  if not T or T.state ~= "tpp" or T.want ~= true or freeze or not camOK or res == nil or (held and onFoot ~= true) then return false end
  for _, command in ipairs(commands or {}) do
    if command == "fpp_on" or command == "cam_off" or command == "rep_off" then return false end
  end
  return true
end
-- diagnostic: which clause of PoseWriteTppWanted vetoes the write ("ok" when none; "command" = a command veto)
function M.PoseWriteDeniedReason(T, freeze, camOK, res, held, onFoot)
  if not T then return "noT" end
  if T.state ~= "tpp" then return "state=" .. tostring(T.state) end
  if T.want ~= true then return "want=false" end
  if freeze then return "freeze" end
  if not camOK then return "camNotOK" end
  if res == nil then return "res=nil" end
  if held and onFoot ~= true then return "held(mounted)" end
  return nil
end

-- ================================================================ vehicle path (API for utpp_vehiclecam)
-- M.vehOps (installed by the game adapter; tests may replace it): activate(blend), deactivate(blend), prio(on), writeNow()
-- -> true | false, why. Poses are world space: pos { x, y, z }, yaw (deg, forward = (-sin, cos)), pitch (deg, + = up).
M.vh = { held = false, why = nil, since = nil, handed = false, prio = false, override = nil, release = nil, last = nil,
         clock = 0, delegate = nil, delegateName = nil, pending = nil, lastWant = nil, holds = 0, refusals = 0,
         handovers = 0, takes = 0, releases = 0, toGame = 0, delegated = 0, ignored = 0, overrides = 0,
         onFoot = false } -- onFoot: set by utpp_vehiclecam at the unmount (see M.VehicleOnFoot)
local function vhOp(name, ...)
  local ops = M.vehOps
  if type(ops) ~= "table" or type(ops[name]) ~= "function" then return false, "no game adapter" end
  local ok, r, e = pcall(ops[name], ...)
  if not ok then return false, tostring(r) end
  if r == false then return false, tostring(e or "failed") end
  return true
end
local function okStr(ok, err) return ok and "ok" or ("FAILED " .. tostring(err)) end
local function poseStr(p)
  if not p then return "-" end
  return string.format("(%.2f,%.2f,%.2f) yaw=%.1f pitch=%.1f", p.x, p.y, p.z, p.yaw or 0, p.pitch or 0)
end
-- the camera can be held for a vehicle: enabled and in (or entering) its third person
function M.VehicleReady()
  if not M.Enabled() then return false, "UTPP camera off" end
  local T = M.T
  if not (CM.trActive(T) and T.want) or T.state == "leaving" then
    return false, "camera not in third person (state " .. tostring(T.state) .. ")"
  end
  return true, "third person (" .. T.state .. ")"
end
function M.VehicleHeld() return M.vh.held end
-- utpp_vehiclecam reports a positively identified car (bikes / unknown classes: false); gates vehHoldColRate
function M.SetVehicleCar(on) M.vh.car = on == true; return M.vh.car end
-- exit hold wanted: false when utpp_perspective reports that the user chose first person
function M.VehicleExitWanted()
  local P = M.Perspective()
  if P then
    local ok, d = pcall(P.Desired)
    if ok and type(d) == "table" and d.wantsTpp == false then return false, "user prefers first person" end
  end
  return true
end
-- on: the camera stays third person through the vehicle decision (refused unless VehicleReady). off: mode nil = back to
-- the camera's own rules (utppCamera re-activated when it was handed over); mode "game" = the game's vehicle camera keeps
-- the view: first person state without an FPP camera activation, body released.
-- mode "game" also when NOT handed over (a mount hold that gives the view to the vehicle
-- camera): the shown camera is deactivated with the engine blend blendS (nil = 0.30 s) and forgotten without a second
-- Deactivate (cam_drop), so the blend is not cut. API version: M.VehicleApiVersion (2 = this, TakeForVehicleExit, fov).
M.VehicleApiVersion = 2
M.VEH_GAME_BLEND = 0.30
function M.HoldForVehicle(on, why, mode, blendS)
  local vh = M.vh
  why = tostring(why or "utpp_vehiclecam")
  if on then
    if vh.held then
      vh.onFoot = false -- a mount while the hold is still running: mounted again, the seated pose is wanted
      vh.airAt, vh.landWait, vh.exitHold, vh.gov, vh.govTail = nil, nil, false, nil, nil
      -- (JO2): a re-mount 0.7 s into the release to the orbit of the previous exit: the release_end of
      -- that release turned the new ride's hold off 0.08 s later ("SetPoseOverride=FAILED not held", the next exit ran without a hold).
      -- The release is not cut: cancelling it made the shown pose jump from the half-blended pose to the release target (up to the whole
      -- vehicle-camera offset and the pitch gap in one frame). It is flagged instead (rl.remount) and goes on blending from the shown pose;
      -- at its end it only hands yaw / pitch to the look state and the hold stays on (VehicleCompose: no release_end, no landWait).
      local rl = vh.release
      if rl ~= nil and not rl.remount then
        rl.remount = true
        M.Log(string.format("vehicle hold: the running release to the orbit (%.2f s of %.2f s) is cancelled by the new mount (%s) - hold kept, "
          .. "the blend finishes from the shown pose", (vh.clock or 0) - (rl.at or 0), rl.dur or 0, why))
      end
      return true, "already held"
    end
    local ok, reason = M.VehicleReady()
    if not ok then
      vh.refusals = vh.refusals + 1
      M.Log(string.format("vehicle hold refused (%s): %s - the camera keeps its own rules", why, reason))
      return false, reason
    end
    vh.held, vh.why, vh.since, vh.handed, vh.override, vh.release, vh.lastWant = true, why, vh.clock, false, nil, nil, nil
    vh.onFoot = false
    vh.frameOff, vh.frameLogged, vh.composeAt, vh.frameVel = 0, false, nil, nil
    vh.hist, vh.basePitch, vh.dtEma = nil, nil, nil
    vh.airAt, vh.airZ, vh.landWait, vh.exitHold, vh.gov, vh.govTail = nil, nil, nil, false, nil, nil
    if M.S and M.S.col then M.S.col.press, M.S.col.inV = 0, 0 end -- a pull-in delay timer of an earlier hold does not carry over
    vh.holds = vh.holds + 1
    M.Log(string.format("vehicle hold ON (%s): utppCamera stays active through the vehicle decision (no FPP handoff), "
      .. "TPP signal stays on", why))
    return true
  end
  if not vh.held then return true, "not held" end
  local prio = ""
  if vh.prio then
    local okP, errP = vhOp("prio", false)
    vh.prio = false
    prio = ", SetIsHighPriority(false) " .. okStr(okP, errP)
  end
  local handed = vh.handed
  vh.held, vh.override, vh.release, vh.handed = false, nil, nil, false
  vh.onFoot = false
  -- The first aim after a vehicle hold blends in softer; own pitch for cfg.ownPitchHold s more
  vh.endedAt, vh.aimSoftAt, vh.frame, vh.frameOff, vh.frameLogged, vh.composeAt, vh.frameVel = vh.clock, vh.clock, nil, 0, false, nil, nil
  vh.airAt, vh.airZ, vh.landWait, vh.exitHold, vh.gov, vh.govTail = nil, nil, nil, false, nil, nil
  if mode == "game" then
    local T = M.T
    local deact = ""
    if handed then
      vh.pending = { "rep_off", "cam_off" }
    else
      -- (2): not handed over - utppCamera / the world camera still renders: engine blend to the vehicle camera
      local b = math.max(0, tonumber(blendS) or M.VEH_GAME_BLEND)
      local okD, errD = vhOp("deactivate", b)
      deact = string.format(", Deactivate(%.2f) %s", b, okStr(okD, errD))
      vh.pending = { "rep_off", "cam_drop" }
    end
    T.state, T.w, T.want, T.last = "fpp", 0, false, "vehicle hold: game camera"
    T.epoch, T.exits = T.epoch + 1, T.exits + 1
    vh.toGame = vh.toGame + 1
    M.Log(string.format("vehicle hold OFF (%s): view left to the game's vehicle camera (no FPP activation), body "
      .. "released, TPP signal off%s%s", why, prio, deact))
    return true
  end
  local extra = ""
  if handed and CM.trActive(M.T) then
    local okA, errA = vhOp("activate", 0)
    extra = ", utppCamera.Activate(0) " .. okStr(okA, errA)
  end
  M.Log(string.format("vehicle hold OFF (%s): camera back to its own rules%s%s", why, prio, extra))
  return true
end
-- utpp_vehiclecam: V left the vehicle (unmount). Only meaningful while held; cleared by the next HoldForVehicle / release.
-- The hold keeps the camera but no longer vetoes the MM pose write (otherwise the game's vanilla post-exit pose is shown).
function M.VehicleOnFoot(on, why)
  local vh = M.vh
  if not vh.held then return true, "not held" end
  on = on == true
  if vh.onFoot ~= on then
    vh.onFoot = on
    M.Log(string.format("vehicle hold: on foot = %s (%s) - MM pose write %s", tostring(on), tostring(why or "utpp_vehiclecam"),
      on and "allowed again during the hold" or "vetoed during the hold"))
  end
  return true
end
-- world pose written instead of the orbit while held (nil = orbit again). Written at once and on every camera update.
-- optional fov (camera component units, accepted 10-120) written with the pose; nil = the
-- profile FOV of the solve as before. The release to the orbit blends a written override FOV back to the profile FOV.
local function vehFov(v)
  v = tonumber(v)
  if v == nil or v ~= v or v < 10 or v > 120 then return nil end
  return v
end
M.vehFov = vehFov
-- keepYaw = true (utpp_vehiclecam mount pull-out): the hidden orbit keeps its world yaw under this
-- override like the hold before it (holdYaw) instead of snapping onto V's body, which the mount slide spins +67 deg and
-- back; nil / false = the previous follow (exit hold, ride retake). Older callers pass 5 arguments (= follow).
function M.SetPoseOverride(pos, yaw, pitch, why, fov, keepYaw)
  local vh = M.vh
  if not vh.held then return false, "not held" end
  if pos == nil then
    vh.gov, vh.govTail = nil, nil
    if vh.override then
      vh.override = nil
      M.Log(string.format("vehicle pose override cleared (%s)", tostring(why or "utpp_vehiclecam")))
    end
    return true
  end
  local first = vh.override == nil
  vh.override = { x = tonumber(pos.x) or 0, y = tonumber(pos.y) or 0, z = tonumber(pos.z) or 0, yaw = tonumber(yaw) or 0,
                  pitch = tonumber(pitch) or 0, fov = vehFov(fov), keepYaw = keepYaw == true }
  -- exit pose governor (cfg.vehExitGov, armed by the exit take): the requested pose is played through CM.poseGovStep; the governed
  -- pose is the override (VehicleCompose and writeNow show it, the release starts from it)
  local g = vh.gov
  if g ~= nil and vh.clock ~= nil then
    -- V's own speed (the cap's reference and the jump-out test below), low passed over 0.1 s: one-frame pops of the entity do not open the cap
    local dt = math.max(0, vh.clock - (g.clock or vh.clock))
    local fr = vh.frame
    if fr and fr.ppos and dt > 1e-4 then
      if g.pp then
        local sp = CM.dist(fr.ppos, g.pp) / dt
        g.ref = (g.ref or sp) + (sp - (g.ref or sp)) * math.min(1, dt / 0.1)
      end
      g.pp = CM.v3(fr.ppos.x, fr.ppos.y, fr.ppos.z)
    end
  end
  -- a jump-out (the landing mode armed, or V seen in flight after the unmount): the request follows V, which the speed cap must not hold back; the
  -- governor hands its remaining offset over as a decaying tail (CM.poseGovTail, continuous) and the requests are shown as they come after it
  -- (a rolling exit that utpp_vehiclecam classed as a jump-out - the vehicle above its exitFollowSpeed, V on the ground below vehExitGovJumpSpeed m/s,
  -- e.g. 4.2 m/s, its follow 3.85 m in 0.25 s - stays governed)
  if g ~= nil and (vh.airAt ~= nil or (vh.exitHold == true and (g.ref or 0) > (M.cfg.vehExitGovJumpSpeed or 6))) then
    vh.govTail = CM.poseGovTail(g, M.cfg.vehExitGovTailRate or 8)
    vh.govTail.at = vh.clock
    vh.gov, g = nil, nil
  end
  local tl = vh.govTail
  if tl ~= nil and vh.clock ~= nil then
    local s = math.max(0, vh.clock - (tl.at or vh.clock))
    if s * tl.w > 12 then
      vh.govTail = nil
    else
      local dx, dy, dz, dyw, dp = CM.poseGovTailAt(tl, s)
      local o = vh.override
      o.x, o.y, o.z, o.yaw, o.pitch = o.x + dx, o.y + dy, o.z + dz, CM.wrapDeg(o.yaw + dyw), o.pitch + dp
    end
  end
  if g ~= nil and vh.clock ~= nil then
    local req = vh.override
    local dt = math.max(0, vh.clock - (g.clock or vh.clock))
    local o = CM.poseGovStep(g, req, dt, M.cfg, g.ref or 0)
    if dt > 1e-4 then g.clock = vh.clock end
    g.maxGap = math.max(g.maxGap or 0, g.gap or 0)
    vh.govReq = req
    vh.override = { x = o.x, y = o.y, z = o.z, yaw = o.yaw, pitch = o.pitch, fov = o.fov, keepYaw = req.keepYaw }
  end
  vh.release = nil
  vh.overrides = vh.overrides + 1
  if first then
    M.Log(string.format("vehicle pose override on (%s): %s%s%s", tostring(why or "utpp_vehiclecam"), poseStr(vh.override),
      vh.override.fov and string.format(" fov=%.1f", vh.override.fov) or "",
      vh.override.keepYaw and " - orbit keeps its world yaw (mount)" or ""))
  end
  vhOp("writeNow")
  return true
end
-- exit take-over from the game's vehicle camera when the camera is NOT held (ride handed to
-- the game: passive ride, vehicle combat, vehicle first person). Allowed while mounted and the camera is in its first
-- person state (or already held). The camera becomes third person at once (w = 1: no enter blend from the eyes): hold on,
-- override = the given world pose (optional fov), the pose written, Activate(blendS) + high priority; the full body is
-- requested on the next frame (pending rep_on). utpp_perspective's following "TPP <- user" is a no-op while held.
-- blendS: the caller's (0.25 s from a readable vehicle TPP camera, 0 = one clean cut from vehicle first person).
function M.TakeForVehicleExit(pos, yaw, pitch, blendS, why, fov)
  local vh = M.vh
  why = tostring(why or "utpp_vehiclecam")
  local function refuse(reason)
    vh.refusals = vh.refusals + 1
    M.Log(string.format("vehicle exit take-over refused (%s): %s", why, reason))
    return false, reason
  end
  if not M.Enabled() then return refuse("UTPP camera off") end
  if type(pos) ~= "table" then return refuse("no pose") end
  local T = M.T
  if not (T.state == "fpp" or vh.held) then return refuse("camera state " .. tostring(T.state) .. " (not first person, not held)") end
  local okM, errM = vhOp("mounted")
  if not okM then return refuse("not mounted (" .. tostring(errM) .. ")") end
  local b = math.max(0, tonumber(blendS) or 0)
  local wasHeld = vh.held
  if not wasHeld then
    -- lastWant false: the camera comes from the first person state (the game's FPP decision), so the coming "TPP <- user"
    -- is logged as ignored
    vh.held, vh.why, vh.since, vh.lastWant = true, why, vh.clock, false
    vh.onFoot = false
    vh.frameOff, vh.frameLogged, vh.composeAt, vh.frameVel = 0, false, nil, nil
    vh.holds = vh.holds + 1
  end
  vh.handed, vh.release, vh.pending = false, nil, nil
  vh.override = { x = tonumber(pos.x) or 0, y = tonumber(pos.y) or 0, z = tonumber(pos.z) or 0, yaw = tonumber(yaw) or 0,
                  pitch = tonumber(pitch) or 0, fov = vehFov(fov) }
  vh.overrides = vh.overrides + 1
  local fromFpp = T.state == "fpp"
  T.state, T.w, T.want, T.last, T.timer = "tpp", 1, true, "vehicle exit take-over", 0
  T.epoch = T.epoch + 1
  if fromFpp then
    T.enters = T.enters + 1
    vh.pending = { "rep_on" }
  end
  local okU, errU = vhOp("useCam")
  local okW, errW = vhOp("writeNow")
  local okA, errA = vhOp("activate", b)
  local okP, errP = vhOp("prio", true)
  if okP then vh.prio = true end
  vh.takes = vh.takes + 1
  vh.exitTakes = (vh.exitTakes or 0) + 1
  M.Log(string.format("vehicle exit take-over (%s): %s%s, camera %s, pose %s %s, Activate(%.2f) %s, SetIsHighPriority(true) %s"
    .. " - third person at once (no enter blend from the eyes), epoch=%d", why, fromFpp and "from the game camera" or "while held",
    wasHeld and "" or ", hold ON", okU and tostring(errU or "ok") or ("FAILED " .. tostring(errU)), poseStr(vh.override),
    okStr(okW, errW), b, okStr(okA, errA), okStr(okP, errP), T.epoch))
  if not (okU and okA) then
    -- review fix: a failed take-over never keeps the hold (the caller treats false as a refusal and stops tracking it):
    -- the view stays with the game's vehicle camera, first person state (rep_on of the take-over replaced, body released)
    vh.takeFails = (vh.takeFails or 0) + 1
    local reason = not okU and ("no camera: " .. tostring(errU)) or ("Activate: " .. tostring(errA))
    M.HoldForVehicle(false, "exit take-over failed (" .. why .. ")", "game", 0)
    M.Log(string.format("vehicle exit take-over failed (%s): %s - hold released, view left to the game camera", why, reason))
    return false, reason
  end
  M.VehicleExitGovArm("exit take-over")
  return okA, errA
end
-- frame stall detector: the time a frame counts for the blends is min(dt, max(base, 2 x the running mean frame time)): a spike of 83 - 120 ms is cut
-- to about two normal frames, a steady slow game (20 fps: every frame over the base) is not slowed down. h.dtEma is the mean of the counted times (seeded
-- with the first one). base <= 0: off (dt as is).
local function blendDt(h, dt, base)
  if not (base and base > 0) or dt <= 0 then return dt end
  local ema = h.dtEma
  local cap = math.max(base, 2 * (ema or 0))
  local eff = dt > cap and cap or dt
  h.dtEma = ema and (ema + (eff - ema) * 0.1) or eff
  return eff
end
-- jump-out landing: utpp_vehiclecam classifies an exit as a jump-out (the vehicle moves faster than its exitFollowSpeed when the exit starts)
-- and arms the capsule pivot's jump-out mode here (vh.exitHold -> CM.solve input jumpOut, CM.jumpOutStep). Optional API (older callers never call it).
-- Only while a vehicle hold is on; a normal exit never arms it, its follow is the unchanged pivot spring. The flag ends with the next hold on / off.
-- the exit take arms the exit pose governor (cfg.vehExitGov, CM.poseGovStep) from the override just written (the shown pose: no
-- step); only for an exit reason (a ride retake / mount hold keeps its own poses). Ends with the release, the hold on / off or a cleared override.
function M.VehicleExitGovArm(why)
  local vh = M.vh
  vh.gov, vh.govTail = nil, nil
  if M.cfg.vehExitGov == false or vh.override == nil or not vh.held then return false end
  if not string.find(string.lower(tostring(why or "")), "exit", 1, true) then return false end
  vh.gov = CM.poseGovNew(vh.override)
  vh.gov.clock = vh.clock
  local fr = vh.frame -- (V's position of this frame: the first governed frame has V's speed already)
  if fr and fr.ppos then vh.gov.pp = CM.v3(fr.ppos.x, fr.ppos.y, fr.ppos.z) end
  vh.govReq = nil
  return true
end
function M.VehicleJumpOut(on, why)
  local vh = M.vh
  why = tostring(why or "utpp_vehiclecam")
  if not vh.held then return false, "not held" end
  vh.exitHold = on == true
  M.Log(string.format("vehicle jump-out landing mode %s (%s)", vh.exitHold and "armed" or "off", why))
  return true
end
-- blend from the last shown pose to the live orbit over blendS, then the hold ends by itself (high priority off).
-- carry = true (jump-out follow): the position offset from the orbit is carried with it and shrinks (the camera
-- keeps moving with V instead of starting from a pose frozen in the world)
function M.ReleaseToOrbit(blendS, why, carry)
  local vh = M.vh
  why = tostring(why or "utpp_vehiclecam")
  if not vh.held then return false, "not held" end
  local b = math.max(0, tonumber(blendS) or 0)
  vh.override = nil
  vh.landWait = nil
  local govMoving = (vh.gov ~= nil and (vh.gov.speed or 0) > 0.05) or vh.govTail ~= nil -- the exit governor still moves the shown pose: its velocity is carried
  local govArmed = vh.gov ~= nil or vh.govTail ~= nil -- this hold is a governed exit (VehicleExitGovArm)
  vh.gov, vh.govTail = nil, nil
  if vh.last == nil or b <= 0 then
    M.Log(string.format("vehicle release to orbit (%s): %s - hold off now", why, vh.last and "cut" or "no shown pose"))
    return M.HoldForVehicle(false, "release to orbit (" .. why .. ")")
  end
  local f = vh.last
  -- the release target is the orbit around V at the shown yaw (frozen: no arc to
  -- wherever V faces after the exit animation) and the shown pitch clamped to [vehReleasePitchMin, vehReleasePitchMax]
  -- (the live orbit pitch followed the game's exit look pitch, about -60: a top-down view at every release end)
  local cfg = M.cfg
  -- jump-out release: the shown pitch of a jump-out hold includes the launch framing swing (vh.frameOff, up to 14 deg while V
  -- rises faster than the camera); taking it as the orbit target kept the orbit that much higher for the rest of the game (target -4.6
  -- against -14.3 before the exit). The target is the pitch the hold was written with (the unframed override), the
  -- one a normal exit ends on; the band still acts on the target pose below. vehJumpReleasePitch = false: the shown pitch as before.
  local pitchSrc = f.pitch or 0
  if carry == true and vh.exitHold == true and cfg.vehJumpReleasePitch ~= false and vh.basePitch ~= nil then pitchSrc = vh.basePitch end
  local pitchT = CM.clamp(pitchSrc, cfg.vehReleasePitchMin or -25, cfg.vehReleasePitchMax or 5)
  -- release speed (cfg.vehReleaseSpeed): the smoothstep's peak speed 1.5 x distance / time stays at vehReleaseSpeed m/s (longer blend, at most
  -- vehReleaseTimeMax s); the distance is the one to the release target of this frame (the orbit at the frozen yaw and the clamped pitch)
  local b0 = b
  -- only after a governed exit hold (a mount hand-over, ride retake or plain API release keeps the caller's time); a target farther than
  -- vehReleaseDistMax m is no continuity case (a far / moved pose) and keeps it too
  if (cfg.vehReleaseSpeed or 0) > 0 and carry ~= true and govArmed then -- (a carried jump-out / follow release keeps its time: its offset rides on the moving orbit)
    local tw = M.OrbitPoseAt(f.yaw, pitchT)
    if tw then
      local d = math.sqrt((tw.x - f.x) ^ 2 + (tw.y - f.y) ^ 2 + (tw.z - f.z) ^ 2)
      if d <= (cfg.vehReleaseDistMax or 20) then b = math.max(b, math.min(cfg.vehReleaseTimeMax or 1.8, 1.5 * d / cfg.vehReleaseSpeed)) end
    end
  end
  vh.release = { at = vh.clock, dur = b, from = { x = f.x, y = f.y, z = f.z, yaw = f.yaw, pitch = f.pitch, fov = f.fov },
                 carry = carry == true, yawT = f.yaw, pitchT = pitchT }
  if b > b0 + 1e-3 then
    M.Log(string.format("vehicle release: blend %.2f s instead of %.2f s (peak speed %.1f m/s, cfg.vehReleaseSpeed)", b, b0, cfg.vehReleaseSpeed))
  end
  -- the blend starts with the velocity the camera has (shown pose history) instead of from rest: the offset to the live orbit
  -- decays on a cubic Hermite curve whose start slope is (shown velocity - orbit velocity); pitch the same (vehReleaseCarryVel = false: from rest)
  -- also for any release that starts while the exit governor still moves the shown pose (normal exits too)
  if ((carry == true and vh.exitHold == true) or govMoving) and cfg.vehReleaseCarryVel ~= false then
    local h = vh.hist
    local dtv = h and h.prev and h.cur and (h.cur.clock - h.prev.clock) or 0
    if dtv > 1e-4 and dtv < 0.2 and (vh.clock or 0) - h.cur.clock < 0.2 then
      local vmax = cfg.vehReleaseVelMax or 12
      local function vel(a, b) return (b - a) / dtv end
      local sx, sy, sz = vel(h.prev.x, h.cur.x), vel(h.prev.y, h.cur.y), vel(h.prev.z, h.cur.z)
      local ox, oy, oz = 0, 0, 0
      if h.cur.ox and h.prev.ox then ox, oy, oz = vel(h.prev.ox, h.cur.ox), vel(h.prev.oy, h.cur.oy), vel(h.prev.oz, h.cur.oz) end
      local dvx, dvy, dvz = sx - ox, sy - oy, sz - oz
      local m = math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz)
      if m > vmax then dvx, dvy, dvz = dvx * vmax / m, dvy * vmax / m, dvz * vmax / m end
      local pr = CM.clamp(vel(h.prev.pitch, h.cur.pitch), -(cfg.vehReleasePitchRateMax or 60), cfg.vehReleasePitchRateMax or 60)
      vh.release.dv = { x = dvx, y = dvy, z = dvz }
      vh.release.vt, vh.release.vtDt = { x = ox, y = oy, z = oz }, dtv
      -- a pitch rate towards the target must not carry the pitch past it (the peak of the carried motion is dPitch / (e * rate))
      local D = pitchT - (f.pitch or 0)
      local rate = cfg.vehReleaseCarryRate or 6.7
      if pr * D > 0 then pr = (D < 0 and -1 or 1) * math.min(math.abs(pr), 0.7 * math.abs(D) * 2.718 * rate) end
      vh.release.dPitch = pr
      -- during the hold utpp_vehiclecam writes a fresh override pose after each camera frame (the shown pose is one step ahead of the composed
      -- one); the frame that starts the release has no such write, so the shown pose stood still for that frame (a one-frame stop at 4.6 m/s in
      -- the offline replay). The first shown pose of the release is the continuation of the shown motion by one frame.
      local lead = math.min(dtv, math.max(cfg.vehBlendDtCap or 0.04, 2 * (vh.dtEma or 0))) -- (a release right after a stalled frame is not extrapolated over the stall)
      local o = { x = h.cur.x + sx * lead, y = h.cur.y + sy * lead, z = h.cur.z + sz * lead, yaw = f.yaw,
                  pitch = h.cur.pitch - (vh.frameOff or 0), fov = f.fov } -- (the pitch is not ahead in the hold: its framing offset is the composed one)
      vh.override = o
      local okW = vhOp("writeNow")
      vh.override = nil
      if okW then f = vh.last or f; vh.release.from = { x = f.x, y = f.y, z = f.z, yaw = f.yaw, pitch = f.pitch, fov = f.fov } end
    end
  end
  vh.releases = vh.releases + 1
  M.Log(string.format("vehicle release to orbit over %.2f s from %s (%s)%s, target yaw %.1f pitch %.1f", b0, poseStr(f), why,
    carry == true and " - offset carried with the orbit (moving V)" or "", f.yaw or 0, pitchT))
  local rl = vh.release
  if rl.dv then
    M.Log(string.format("vehicle release: start velocity carried (%.1f m/s relative to the orbit, pitch rate %.0f deg/s, target pitch from %s)",
      math.sqrt(rl.dv.x ^ 2 + rl.dv.y ^ 2 + rl.dv.z ^ 2), rl.dPitch or 0, (carry == true and vh.exitHold == true and cfg.vehJumpReleasePitch ~= false and vh.basePitch ~= nil) and "the hold pitch" or "the shown pose"))
  end
  return true
end
-- view to the vehicle camera: utppCamera.Deactivate(blendS) (engine blend from the shown pose), high priority off
function M.HandToVehicle(blendS, why)
  local vh = M.vh
  if not vh.held then return false, "not held" end
  local b = math.max(0, tonumber(blendS) or 0)
  local okD, errD = vhOp("deactivate", b)
  local okP, errP = vhOp("prio", false)
  vh.prio, vh.handed = false, true
  vh.handovers = vh.handovers + 1
  M.Log(string.format("vehicle hand-over (%s): utppCamera.Deactivate(%.2f) %s, SetIsHighPriority(false) %s - the vehicle "
    .. "camera renders, TPP signal stays on", tostring(why or "utpp_vehiclecam"), b, okStr(okD, errD), okStr(okP, errP)))
  return okD, errD
end
-- view back from the vehicle camera: the override pose is written first, then Activate(blendS) + high priority
function M.TakeFromVehicle(blendS, why)
  local vh = M.vh
  if not vh.held then return false, "not held" end
  local b = math.max(0, tonumber(blendS) or 0)
  local okW, errW = vhOp("writeNow")
  local okA, errA = vhOp("activate", b)
  local okP, errP = vhOp("prio", true)
  if okP then vh.prio = true end
  vh.handed = false
  vh.takes = vh.takes + 1
  M.VehicleExitGovArm(why)
  M.Log(string.format("vehicle take-over (%s): pose %s %s, utppCamera.Activate(%.2f) %s, SetIsHighPriority(true) %s",
    tostring(why or "utpp_vehiclecam"), vh.override and poseStr(vh.override) or "orbit", okStr(okW, errW), b,
    okStr(okA, errA), okStr(okP, errP)))
  return okA, errA
end
function M.SetVehiclePriority(on, why)
  local vh = M.vh
  on = on and true or false
  if on and not vh.held then return false, "not held" end
  local ok, err = vhOp("prio", on)
  if ok then vh.prio = on end
  M.Log(string.format("vehicle priority (%s): SetIsHighPriority(%s) %s", tostring(why or "utpp_vehiclecam"), tostring(on),
    okStr(ok, err)))
  return ok, err
end
-- fn(reason) -> true when the delegate took the vehicle over (HoldForVehicle(true) done)
function M.SetVehicleDelegate(fn, name)
  M.vh.delegate = type(fn) == "function" and fn or nil
  M.vh.delegateName = name
  M.Log(string.format("vehicle delegate %s: %s", M.vh.delegate and "registered" or "cleared", tostring(name)))
  return M.vh.delegate ~= nil
end
function M.AskVehicleDelegate(reason)
  local vh = M.vh
  if vh.held or vh.delegate == nil or not CM.trActive(M.T) then return false end
  vh.delegated = vh.delegated + 1
  local ok, took = pcall(vh.delegate, reason)
  local held = ok and took == true and vh.held
  M.Log(string.format("vehicle decision (%s) in third person: asked %s -> %s", tostring(reason), tostring(vh.delegateName),
    ok and (held and "held (no FPP handoff)" or "declined - first person as before") or ("error " .. tostring(took))))
  return held
end
-- final world pose while held: override | release blend (last shown pose -> live orbit) | orbit | nil (handed over,
-- nothing to show). kind "release_end" = the blend is over (the adapter ends the hold after this write).
-- 5th value = FOV to write (nil = res.fov): the override FOV ((3)), blended back during the release.
-- vh.frame = { ppos, unmounted } of this frame (set by the game adapter while held; nil in pure tests).
-- (1)(2): after the unmount the shown pitch keeps V's pivot (player + cfg.vehFrameHeight m) inside the vertical band
-- of CM.vehFramePitch - on the override pose through a rate limited correction (cfg.vehFrameRate deg/s, no snap when it
-- starts), on the release through its target pitch. The release blends to the orbit around V at the frozen
-- shown yaw (r.yawT) and the clamped pitch (r.pitchT); release_end hands that yaw / pitch to the look state.
local function vehFrameTarget(pos, pitch, fov)
  local fr, cfg = M.vh.frame, M.cfg
  if not (fr and fr.unmounted and fr.ppos) or cfg.vehFrame == false then return pitch end
  -- jump-out hold (vh.exitHold): V rises faster than the camera does: her pivot may sit higher in the frame (vehFrameHighJump of the half
  -- vertical FOV instead of vehFrameHigh) so the camera does not tilt up by the full lag (a 13 - 14 deg swing). The lower edge is unchanged. vehJumpFrame = false: off
  local jump = M.vh.exitHold == true and cfg.vehJumpFrame ~= false
  local pv = CM.v3(fr.ppos.x, fr.ppos.y, fr.ppos.z + (cfg.vehFrameHeight or 1.4))
  if jump then
    local c2 = { vehFrameLow = cfg.vehFrameLow, vehFrameHigh = cfg.vehFrameHighJump or 0.6 }
    return CM.vehFramePitch(pitch, pos, pv, fov, c2)
  end
  return CM.vehFramePitch(pitch, pos, pv, fov, cfg)
end
-- jump-out landing: V's vertical speed from the held frames' player positions (vh.frame.ppos): vh.airAt = the last clock at
-- which V was unmounted and faster than cfg.vehLandVz m/s vertically (nil = not seen in this hold)
local function vehAirStep(vh, dt, clock)
  local fr = vh.frame
  if not (fr and fr.ppos) then vh.airZ = nil; return end
  local z = fr.ppos.z
  if vh.airZ ~= nil and dt > 1e-4 and fr.unmounted and math.abs(z - vh.airZ) / dt > (M.cfg.vehLandVz or 3) then vh.airAt = clock end
  vh.airZ = z
end
local vehComposeCore
function M.VehicleCompose(res, clock)
  local pos, yaw, pitch, kind, fov = vehComposeCore(res, clock)
  -- shown pose / orbit pivot history (two frames): the start velocity of a release (ReleaseToOrbit)
  local vh = M.vh
  if pos ~= nil then
    local h = vh.hist or {}
    vh.hist = h
    local pv = res and res.pivot
    h.prev, h.cur = h.cur, { x = pos.x, y = pos.y, z = pos.z, pitch = pitch or 0, clock = clock or 0, ox = pv and pv.x, oy = pv and pv.y, oz = pv and pv.z }
  end
  return pos, yaw, pitch, kind, fov
end
vehComposeCore = function(res, clock)
  local vh = M.vh
  local dt = vh.composeAt and math.max(0, (clock or 0) - vh.composeAt) or 0
  vh.composeAt = clock
  local dtEff = blendDt(vh, dt, M.cfg.vehBlendDtCap or 0.04) -- (the mean frame time is tracked on every held frame)
  vehAirStep(vh, dt, clock)
  vh.jpShown = 0 -- the landing framing tilt (res.joPitch) included in the pose returned this frame (trace / tests)
  if vh.override then
    local o = vh.override
    local p = CM.v3(o.x, o.y, o.z)
    local want = vehFrameTarget(p, o.pitch, o.fov or (res and res.fov)) - o.pitch
    local cap = (M.cfg.vehFrameRate or 120) * dt
    local cur = vh.frameOff or 0
    vh.basePitch = o.pitch -- the unframed pitch of the hold (the release target of a jump-out, ReleaseToOrbit)
    local ease = (vh.exitHold == true or (vh.car == true and vh.airAt ~= nil)) and (M.cfg.vehFrameAccel or 0) > 0
    if ease then
      -- jump-out launch: the correction went 0 -> 6 deg within 50 ms (vehFrameRate from a standstill) and
      -- stopped dead at the band edge, a 13 deg up / 19 deg down pitch bob in 0.35 s. During the jump-out hold (vh.exitHold), and
      -- for a car as soon as V is seen in flight (vh.airAt: the native launch arms the hold a few frames later) the angular speed ramps with
      -- vehFrameAccel (deg/s^2) from the speed of the plain clamp before it and is capped so that the acceleration still stops
      -- at the target (no dead stop). Other exits (bikes, walk-offs) keep the plain clamp bit for bit; a car exit whose V snaps in z by more than
      -- vehLandVz m/s in one frame also sets vh.airAt, which changes the output only if V's pivot then leaves the framing band.
      local a, err = M.cfg.vehFrameAccel, want - cur
      local vmax = math.min(M.cfg.vehFrameEaseRate or (M.cfg.vehFrameRate or 120), M.cfg.vehFrameRate or 120)
      local vt = CM.clamp(err >= 0 and math.sqrt(2 * a * err) or -math.sqrt(-2 * a * err), -vmax, vmax)
      local v0 = vh.frameVel or 0
      local v1 = v0 + CM.clamp(vt - v0, -a * dt, a * dt)
      local step = v1 * dt
      if (step > err and err >= 0) or (step < err and err < 0) then step, v1 = err, 0 end -- never beyond the target
      vh.frameOff, vh.frameVel = cur + step, v1
    else
      vh.frameOff = cur + CM.clamp(want - cur, -cap, cap)
      vh.frameVel = dt > 1e-4 and (vh.frameOff - cur) / dt or 0 -- the speed a later jump-out arming continues from
    end
    if math.abs(vh.frameOff) > 1e-6 and not vh.frameLogged then
      vh.frameLogged = true
      M.Log(string.format("vehicle framing: V's pivot kept in the frame after the unmount (pitch %.1f -> %.1f)", o.pitch,
        o.pitch + vh.frameOff))
    end
    return p, o.yaw, o.pitch + vh.frameOff, "override", o.fov
  end
  vh.frameOff, vh.frameVel = 0, nil
  if vh.release and res then
    local r = vh.release
    -- frame stall: the blend parameter is time, and a frame of 83 - 120 ms moved it (and with it the offset to the orbit) by 10 - 15 % in one
    -- frame, a catch-up step after the stalled frame (0.535 m / 0.602 m). Time beyond vehBlendDtCap s of one frame is not counted
    -- (r.lost); frames up to the cap - every normal frame - give exactly the previous value. vehBlendDtCap = 0: off.
    if dt > dtEff + 1e-9 and (r.lastDtAt ~= clock) then
      r.lost, r.lastDtAt = (r.lost or 0) + (dt - dtEff), clock
      if not r.lostLogged then
        r.lostLogged = true
        M.Log(string.format("vehicle release: frame of %.0f ms inside the blend - %.0f ms of it not counted (no catch-up step)", dt * 1000, (dt - dtEff) * 1000))
      end
    end
    local t = (clock - r.at - (r.lost or 0)) / math.max(r.dur, 1e-3)
    local f = r.from
    local yawT = r.yawT or f.yaw
    local pitchT = r.pitchT or f.pitch
    -- target: the orbit around V at the frozen yaw / clamped pitch (framing band after the unmount on the target pitch)
    local tw = M.OrbitPoseAt(yawT, pitchT)
    if tw then
      local pf = vehFrameTarget(CM.v3(tw.x, tw.y, tw.z), pitchT, res.fov)
      -- jump-out landing (a 9.75 m/s launch, g 15.6): the framed target pitch followed V's position of each frame
      -- without a rate limit - tilting down with the falling V, then up by 2.8 deg within two frames (88 deg/s) when she stopped at the touchdown while
      -- the camera still sank. During a jump-out release (vh.exitHold, or V seen in flight) the framing offset of the target eases like the hold's
      -- (vehFrameAccel deg/s^2 up to vehFrameEaseRate deg/s, stopping on the target); other releases keep the plain framing. vehReleaseFrameEase = false: off
      local easeRel = (vh.exitHold == true or vh.airAt ~= nil) and M.cfg.vehReleaseFrameEase ~= false and (M.cfg.vehFrameAccel or 0) > 0
      if easeRel then
        local a = M.cfg.vehFrameAccel
        local vmax = math.min(M.cfg.vehFrameEaseRate or (M.cfg.vehFrameRate or 120), M.cfg.vehFrameRate or 120)
        local cur, err = r.fOff or 0, (pf - pitchT) - (r.fOff or 0)
        local vt = CM.clamp(err >= 0 and math.sqrt(2 * a * err) or -math.sqrt(-2 * a * err), -vmax, vmax)
        local v0 = r.fVel or 0
        local v1 = v0 + CM.clamp(vt - v0, -a * dtEff, a * dtEff)
        local step = v1 * dtEff
        if (step > err and err >= 0) or (step < err and err < 0) then step, v1 = err, 0 end -- never beyond the target
        r.fOff, r.fVel = cur + step, v1
        -- the eased offset fades with the blend weight: the release ends on the unframed target pitch (the hold's pitch, as before the exit), not on
        -- whatever framing was still easing at that moment (the look state keeps the end pitch for the rest of the game)
        pf = pitchT + r.fOff * (1 - CM.smoothstep(CM.clamp(t, 0, 1)))
      end
      if math.abs(pf - pitchT) > 1e-6 then
        pitchT = pf
        tw = M.OrbitPoseAt(yawT, pitchT) or tw
      end
    end
    local target = tw and CM.v3(tw.x, tw.y, tw.z) or res.world
    -- jump-out landing framing (cfg.jumpOutFrame): the orbit's tilt of this frame (res.joPitch, in res.shownPitch of the orbit frames after
    -- the release) comes in with the blend weight - the release starts on its own pose and ends on the orbit's shown pitch; the look state gets pitchT only
    local jb = (res and res.joPitch) or 0
    vh.jpShown = jb
    if t >= 1 then
      vh.release = nil
      -- the look state continues from the released pose: own pitch / yaw, the follow limiter brings it behind V
      local L = M.S and M.S.look
      if L then
        L.pitch, L.recovering, L.ownGlide = pitchT, false, false
        L.out, L.freeYaw, L.vel, L.hold, L.hist = yawT, yawT, 0, 0, {}
      end
      -- a new mount cancelled this release (HoldForVehicle(true)): the blend has reached the orbit, the new hold goes on
      if r.remount then return target, yawT, pitchT + jb, "release" end
      -- jump-out landing: V still in flight (or landed less than vehLandSettle s ago): the hold stays on, the live orbit
      -- (the release target) is shown, and the end comes with vehLandSettle s of ground under V (VehicleCompose landWait below)
      if r.carry and M.cfg.vehLandHold ~= false and vh.airAt ~= nil and clock - vh.airAt < (M.cfg.vehLandSettle or 0.2) then
        vh.landWait = { since = clock, airAt = vh.airAt }
        M.Log(string.format("vehicle release blend over, V in flight (vertical speed > %.1f m/s %.2f s ago): hold kept until %.2f s after "
          .. "the landing (at most %.1f s)", M.cfg.vehLandVz or 3, clock - vh.airAt, M.cfg.vehLandSettle or 0.2, M.cfg.vehLandMax or 2.5))
        return target, yawT, pitchT + jb, "release" -- this frame still writes the target pose / angles; the next frames show the live orbit
      end
      return target, yawT, pitchT + jb, "release_end"
    end
    local w = CM.smoothstep(t)
    local fov = (f.fov and res.fov) and (f.fov + (res.fov - f.fov) * w) or nil
    local yaw = CM.wrapDeg(f.yaw + CM.wrapDeg(yawT - f.yaw) * w)
    local pitch = f.pitch + (pitchT - f.pitch) * w + jb * w
    vh.jpShown = jb * w
    if r.carry then
      if r.off == nil then
        r.off = CM.sub(f, target)
        -- f belongs to the frame before this one: the target is taken where it was then (one frame of its own velocity back), else the first
        -- frame of the blend shows f again and a V moving at the vehicle's speed stands still for it
        if r.vt then r.off = CM.add(r.off, CM.scale(CM.v3(r.vt.x, r.vt.y, r.vt.z), r.vtDt or 0)) end
      end
      local pos = CM.add(target, CM.scale(r.off, 1 - w))
      if r.dv then
        -- start velocity: the shown camera moves relative to the orbit with dv (camera velocity - orbit velocity) at the release start; that
        -- motion goes on and dies out (dv * s * exp(-s / tau), s = seconds into the blend, faded by 1 - w so it is gone at the blend end).
        -- The offset term above has zero slope at the start, so the camera's speed is continuous (the old blend started from rest).
        local sS = t * r.dur
        local k = sS * math.exp(-sS * (M.cfg.vehReleaseCarryRate or 6.7)) * (1 - w)
        pos = CM.add(pos, CM.scale(CM.v3(r.dv.x, r.dv.y, r.dv.z), k))
        pitch = pitch + (r.dPitch or 0) * k
      end
      return pos, yaw, pitch, "release", fov
    end
    local pos = CM.vlerp(f, target, w)
    if r.dv then
      -- (a normal exit released while the exit governor still moved the shown pose): the blend from the world pose f starts at rest, so the
      -- carried term is the shown camera's world velocity (dv + the orbit's velocity r.vt), with the same decay as the jump-out release
      local sS = t * r.dur
      local k = sS * math.exp(-sS * (M.cfg.vehReleaseCarryRate or 6.7)) * (1 - w)
      local vt = r.vt or { x = 0, y = 0, z = 0 }
      pos = CM.add(pos, CM.scale(CM.v3(r.dv.x + vt.x, r.dv.y + vt.y, r.dv.z + vt.z), k))
      pitch = pitch + (r.dPitch or 0) * k
    end
    return pos, yaw, pitch, "release", fov
  end
  if vh.handed or res == nil then return nil end
  vh.jpShown = res.joPitch or 0
  local lw = vh.landWait
  if lw ~= nil then
    local air = vh.airAt ~= nil and clock - vh.airAt < (M.cfg.vehLandSettle or 0.2)
    local capped = clock - lw.since >= (M.cfg.vehLandMax or 2.5)
    if not air or capped then
      vh.landWait = nil
      M.Log(string.format("vehicle release: hold ends %.2f s after the landing%s", vh.airAt and (clock - vh.airAt) or 0,
        capped and air and " (cap reached while V was still in flight)" or ""))
      return res.world, res.yaw, res.shownPitch or res.pitch, "release_end" -- the shown pitch (ragdoll boom bias included): no pop at the write
    end
  end
  return res.world, res.yaw, res.shownPitch or res.pitch, "orbit"
end
-- mount pivot continuity: inputs of CM.mountShiftStep (cfg.mountBlend*). The camera reads the mount itself
-- (Game.GetMountedVehicle, mounted): it updates before utpp_vehiclecam, whose hold comes one frame after the game's mount
-- (hold ON: the first camera frame after it already shows the seat slot). st = adapter state, updN =
-- update counter (consecutive frames only), state = M.T.state, ppos = player position of this frame.
-- Returns arm (a jump may start a blend: inside cfg.mountBlendWindow s from the first mounted frame, third person, the orbit
-- shown - no vehicle override / release; once per mount: the window is armed on the first mounted frame after an unmounted
-- one and never after a gap, and at most one blend start per mount), keep (a running blend continues: mounted, not on foot,
-- third person or leaving) and a diagnostic line or nil (once per mount when the window closed without a blend: largest
-- step inside it).
-- ('behind' mount): the line above only measures the PIVOT (0.04 m) and so reported "pivot unchanged" while the boom
-- came in 2 m within 0.2 s and V's face filled the frame. cam (optional, the solve of the previous frame while the orbit is shown: { x, y, z, arm,
-- fullLen } world camera and boom) adds a fourth result: one line per mount at the end of cfg.mountCamWindow s (or when the override of the pull-out
-- takes the view) with the shortest boom, the nearest camera distance to V (xy) and the largest move of the camera relative to V within
-- cfg.mountCamSpan s (a time window, not a frame: 0.22 m per 18 ms frame = 12 m/s, the same motion is 0.6 m per 50 ms line);
-- a WARNING when that move exceeds mountBlendMinStep or the camera came closer than mountCamNear m to V.
function M.MountShiftGate(st, cfg, clock, updN, state, mounted, vh, ppos, cam)
  if cfg.mountBlend == false or not (state == "tpp" or state == "leaving") then
    st.mshMounted, st.mshN, st.mshSince, st.mshPrev, st.mshLastStep = nil, nil, nil, nil, nil
    st.mcHist, st.mcMinD, st.mcRep = nil, nil, nil
    st.mcShownPrev, st.mcShownN, st.mcMaxPull, st.mcMaxPullAt, st.mcMinShownD = nil, nil, nil, nil, nil
    return false, false, nil
  end
  local contiguous = st.mshN ~= nil and st.mshN == updN - 1
  local step = 0
  if contiguous and st.mshPrev and ppos then
    local dx, dy = ppos.x - st.mshPrev.x, ppos.y - st.mshPrev.y
    step = math.sqrt(dx * dx + dy * dy)
  end
  local line, camLine = nil, nil
  local prevStep = st.mshLastStep or 0
  -- a new mount = the first mounted frame after a frame seen unmounted (or after a reset); a gap in the updates (menu, photo
  -- mode, camera inactive) while the last seen frame was mounted continues the same mount and never re-arms the window
  if mounted and st.mshMounted ~= true then
    st.mshSince, st.mshMax, st.mshMaxAt, st.mshStarted, st.mshReported = clock, 0, 0, false, false
    st.mshFirstStep = step -- step on the first mounted frame
    st.mshPreStep = contiguous and prevStep or 0 -- step one frame before the mount read (a snap before it is not blended)
    st.mcHist, st.mcMinD, st.mcMinAt, st.mcMinArm, st.mcFull, st.mcMaxStep, st.mcMaxAt, st.mcRep = nil, nil, 0, nil, 0, 0, 0, false
    st.mcShownPrev, st.mcShownN, st.mcMaxPull, st.mcMaxPullAt, st.mcMinShownD = false, 0, 0, 0, nil -- samples of the pull-out override
  elseif not mounted then
    st.mshSince = nil
  end
  -- camera watch: the orbit camera relative to V over the first cfg.mountCamWindow s of the mount
  if st.mshSince ~= nil and st.mcRep == false then
    local age, cwin = clock - st.mshSince, cfg.mountCamWindow or 1.5
    local watching = cam ~= nil and ppos ~= nil and age <= cwin
    if watching and cam.skip then
      st.mcHist = nil -- the previous solve is not the previous update (a gap, a first-person frame): no sample, and no step across the gap; the window goes on
    elseif watching then
      -- the camera of the PREVIOUS solve is measured against V's position of THAT solve (cam.px/py/pz, cam.t): V's own snap onto the seat or a stale solve is
      -- no camera motion (the old pairing was the previous camera against this frame's V, a false WARNING on every seat snap)
      local cpx, cpy, cpz, ts = ppos.x, ppos.y, ppos.z or 0, clock
      if cam.px ~= nil then cpx, cpy, cpz, ts = cam.px, cam.py, cam.pz or 0, cam.t or clock end
      local rx, ry, rz = cam.x - cpx, cam.y - cpy, cam.z - cpz
      local d = math.sqrt(rx * rx + ry * ry)
      local shown = cam.shown == true -- the pose shown by the pull-out override (no boom: arm / fullLen nil), measured against this frame's V
      if st.mcMinD == nil or d < st.mcMinD then st.mcMinD, st.mcMinAt = d, age end
      if shown then
        st.mcShownN = (st.mcShownN or 0) + 1
        if st.mcMinShownD == nil or d < st.mcMinShownD then st.mcMinShownD = d end
      end
      if cam.arm and (st.mcMinArm == nil or cam.arm < st.mcMinArm) then st.mcMinArm = cam.arm end
      st.mcFull = math.max(st.mcFull or 0, cam.fullLen or cam.arm or 0)
      -- the move over the last cfg.mountCamSpan s: against the oldest kept sample (samples older than the span are dropped, the newest one beyond it is the base)
      local span = cfg.mountCamSpan or 0.1
      local h = st.mcHist
      if h == nil or not contiguous then h = {}; st.mcHist = h end
      h[#h + 1] = { t = ts, x = rx, y = ry, z = rz }
      while #h > 2 and h[2].t <= ts - span do table.remove(h, 1) end
      local b = h[1]
      if #h >= 2 and ts - b.t >= 0.5 * span then
        local sx, sy, sz = rx - b.x, ry - b.y, rz - b.z
        local sp = math.sqrt(sx * sx + sy * sy + sz * sz) * math.min(1, span / math.max(ts - b.t, 1e-6)) -- (a base older than the span is scaled to it)
        if shown and st.mcShownPrev == true then
          -- inside the pull-out the designed motion is fast (peak about 7 m/s); the step has its own limit (mountPullStep). The first shown sample (the
          -- hand-over from the orbit to the override) counts with the orbit steps: a jump there is a discontinuity, not the path.
          if sp > (st.mcMaxPull or 0) then st.mcMaxPull, st.mcMaxPullAt = sp, age end
        elseif sp > st.mcMaxStep then st.mcMaxStep, st.mcMaxAt = sp, age end
      end
      st.mcShownPrev = shown
    else
      st.mcHist = nil
    end
    if (not watching) and st.mcMinD ~= nil then -- window over, or the pull-out override took the view
      st.mcRep = true
      local warn = st.mcMaxStep > (cfg.mountBlendMinStep or 0.3) or st.mcMinD < (cfg.mountCamNear or 1.0) or (st.mcMaxPull or 0) > (cfg.mountPullStep or 1.0)
      local pullTxt = ""
      if (st.mcShownN or 0) > 0 then
        pullTxt = string.format("; pull-out override %d samples: nearest to V %.2f m, largest camera move relative to V %.2f m per %.2f s (at +%.2f s, limit %.2f m)",
          st.mcShownN, st.mcMinShownD or 0, st.mcMaxPull or 0, cfg.mountCamSpan or 0.1, st.mcMaxPullAt or 0, cfg.mountPullStep or 1.0)
      end
      camLine = string.format("mount camera%s: orbit over %.2f s after the mount - shortest boom %.2f m of %.2f m, nearest to V %.2f m (at +%.2f s), "
        .. "largest camera move relative to V %.2f m per %.2f s (at +%.2f s)%s%s", warn and " WARNING" or "", math.min(age, cwin), st.mcMinArm or 0,
        st.mcFull or 0, st.mcMinD, st.mcMinAt or 0, st.mcMaxStep or 0, cfg.mountCamSpan or 0.1, st.mcMaxAt or 0, pullTxt,
        warn and string.format(" - step limit %.2f m, V clearance %.2f m", cfg.mountBlendMinStep or 0.3, cfg.mountCamNear or 1.0) or "")
    end
  end
  local win = cfg.mountBlendWindow or 0.5
  if st.mshSince ~= nil then
    local age = clock - st.mshSince
    if age <= win and step > (st.mshMax or 0) then st.mshMax, st.mshMaxAt = step, age end
    if age > win and not st.mshReported then
      st.mshReported = true
      if not st.mshStarted then
        line = string.format("mount pivot: no jump > %.2f m in the first %.2f s after the mount (largest step %.2f m at "
          .. "+%.2f s, step on the first mounted frame %.2f m, one frame before the mount read %.2f m) - pivot unchanged",
          cfg.mountBlendMinStep or 0.3, win, st.mshMax or 0, st.mshMaxAt or 0, st.mshFirstStep or 0, st.mshPreStep or 0)
      end
    end
  end
  st.mshLastStep = step
  st.mshMounted, st.mshN = mounted, updN
  st.mshPrev = ppos and { x = ppos.x, y = ppos.y } or nil
  local keep = mounted == true and not (vh and vh.onFoot)
  -- at most one blend start per mount (st.mshStarted is set by the caller when the solver reports the start)
  local arm = keep and contiguous and state == "tpp" and st.mshSince ~= nil and (clock - st.mshSince) <= win
    and not st.mshStarted and not (vh and vh.held and (vh.override ~= nil or (vh.release ~= nil and not vh.release.remount)))
    -- (a release that a new mount keeps blending out, rl.remount, counts as no release: the mount blend may start, the target follows the pivot)
  return arm, keep, line, camLine
end
-- ragdoll camera: the inputs of CM.solve's `ragdoll` (see CM.ragdollPhase) from the DLL's natives, each guarded by pcall:
-- Game.UTPPRagdollState() (0 idle / 1 ragdoll owns the body / 2 blend-out) and, while a state is on or the episode runs,
-- Game.UTPPRagdollPelvis() (Hips in V's model space, w = 1 when fresh). det = PSM LocomotionDetailed (Knockdown 29 / Felled 31 keep a
-- running episode, they never start one). A DLL without the natives (or cfg.ragdollCam = false) gives nil = the old camera; a missing
-- native is retried every 5 s (not every frame: a failing pcall builds an error string).
local RAG_KNOCK_DETAILED = { [29] = true, [31] = true }
function M.RagdollRead(G, det, vhHeld)
  if M.cfg.ragdollCam == false then return nil end
  if G.ragNaAt ~= nil and G.clock < G.ragNaAt then return nil end
  local okS, st = pcall(function() return Game.UTPPRagdollState() end)
  if not okS or type(st) ~= "number" then
    G.ragNaAt = G.clock + 5
    M.LogOnce("ragNa", "ragdoll camera: Game.UTPPRagdollState unavailable (" .. tostring(st) .. ") - DLL without the ragdoll natives, "
      .. "the normal camera is kept")
    return nil
  end
  G.ragNaAt = nil
  st = math.floor(st)
  -- (a car hit at a jump-out landing): a UTPP ragdoll episode whose vanilla knockdown effect never arrived (aborted
  -- after 0.44 s, 'knockdown effect missing') still turned the camera on at its begin (native state 1) and dipped the view. Game.UTPPRagdollConfirmed()
  -- is true while an episode runs / blends out AND its knockdown effect was seen; a camera that is not on yet waits for it (the input is idle meanwhile:
  -- state 0, no pelvis). Once on, the episode is not gated again (state 2, the vanilla get-up). A missing native or a non-boolean answer = the old behaviour.
  local pending = false
  if st >= 1 and G.ragEp ~= true and not (G.ragConfNaAt ~= nil and G.clock < G.ragConfNaAt) then
    local okC, conf = pcall(function() return Game.UTPPRagdollConfirmed() end)
    if okC and type(conf) == "boolean" then
      G.ragConfNaAt = nil
      if not conf then pending = true end
    else
      G.ragConfNaAt = G.clock + 5 -- native missing (older DLL / reds): the old behaviour, retried every 5 s (a failing pcall builds an error string)
    end
  end
  G.ragPending = pending or nil
  if pending then st = 0 end
  local r = { state = st, knock = G.ragEp == true and RAG_KNOCK_DETAILED[det] == true, noLook = vhHeld == true }
  -- the vanilla car knockdown keeps PSM detail Stand: utpp_ragdoll.reds UTPPRagdollBusy() covers the get-up (knockdown effect)
  if G.ragEp == true and not r.knock then
    local okB, busy = pcall(function() return Game.UTPPRagdollBusy() end)
    r.knock = okB and busy == true
  end
  -- early return: the native get-up (Game.UTPPRagdollGetUpState: 0 idle, 1 blend, 2 play, 3 hand-off, -1 aborted) while the episode runs;
  -- frac = clip time / clip length from the 't=<now>/<len>' of Game.UTPPRagdollGetUpStatus (read in state 2 only, until the return has begun). A missing
  -- native (or cfg.ragReturnEarly = false) = no input = the old return after the episode end; retried every 5 s like the other natives.
  if G.ragEp == true and M.cfg.ragReturnEarly ~= false and not (G.ragGuNaAt ~= nil and G.clock < G.ragGuNaAt) then
    local okG, gs = pcall(function() return Game.UTPPRagdollGetUpState() end)
    if okG and type(gs) == "number" then
      G.ragGuNaAt = nil
      gs = math.floor(gs)
      local g = { state = gs }
      if gs == 2 and not (M.S ~= nil and M.S.rag ~= nil and M.S.rag.ret ~= nil) then
        local okT, status = pcall(function() return Game.UTPPRagdollGetUpStatus() end)
        if okT and type(status) == "string" then
          local a, b = status:match("%st=([%d%.%-]+)/([%d%.%-]+)")
          a, b = tonumber(a), tonumber(b)
          if a ~= nil and b ~= nil and b > 0 then g.frac = a / b end
        end
      end
      r.getup = g
    else
      G.ragGuNaAt = G.clock + 5
    end
  end
  if st ~= 0 or G.ragEp == true then
    local okP, pv = pcall(function() return Game.UTPPRagdollPelvis() end)
    if okP and pv ~= nil then
      pcall(function()
        r.pelvis = CM.v3(tonumber(pv.x) or 0, tonumber(pv.y) or 0, tonumber(pv.z) or 0)
        r.fresh = (tonumber(pv.w) or 0) > 0.5
      end)
    end
  end
  return r
end
-- the orbit keeps its world yaw while held (no follow of the body): before
-- any override / release, and under an override marked keepYaw (utpp_vehiclecam mount pull-out)
function M.VehicleHoldYaw(vh)
  vh = vh or M.vh
  return vh.held == true and (vh.release == nil or vh.release.remount == true) and (vh.override == nil or vh.override.keepYaw == true)
end
-- utpp_vehiclecam: the near pull-out trigger waits while this is true (one motion after the other)
function M.MountBlendActive() return M.S ~= nil and M.S.mshift ~= nil end
function M.SetMountBlend(on)
  M.cfg.mountBlend = on and true or false
  M.Log("mount pivot blend " .. (M.cfg.mountBlend and "ON" or "OFF") .. " (A/B, not persisted)")
  return M.cfg.mountBlend
end
-- last orbit (solver) pose in the world while the camera is in third person
function M.OrbitPose()
  local r = M.G and M.G.last
  if not r or not CM.trActive(M.T) then return nil end
  return { x = r.world.x, y = r.world.y, z = r.world.z, yaw = r.yaw, pitch = r.shownPitch or r.pitch, fov = r.fov }
end
-- orbit pose around V (pivot of the last solve, current profile / zoom / shoulder, collision ratio of the last
-- solve) for the given world yaw / pitch - used by utpp_vehiclecam to follow V after a jump-out with the pitch kept
function M.OrbitPoseAt(yaw, pitch)
  local r = M.G and M.G.last
  if not r or not r.pivot or not CM.trActive(M.T) then return nil end
  yaw, pitch = tonumber(yaw) or r.yaw, tonumber(pitch) or r.pitch
  local p = CM.blendCurrent(M.blend)
  local dist = CM.clamp(p.distance + (M.rt.zoom or 0), M.cfg.distMin, M.cfg.distMax)
  local off = CM.boomOffset(yaw, pitch, p.side * (M.shoulder.s or 1), dist, p.height)
  local ratio = 1
  if r.fullLen and r.fullLen > 1e-3 and r.arm then ratio = CM.clamp(r.arm / r.fullLen, 0, 1) end
  local w = CM.add(r.pivot, CM.scale(off, ratio))
  return { x = w.x, y = w.y, z = w.z, yaw = yaw, pitch = pitch }
end
-- return from aiming: shorter entry blend for this transition and the orbit yaw behind V's back. bodyYaw nil =
-- applied by the game adapter on its next frame (it knows the body yaw).
function M.AimReturn(why, bodyYaw)
  if not M.rt.aimReturn then return false end
  if bodyYaw == nil then M.aimReturnPending = why or "aim return"; return true end
  local T = M.T
  T.enterOverride = math.max(0, tonumber(M.rt.aimReturnBlend) or 0.35)
  CM.lookReset(M.S.look, bodyYaw, (T.w or 0) > 0)
  M.stats.aimReturns = M.stats.aimReturns + 1
  M.Log(string.format("aim return (%s): third person over %.2f s (camera.aimReturnBlend, normal %.2f s), orbit yaw reset "
    .. "behind the back (body yaw %.1f, state %s w=%.2f)", tostring(why), T.enterOverride, M.cfg.enterTime, bodyYaw,
    tostring(T.state), T.w or 0))
  return true
end
-- aim: perspective.aiming from utpp_core (registered by utpp_perspective), else utpp_perspective's own cfg
function M.AimingSetting()
  local v = nil
  if M.core and M.core.Get then v = M.core.Get("perspective.aiming") end
  local p = M.persp
  if v == nil and type(p) == "table" and type(p.cfg) == "table" then v = p.cfg.aiming end
  return v
end
-- one aim frame (no game API): o = CM.aimWanted input + weapon (record for the log), fov (weapon AimFOV | nil).
-- Returns the event of this frame ("enter" | "exit" | nil).
function M.AimUpdate(o, dt)
  local A = M.aim
  local want, reason = CM.aimWanted(o)
  local ev = CM.aimStep(A, M.cfg, dt, want, reason)
  if A.on then M.aimFov = (o and o.fov) or M.cfg.aimFov end -- live: weapon change / settings slider while aiming
  if ev == "enter" then
    M.aimWeapon = o and o.weapon
    -- the aim is refused while held (CM.aimWanted "vehicle hold"); the first aim within
    -- cfg.vehAimSoftWindow s after the hold ended blends in over >= cfg.vehAimBlendIn s (the release end, the body align
    -- and the aim pose landed on separate frames: several one-frame steps)
    local vh = M.vh
    local soft = ""
    if vh.aimSoftAt ~= nil and not vh.held and (vh.clock or 0) - vh.aimSoftAt <= (M.cfg.vehAimSoftWindow or 1.0) + 1e-6 then
      local d = math.max(A.dur or 0, M.cfg.vehAimBlendIn or 0.20)
      if d > (A.dur or 0) then
        A.dur = d
        if A.t >= 1 then A.t = 0 end
        A.w = A.from + (A.target - A.from) * CM.smoothstep(A.t)
      end
      vh.aimSoftAt = nil -- only the first aim after the hold
      soft = string.format(" (first aim after a vehicle hold: blend in %.2f s)", d)
    end
    M.Log(string.format("cam aim enter weapon=%s fov=%.0f side=%s%s", tostring(M.aimWeapon or "?"), M.aimFov,
      (M.shoulder.target or 1) >= 0 and "R" or "L", soft))
    M.BodyNudge("aim enter")
  elseif ev == "exit" then
    M.Log(string.format("cam aim exit (%s) after %.2f s", tostring(reason), A.since - (A.prevSince or A.since)))
    M.BodyNudge("aim exit")
  end
  return ev
end
-- (male V test: shirt + right arm gone at every aim enter / exit while the body request had been given up):
-- utpp_body Nudge(why) - the body parts are verified at once after the game re-resolved the appearance and a
-- given-up TPP body request is re-armed. Only while utpp_body owns the body for this camera; false otherwise.
function M.BodyNudge(why)
  local G = M.G
  local B = M.bodyMod
  if not G or G.repOwned ~= "utpp_body" or type(B) ~= "table" or type(B.Nudge) ~= "function" then return false end
  local ok, r = pcall(B.Nudge, why)
  if ok and r == true then M.stats.bodyNudges = (M.stats.bodyNudges or 0) + 1 end
  return ok and r == true
end
-- session reset / shutdown: aim off at once (logged when it was on)
function M.AimReset(why)
  local A = M.aim
  if A.on then M.Log(string.format("cam aim exit (%s) after %.2f s", tostring(why), CM.aimTime(A))) end
  M.aim = CM.newAim()
end
-- ================================================================ unarmed zoom (RMB with empty hands)
-- one wheel step (dir + = closer / ZoomIn) while the state is on; false = not consumed (the state is off)
function M.ZoomWheel(dir, src)
  dir = tonumber(dir) or 0
  if not M.zoomU.on or dir == 0 then return false end
  M.zuIn.wheel = M.zuIn.wheel + (dir > 0 and 1 or -1)
  M.zuIn.src = src or "console"
  return true
end
-- first person handoff request: utpp_perspective.RequestFppZoom(on, why) when it decides (active / off mode), else the own
-- block rule "unarmed zoom" (no utpp_perspective). Returns ok, route | error.
function M.RequestFppHandoff(on, why)
  if on then M.PoseWriteTpp(false) end
  on = on and true or false
  local I = M.zuIn
  if I.requested == on then return true, I.route end
  local P = M.Perspective()
  local ok, info
  if P then
    if type(P.RequestFppZoom) == "function" then
      local okC, r = pcall(P.RequestFppZoom, on, "utpp_camera: " .. tostring(why))
      ok = okC and r ~= false
      info = okC and "utpp_perspective.RequestFppZoom" or ("utpp_perspective.RequestFppZoom error " .. tostring(r))
    else
      ok, info = false, "utpp_perspective " .. tostring(P.VERSION) .. " has no RequestFppZoom"
    end
  else
    ok, info = true, "own rule 'unarmed zoom' (no utpp_perspective)"
  end
  if ok or not on then I.requested, I.route = on and ok, info end
  return ok, info
end
local function zuSide() return (M.shoulder.target or 1) >= 0 and "R" or "L" end
local function zuGameZoom()
  local z = M.zuIn.gameZoom
  return z and string.format(", game zoom level %.2f", z) or ""
end
-- one unarmed zoom frame (no game API): o = CM.zoomUnarmedWanted input (upperBody, weaponInHand, ... , tpp) + gameZoom
-- (PSM ZoomLevel, log only). Consumes the pending wheel steps. Returns the events of this frame.
function M.ZoomUnarmedUpdate(o, dt)
  local Z, I, cfg = M.zoomU, M.zuIn, M.cfg
  o = o or {}
  dt = math.max(tonumber(dt) or 0, 0)
  I.held = (tonumber(o.upperBody) == 6) and (I.held + dt) or 0
  I.gameZoom = o.gameZoom
  o.held, o.on, o.handoff, o.enterDelay = I.held, Z.on, Z.fpp, cfg.zoomUnarmedEnterDelay
  o.enabled = cfg.zoomUnarmedEnabled
  local want, reason = CM.zoomUnarmedWanted(o)
  local wheel, src = I.wheel, I.src
  I.wheel, I.src = 0, nil
  local evs = CM.zoomUnarmedStep(Z, cfg, dt, want, reason, wheel)
  local n = CM.zoomUnarmedSteps(cfg)
  for _, e in ipairs(evs) do
    if e.kind == "enter" then
      local p = CM.zoomUnarmedPose(cfg, 0)
      M.Log(string.format("cam zoomUnarmed enter k=0/%d d=%.2f fov=%.0f side=%s (RMB with empty hands, UpperBody=6)", n,
        p.distance, p.fov, zuSide()))
      M.BodyNudge("zoom enter")
    elseif e.kind == "step" then
      local p = CM.zoomUnarmedPose(cfg, e.k)
      M.Log(string.format("cam zoomUnarmed step %d/%d d=%.2f fov=%.0f (%s %s)%s", e.k, n, p.distance, p.fov,
        e.dir > 0 and "in" or "out", tostring(src or "wheel"), zuGameZoom()))
    elseif e.kind == "handoff" then
      local ok, info = M.RequestFppHandoff(true, string.format("unarmed zoom past step %d/%d", n - 1, n))
      if ok then
        M.Log(string.format("cam zoomUnarmed fpp handoff (step %d/%d, %s): first person, the game's zoom takes over via %s%s",
          e.k, n, tostring(src or "wheel"), tostring(info), zuGameZoom()))
      else
        CM.zoomUnarmedCancelHandoff(Z, cfg)
        M.Log(string.format("cam zoomUnarmed fpp handoff REFUSED (%s) - stays at step %d/%d", tostring(info), Z.k, n))
      end
    elseif e.kind == "exit" then
      local extra = ""
      if I.requested then
        local ok, info = M.RequestFppHandoff(false, "zoom exit: " .. tostring(e.reason))
        extra = string.format(" - handoff cleared via %s%s", tostring(info), ok and "" or " FAILED")
      end
      M.Log(string.format("cam zoomUnarmed exit (%s) after %.2f s at step %d/%d%s%s", tostring(e.reason), e.dur or 0, e.k, n,
        e.wasFpp and " (first person)" or "", extra))
      M.BodyNudge("zoom exit")
    end
  end
  -- a request without an active handoff (cancelled / refused) is cleared
  if I.requested and not Z.fpp then M.RequestFppHandoff(false, "handoff no longer active") end
  return evs
end
-- session reset / shutdown: state off at once (logged when it was on), request cleared
function M.ZoomUnarmedReset(why)
  local Z, I = M.zoomU, M.zuIn
  if Z.on then
    M.Log(string.format("cam zoomUnarmed exit (%s) after %.2f s at step %d/%d%s", tostring(why), Z.clock - Z.since, Z.k,
      CM.zoomUnarmedSteps(M.cfg), Z.fpp and " (first person)" or ""))
  end
  if I.requested then pcall(M.RequestFppHandoff, false, tostring(why)) end
  M.zoomU = CM.newZoomUnarmed()
  I.wheel, I.src, I.held, I.requested, I.gameZoom = 0, nil, 0, false, nil
end
-- plain copy of the state (console / other mods)
function M.ZoomUnarmed()
  local Z = M.zoomU
  return { on = Z.on, k = Z.k, steps = CM.zoomUnarmedSteps(M.cfg), fpp = Z.fpp, w = Z.w, requested = M.zuIn.requested,
           route = M.zuIn.route, enters = Z.enters, exits = Z.exits, handoffs = Z.handoffs, gameZoom = M.zuIn.gameZoom }
end
-- aim occlusion (level A): the muzzle is blocked while the shot (from the active camera) would pass. Lua-side flag
-- for other mods / a future redscript bridge: GetMod("utpp_camera").AimBlocked()
function M.AimBlocked() return M.occ.blocked == true end
function M.AimBlockInfo()
  local st = M.occ
  local i = st.info or {}
  return { blocked = st.blocked == true, dMuzzleHit = i.dMuzzleHit, dMuzzleP = i.dMuzzleP, t = i.t, t0 = i.t0,
           p = i.p and CM.v3(i.p.x, i.p.y, i.p.z) or nil, raw = i.raw, reason = i.reason, since = st.since,
           samples = st.samples, blocks = st.blocks }
end
local function occClearLog(dur, why)
  M.Log(string.format("aim clear after %.2f s%s", dur or 0, why and (" (" .. tostring(why) .. ")") or ""))
end
-- sampling stopped: flag off at once (logged when it was on)
function M.AimOccReset(why, clock)
  local ev, dur = CM.aimOccReset(M.occ, tonumber(clock) or M.aim.clock or 0)
  if ev == "clear" then occClearLog(dur, why) end
  return ev
end
-- one occlusion frame (no game API): cam / fwd = the camera pose written this frame (the shot origin / direction),
-- muzzleFn() -> world v3 | nil, why; ray(from, to) -> hit distance from `from` | nil. Samples only while
-- camera.aim.occlusion is on and the aim weight > cfg.aimOccMinW, throttled by CM.aimOccDue. Returns the edge event.
function M.AimOccFrame(clock, cam, fwd, muzzleFn, ray)
  local cfg, st = M.cfg, M.occ
  if cfg.aimOcclusion == false then return M.AimOccReset("camera.aim.occlusion off", clock) end
  if not (M.aim.w > (cfg.aimOccMinW or 0.5)) then return M.AimOccReset("aim off", clock) end
  if cam == nil or fwd == nil or type(ray) ~= "function" then return M.AimOccReset("no camera pose / raycast", clock) end
  if not CM.aimOccDue(st, cfg, clock) then return nil end
  local muzzle, why = nil, "no muzzle reader"
  if type(muzzleFn) == "function" then
    local okM, m, w = pcall(muzzleFn)
    if okM then muzzle, why = m, w else why = tostring(m) end
  end
  local info
  if muzzle == nil then
    M.LogOnce("aimoccmuzzle", "aim occlusion: weapon muzzle position unreadable (" .. tostring(why)
      .. ") - aimBlocked stays 0 while it is missing")
    info = { raw = false, reason = "no muzzle: " .. tostring(why) }
  else
    info = CM.aimOccSample(cfg, cam, CM.norm(fwd), muzzle, ray)
  end
  local ev, dur = CM.aimOccStep(st, cfg, clock, info.raw, info)
  if ev == "blocked" then
    M.Log(string.format("aim blocked d=%.2f (muzzle->P hit at %.2f of %.2f)", info.dMuzzleHit or 0, info.dMuzzleHit or 0,
      info.dMuzzleP or 0))
  elseif ev == "clear" then
    occClearLog(dur)
  end
  return ev
end
function M.VehicleStatus()
  local vh = M.vh
  return string.format("vehicle held=%s why=%s handed=%s prio=%s override=%s release=%s last=%s holds=%d refusals=%d "
    .. "handovers=%d takes=%d releases=%d toGame=%d delegated=%d delegate=%s ignored=%d",
    tostring(vh.held), tostring(vh.why), tostring(vh.handed), tostring(vh.prio), vh.override and poseStr(vh.override) or "-",
    vh.release and string.format("%.2f s", vh.release.dur) or "-", poseStr(vh.last), vh.holds, vh.refusals, vh.handovers,
    vh.takes, vh.releases, vh.toGame, vh.delegated, tostring(vh.delegateName), vh.ignored)
    -- exit take-overs from the game camera, FPP activations skipped (vehicle TPP), API version
    .. string.format(" exitTakes=%d fppGuards=%d api=%d", vh.exitTakes or 0, M.stats.fppGuards or 0, M.VehicleApiVersion)
end

-- ================================================================ actions (also the console API)
local function refuse(what)
  if M.Enabled() then return false end
  M.Log(string.format("%s ignored: UTPP camera is off (setting 'Use Ultimate Third Person' off)", what))
  return true
end
-- utpp_body (when present and enabled) owns the full-body layers (representation ownership, classifier, garments)
function M.BodyModule()
  local b = M.bodyMod
  if type(b) ~= "table" or type(b.SetTppBody) ~= "function" then return nil end
  local ok, en = pcall(function() return b.Enabled == nil or b.Enabled() end)
  if ok and en then return b end
  return nil
end
-- utpp_perspective (when present) owns the perspective decision
function M.Perspective()
  local p = M.persp
  if type(p) ~= "table" or type(p.Desired) ~= "function" then return nil end
  return p
end
function M.Toggle(src)
  if refuse("perspective toggle") then return false end
  local P = M.Perspective()
  if P and P.ToggleUser then
    P.ToggleUser("utpp_camera " .. tostring(src or "console"))
    return true
  end
  CM.trRequest(M.T, not M.T.want, "toggle " .. tostring(src or "console"))
  M.Log(string.format("perspective -> %s (%s)", M.T.want and "TPP" or "FPP", tostring(src or "console")))
  return M.T.want
end
function M.Enter() if refuse("enter") then return false end CM.trRequest(M.T, true, "enter"); return true end
function M.Exit() CM.trRequest(M.T, false, "exit"); return true end
function M.Swap(src)
  if refuse("shoulder swap") then return false end
  local s = CM.shoulderSwap(M.shoulder)
  M.aim.swap.t = 0 -- a manual swap restarts the aim auto-swap hold
  -- optional source label (key / pad of the shoulder-swap input); the console call logs the old line
  -- (string only: GetMod("utpp_camera"):Swap() with a colon passes the module table)
  M.Log("shoulder -> " .. (s > 0 and "right" or "left") .. (type(src) == "string" and (" (" .. src .. ")") or ""))
  return s
end
-- shoulder-swap input (keyboard L, controller R3 held 0.5 s, rebindable in Mod Settings > Camera).
-- r6\input\utpp_shoulder.xml (Input Loader) adds the actions, r6\scripts\UTPP\utpp_shoulder_input.reds (UTPPShoulderInput)
-- counts swap requests; the game adapter drains Game.UTPPShoulderSwapTake() every frame, gates the requests
-- (M.ShoulderInputBlock reasons) and applies an odd count as one M.Swap(src). Gated requests are dropped, never queued: a
-- press in a menu, in first person or in a vehicle does nothing later either. The side stays session state (right at start)
-- and the automatic aim swap at walls keeps its behaviour (M.Swap restarts its hold).
-- missing: nil = not polled yet, true = reds system / function unavailable (logged once), false = found
M.shIn = { requests = 0, swaps = 0, cancelled = 0, ignored = 0, missing = nil, retryAt = nil, lastWhy = nil, logAt = nil,
           last = nil }
M.SHOULDER_IN_RETRY = 5.0   -- s between polls while the reds system is missing
M.SHOULDER_IN_LOG_GAP = 2.0 -- s: the same "ignored" reason is logged at most this often
-- reason a request is ignored, or nil. ctx: enabled, menu, photo, loading, tpp (third person shown or entering), vehicle
-- (vehicle hold / PSM vehicle / mounted), psm (side-effect-free PSM reason: workspot, swimming, scene, takedown, carrying,
-- device, scanner, braindance)
function M.ShoulderInputBlock(ctx)
  if not ctx.enabled then return "UTPP camera off" end
  if ctx.menu then return "menu" end
  if ctx.photo then return "photo mode" end
  if ctx.loading then return "loading" end
  if ctx.vehicle then return "vehicle" end
  if ctx.psm then return ctx.psm end
  if not ctx.tpp then return "first person" end
  return nil
end
-- n requests drained this frame from source src; why = M.ShoulderInputBlock(...) or nil; clock = adapter clock (s)
function M.ShoulderInputApply(n, src, why, clock)
  local si = M.shIn
  n = math.floor(tonumber(n) or 0)
  if n <= 0 then return nil end
  si.requests = si.requests + n
  si.last = src
  if why then
    si.ignored = si.ignored + n
    if why ~= si.lastWhy or si.logAt == nil or (clock or 0) - si.logAt >= M.SHOULDER_IN_LOG_GAP then
      si.lastWhy, si.logAt = why, clock or 0
      M.Log(string.format("shoulder swap ignored (%s)%s", why, src and (" [" .. tostring(src) .. "]") or ""))
    end
    return nil
  end
  if n % 2 == 0 then
    si.cancelled = si.cancelled + n
    M.Log(string.format("shoulder swap: %d requests in one frame cancel out (%s)", n, tostring(src)))
    return nil
  end
  si.swaps = si.swaps + 1
  return M.Swap(src or "input")
end
function M.ShoulderInputStatus()
  local si = M.shIn
  return string.format("shoulderIn=%s requests=%d swaps=%d cancelled=%d ignored=%d last=%s why=%s",
    si.missing == nil and "-" or (si.missing and "missing" or "ok"), si.requests, si.swaps, si.cancelled, si.ignored,
    tostring(si.last or "-"), tostring(si.lastWhy or "-"))
end
-- one fixed camera profile. The active slot is always FIXED_PROFILE (the former profile 2
-- "Medium" values); a stored camera.profile, the profile keys and console calls cannot select another slot. The five
-- stored profiles (camera.pN.*) and camera.profile / profileCount / profileBlend stay registered so old settings files
-- load and save unchanged.
M.FIXED_PROFILE = 2
function M.ProfileKey(f) return "camera." .. (f == "sens" and "framingSens" or f) end
function M.Profile(i, silent)
  local want = M.FIXED_PROFILE
  local req = math.floor(tonumber(i) or want)
  if req ~= want and not silent then M.Log(string.format("profile %d ignored: the camera uses one fixed profile (%d)", req, want)) end
  if M.rt.profileIndex ~= want then
    M.rt.profileIndex = want
    -- the framing views ("camera.edit.*") are re-read (nil-safe before the views exist / older core)
    if M.core and type(M.core.RefreshView) == "function" then pcall(M.core.RefreshView, "camera.edit.") end
    CM.blendTo(M.blend, M.profiles[want], M.rt.profileBlend)
  end
  return want
end
function M.NextProfile(dir)
  if refuse("profile change") then return false end
  return M.Profile(M.FIXED_PROFILE, true)
end
function M.QuickFpp(down)
  if refuse("quick FPP") then return false end
  local P = M.Perspective()
  if P and P.SetQuickFpp then P.SetQuickFpp(down) return true end
  CM.trQuick(M.T, down)
  return true
end
function M.FreeLook(down) M.rt.freeHeld = down and true or false; return M.rt.freeHeld end
-- utpp_bodyturn aim align (turns V onto the game's look while aiming): FPP heading lock while it turns V
function M.AlignHold(on) M.rt.alignHold = on and true or false; return M.rt.alignHold end
-- yaw (deg) of the camera shown last frame (world camera / component) - utpp_bodyturn turns V onto it while
-- aiming (camera.aimOwnYaw); nil when the camera is not shown
function M.ShownYaw()
  local G = M.G
  if not G or not G.last or type(G.last.yaw) ~= "number" or not M.T or M.T.state ~= "tpp" then return nil end
  return G.last.yaw
end
-- game look sync: true while the camera shows the game's look (heading lock on, param set sent,
-- confirmed, not broken) - utpp_bodyturn's aim align spin guard is not needed then
function M.LookSynced()
  local G = M.G
  return G ~= nil and G.lookUsed == true
end
-- true while the game's look is written from the camera this frame (game look write ON);
-- false otherwise (off, failed, tripped, unavailable, world camera not shown). utpp_bodyturn's camera coupling guard
-- skips the game-look input when it is false (the game's look then follows V's body).
function M.LookWritten()
  local G = M.G
  if G == nil then return false end
  if G.lwState == nil then return nil end -- unknown
  return G.lwState == "on"
end
-- game look write: state on | off | fail | unavailable, last read-back error (deg)
function M.LookWriteStatus()
  local G = M.G
  if not G then return "lookWrite=-" end
  -- (trips = native divergence trips seen; retry = s until the next try after a trip, - = none / for good)
  local retry = "-"
  if G.lwTripped and not G.lwTripPermanent and G.lwRetryAt and G.clock then
    retry = string.format("%.1f", math.max(0, G.lwRetryAt - G.clock))
  end
  return string.format("lookWrite=%s state=%s sets=%d fails=%d lwerr=%s lwperr=%s aheadSends=%d aheadFails=%d trips=%d "
    .. "retry=%s adjFrames=%d relocks=%d skips=%d",
    tostring(M.cfg.lookWrite ~= false), G.lwUnavailable and "unavailable" or (G.lwTripped and "tripped"
    or tostring(G.lwState or "off")), G.lwSets or 0,
    G.lwFails or 0, G.lwErr and string.format("%.2f", G.lwErr) or "-", G.lwPErr and string.format("%.2f", G.lwPErr) or "-",
    G.lwAheadSends or 0, G.lwAheadFails or 0, G.lwTrips or 0, retry, G.adjFrames or 0, M.stats.lockRelocks or 0,
    M.stats.lockSkips or 0)
end
function M.LookSyncStatus()
  local G = M.G
  local ls = G and G.ls
  if not ls then return "lookSync=-" end
  return string.format("lookSync=%s set=%s sent=%s route=%s ok=%s broken=%s used=%s confirm=%d maintained=%d carried=%d "
    .. "resends=%d", tostring(M.cfg.lookSync ~= false), tostring(M.cfg.lookSyncSet), tostring(ls.sent == true),
    tostring(ls.route), tostring(ls.ok), tostring(ls.broken), tostring(G.lookUsed == true), ls.confirm, ls.maintained,
    ls.carriedTotal, ls.resends)
end
-- (image shimmer on fast direction changes, also without DLSS): utpp_bodyturn runs AFTER the camera in the
-- CET frame (measured, log "predict: ... after"), so the step it requests for the coming tick was only predicted. It
-- calls this right after its request: the local transform of this frame is rewritten with the exact step (the world
-- camera pose stays the one solved this frame). Only when the order was measured as "after" and the camera wrote an
-- orbit pose in free mode this frame. Returns true when rewritten.
function M.OnBodyStep(step)
  local G = M.G
  if not G or G.btBefore ~= false or M.cfg.predictBodyTurn == false then return false end
  if G.use == "world" then return false end -- the world camera does not turn with V (nothing to cancel)
  local w = G.lastWrite
  -- exact step: no prediction cap (at 30 fps V turns 17-19 deg per frame; the old 15 deg cap left those uncompensated)
  if not w or w.clock ~= G.clock or type(step) ~= "number" or math.abs(step) > 180 then return false end
  local ok = pcall(function()
    local lp, lq = CM.toLocal(w.ppos, CM.qmul(CM.qFromYawPitch(step, 0), w.q), w.wpos, w.wq)
    w.comp:SetLocalTransform(Vector4.new(lp.x, lp.y, lp.z, 1), Quaternion.new(lq.i, lq.j, lq.k, lq.r))
  end)
  if ok then
    G.exactSteps = (G.exactSteps or 0) + 1
    G.pred, G.predExact = step, true
  end
  return ok
end
-- game look write + body turn: utpp_bodyturn calls this right after a successful request (Locomotion
-- apply / aim align applyAim) with the exact yaw step V will turn in the coming game tick. The request lands after the
-- FPP camera update of that tick, so the native must know it in advance (UTPPSetGameLook entityYawAheadDeg) - without it
-- the look turned with V for a tick and was pulled back (up to 18 deg per frame at 60 fps with the 1080 deg/s aim align).
-- utpp_bodyturn runs after the camera in the CET frame (measured; G.btBefore false / nil): the target written this
-- frame is sent again with the step. When it was measured BEFORE the camera (G.btBefore true) the step is kept for the
-- camera's write later in the same frame (lookWriteFrame). Returns true when sent / kept.
-- (review): the order is only measured on utppCamera (G.btBefore stays nil on the world camera). If utpp_bodyturn
-- runs first there, the last frame's target goes out with the step and the camera's own write follows with ahead 0 -
-- keeps the pending step then (a 0 never clears it; steps add up until the next tick uses them).
function M.OnBodyLookStep(step)
  local G = M.G
  if not G or type(step) ~= "number" or step ~= step or math.abs(step) > 180 or M.cfg.lookWrite == false
      or G.lwTripped or G.lwUnavailable then
    return false
  end
  if G.btBefore == true then
    G.lwAheadPend = step
    return true
  end
  local s = G.lwSent
  if G.lwState ~= "on" or not s or G.lwSentAt ~= G.clock then return false end
  local ok, r = pcall(function() return Game.UTPPSetGameLook(s.yaw, s.pitch, step) end)
  if ok and r == true then
    G.lwAheadSends = (G.lwAheadSends or 0) + 1
    G.lwAheadLast, s.ahead = step, step
    return true
  end
  G.lwAheadFails = (G.lwAheadFails or 0) + 1
  return false
end
function M.Zoom(dir)
  -- while the unarmed zoom is on, Zoom steps it (closer = Zoom(-1) = one step in); the profile zoom is masked
  if M.ZoomWheel(-(tonumber(dir) or 0), (tonumber(dir) or 0) < 0 and "console zoom in" or "console zoom out") then
    return M.rt.zoom
  end
  M.rt.zoom = CM.clamp(M.rt.zoom + (dir or 0) * M.cfg.zoomStep, M.cfg.zoomMin, M.cfg.zoomMax)
  return M.rt.zoom
end

-- ================================================================ settings (utpp_core public API)
local FIELD_LABEL = { distance = "distance (m)", height = "height (m)", side = "shoulder offset (m)", fov = "FOV",
                      sens = "sensitivity" }
-- labels of the camera framing views (the fixed profile's fields)
local EDIT_LABEL = { distance = "Distance (m)", height = "Height (m)", side = "Shoulder offset (m)", fov = "FOV" }
function M.RegisterSettings(core)
  M.core = core
  core.RegisterSection("camera", "Camera")
  core.RegisterSection("camera_placement", "Camera placement (raw values)")
  local R = core.Register
  R({ key = "camera.impl", section = "camera", type = "bool", default = true, label = "Use Ultimate Third Person",
    desc = "On: the mod's own third-person camera. Off: the game's first-person view only (the third-person camera needs "
      .. "utpp_camera.archive + .xl).",
    apply = function(v) M.implApplied = v end })
  R({ key = "camera.startTpp", section = "camera", type = "bool", default = true, label = "Start in third person",
    desc = "Enter the third-person view after loading a save.", apply = function(v) M.rt.startTpp = v end })
  R({ key = "camera.body", section = "camera", type = "bool", default = true, label = "Show the full body",
    desc = "Uses the game's own third-person representation of V (TPP head, clothing) while the camera is active.",
    apply = function(v) M.rt.body = v; M.T.useRep = v end })
  R({ key = "camera.audio", section = "camera", type = "bool", default = true, label = "Audio listener at the camera",
    desc = "The active camera also moves the audio listener.", apply = function(v) M.rt.audio = v end })
  R({ key = "camera.sens", section = "camera", type = "float", default = 1.0, min = 0.2, max = 3, step = 0.05,
    label = "Orbit sensitivity", desc = "Free look / idle orbit speed.",
    apply = function(v) M.rt.sens = v end })
  R({ key = "camera.mouseScale", section = "camera", type = "float", default = 0, min = 0, max = 1, step = 0.005,
    format = "%.3f", label = "Mouse degrees per unit (0 = auto)",
    desc = "0 = measured from how far the game turns V for the same mouse movement.",
    apply = function(v) M.rt.mouseScale = v end })
  R({ key = "camera.invertX", section = "camera", type = "bool", default = false, label = "Invert horizontal orbit",
    apply = function(v) M.cfg.invertX = v end })
  R({ key = "camera.invertY", section = "camera", type = "bool", default = false, label = "Invert vertical orbit",
    apply = function(v) M.cfg.invertY = v end })
  R({ key = "camera.idleOrbit", section = "camera", type = "bool", default = true, label = "Orbit while standing",
    desc = "Standing still, the mouse orbits the camera around V instead of turning her.",
    apply = function(v) M.cfg.idleOrbit = v end })
  R({ key = "camera.pitchSource", section = "camera", type = "choice", default = "fpp", label = "Vertical aim",
    choices = { { "fpp", "Follow first-person aim" }, { "input", "Own (mouse / stick)" } },
    desc = "Follow first-person aim keeps aiming and interactions aligned with the view.",
    apply = function(v) M.cfg.pitchSource = v end })
  R({ key = "camera.pivotSource", section = "camera", type = "choice", default = "capsule", label = "Camera height source",
    choices = { { "capsule", "Body capsule (stand / crouch)" }, { "fpp", "First-person camera (old)" } },
    desc = "Capsule: V's position + 1.55 m standing / 1.05 m crouching. The first-person camera drops in the game's "
      .. "first-person melee / block / vehicle-exit animations.",
    apply = function(v)
      if v ~= M.cfg.pivotSource then
        M.cfg.pivotSource = v
        M.Log(string.format("pivot source -> %s%s", tostring(v), v == "fpp" and " (FPP camera eye height, old)"
          or string.format(" (stand %.2f m, crouch %.2f m, blend %.2f s)", M.cfg.standHeight, M.cfg.crouchHeight, M.cfg.stanceBlend)))
      end
    end })
  R({ key = "camera.followMaxRate", section = "camera", type = "float", default = 240, min = 0, max = 720, step = 10,
    format = "%.0f", label = "Follow turn speed limit (deg/s, 0 = off)",
    desc = "Body turns not made with the mouse (running backwards, turning animations) are followed at most this fast, "
      .. "with ease-in. Mouse turns stay instant.",
    apply = function(v)
      if v ~= M.cfg.followMaxRate then
        M.cfg.followMaxRate = v
        M.Log(string.format("follow yaw limit -> %s", v > 0 and string.format("%.0f deg/s (ease-in %.2f s)", v, M.cfg.followEase) or "off"))
      end
    end })
  R({ key = "camera.followHoldOnFlip", section = "camera", type = "bool", default = true, label = "Hold the view when V turns around",
    desc = "When V's body turns more than 120 deg within 0.3 s without the mouse, the camera keeps its direction for 0.6 s.",
    apply = function(v)
      if v ~= M.cfg.followHoldOnFlip then
        M.cfg.followHoldOnFlip = v
        M.Log("follow hold on a body flip -> " .. tostring(v))
      end
    end })
  R({ key = "camera.aimReturn", section = "camera", type = "bool", default = true, label = "Fast return after aiming",
    desc = "After aiming (first person) the camera returns over the aim return time and lands behind V's back.",
    apply = function(v) M.rt.aimReturn = v end })
  R({ key = "camera.aimReturnBlend", section = "camera", type = "float", default = 0.35, min = 0, max = 2, step = 0.05,
    label = "Aim return time (s)", apply = function(v) M.rt.aimReturnBlend = v end })
  -- over-the-shoulder aim (only with perspective.aiming = tpp)
  R({ key = "camera.aim.enabled", section = "camera", type = "bool", default = true, label = "Over-the-shoulder aiming",
    desc = "When 'Perspective while aiming' is third person: aiming a ranged weapon moves the camera close over the "
      .. "shoulder, V faces the camera direction and the view centre is where the shot goes.",
    apply = function(v) M.cfg.aimEnabled = v end })
  R({ key = "camera.decoupleLocomotion", section = "camera", type = "bool", default = true, label = "Free camera while V turns toward the movement",
    desc = "With UTPP Locomotion active (V turns toward the movement direction) the camera keeps its own world yaw and only "
      .. "the mouse / right stick turns it; off = the camera follows V's body yaw (shakes on quick direction changes).",
    apply = function(v) M.cfg.decoupleLocomotion = v end })
  R({ key = "camera.predictBodyTurn", section = "camera", type = "bool", default = false, label = "Free camera: cancel V's turn lag",
    desc = "While V turns toward the movement under the free camera, the turn of the coming game frame is predicted from "
      .. "the last one and taken out of the camera (the camera is attached to V and would turn with her for a frame).",
    apply = function(v) M.cfg.predictBodyTurn = v end })
  R({ key = "camera.worldCamera", section = "camera", type = "bool", default = true,
    label = "Camera as a separate world object",
    desc = "The third-person camera is its own entity in the world (needs Codeware), so V's body turns never carry it "
      .. "along. Off, or when it cannot be created: the camera component attached to V (with the turn prediction).",
    apply = function(v)
      v = v ~= false
      if v ~= (M.rt.worldCamera ~= false) then M.Log("world camera setting -> " .. tostring(v)) end
      M.rt.worldCamera = v
    end })
  R({ key = "camera.worldCameraRetry", section = "camera", type = "bool", default = true,
    label = "World camera: retry after a failure",
    desc = "When the separate world camera fails (e.g. right after loading a save the game still shows its loading camera), "
      .. "try to create it again after 8 s, up to 3 times per session, so the game look write comes back. Off: one failure "
      .. "= the camera attached to V for the rest of the session.",
    apply = function(v)
      v = v ~= false
      if v ~= (M.rt.worldCameraRetry ~= false) then M.Log("world camera retry setting -> " .. tostring(v)) end
      M.rt.worldCameraRetry = v
    end })
  R({ key = "camera.trace", section = "camera", type = "bool", default = false, label = "Camera trace (log, 10 per second)",
    desc = "Test aid: one 'camTrace' line per 0.1 s in third person (yaw / pitch / mode / Locomotion / body turn / aim weight / "
      .. "distance / pivot / mouse) and the aim offset status while aiming. Console: GetMod(\"utpp_core\").Set(\"camera.trace\", true).",
    apply = function(v) M.rt.trace = v end })
  R({ key = "camera.aim.distance", section = "camera", type = "float", default = 1.5, min = 0.6, max = 2.0, step = 0.05,
    label = "Aim distance (m)", apply = function(v) M.cfg.aimDistance = v end })
  R({ key = "camera.aim.boomPitchScale", section = "camera", type = "float", default = 0.35, min = 0, max = 1, step = 0.05,
    label = "Aim: camera orbit when looking up/down",
    desc = "While aiming a weapon the camera always looks where V aims; this sets how far its position orbits with the "
      .. "up / down look. 0 = the camera stays at shoulder height and only rotates, 1 = full orbit (over the head / under V). "
      .. "With camera.aim.boomFrame above 0 the camera orbits further at steep angles to keep V in the picture. "
      .. "The empty-hands zoom has its own setting (camera.zoomUnarmed.boomPitchScale).",
    apply = function(v) M.cfg.aimBoomPitchScale = v end })
  -- framing limit of the weapon aim and the unarmed zoom's own orbit (CM.solve inp.zoomShare);
  -- weapon aim default 0 (at 0.8 the camera orbited ~1:1 with the look past about -35 / +30)
  R({ key = "camera.aim.boomFrame", section = "camera", type = "float", default = 0.45, min = 0, max = 1, step = 0.05,
    label = "Aim: keep V in the picture when looking up/down",
    desc = "0 = no framing limit (pure partial orbit: V may leave the picture at steep angles). Above 0: while "
      .. "aiming a weapon at steep angles the camera orbits further so V's shoulder stays within this fraction of half the "
      .. "screen height from the centre (1 = up to the screen edge).",
    apply = function(v) M.cfg.aimBoomFrame = v end })
  R({ key = "camera.zoomUnarmed.boomPitchScale", section = "camera", type = "float", default = 0.25, min = 0, max = 1,
    step = 0.05, label = "Empty-hands zoom: camera orbit when looking up/down",
    desc = "While zooming with empty hands the camera always looks where V looks; this sets how far its position orbits "
      .. "with the up / down look. 0 = the camera stays at shoulder height and only rotates, 1 = full orbit (over the "
      .. "head / under V).",
    apply = function(v) M.cfg.zoomBoomPitchScale = v end })
  R({ key = "camera.zoomUnarmed.boomFrame", section = "camera", type = "float", default = 0.5, min = 0, max = 1, step = 0.05,
    label = "Empty-hands zoom: keep V in the picture when looking up/down",
    desc = "0 = no framing limit (pure partial orbit: V may leave the picture at steep angles). Above 0: the "
      .. "camera orbits further so V's shoulder stays within this fraction of half the screen height from the centre.",
    apply = function(v) M.cfg.zoomBoomFrame = v end })
  R({ key = "camera.aim.side", section = "camera", type = "float", default = 0.5, min = 0, max = 1.0, step = 0.01,
    label = "Aim shoulder offset (m)", apply = function(v) M.cfg.aimSide = v end })
  R({ key = "camera.aim.zoomFov", section = "camera", type = "bool", default = false, label = "Aim: zoom the FOV",
    desc = "On: aiming without a scope narrows the FOV to the weapon's aim FOV. Off (default): the camera only moves over the shoulder.",
    apply = function(v) M.cfg.aimZoomFov = v == true end })
  R({ key = "camera.aim.ownYaw", section = "camera", type = "bool", default = true, label = "Aim: camera keeps its direction",
    desc = "On: while aiming the camera keeps the direction of the mouse and V turns to it. Off: the camera aligns onto the game's look.",
    apply = function(v) M.cfg.aimOwnYaw = v ~= false end })
  -- game look sync
  R({ key = "camera.lookSync", section = "camera", type = "bool", default = false, -- OFF (VehicleCombat set did not hold the heading in game)
    label = "Camera = the game's look",
    desc = "On: while the camera is free or aiming, the game's own look (aim, shot, crosshair, movement heading) keeps its "
      .. "direction when V turns, and the camera shows exactly that look (mouse speed and aim sensitivity are the game's). "
      .. "Off: the camera integrates the mouse itself (old behaviour).",
    apply = function(v)
      v = v ~= false
      if v ~= (M.cfg.lookSync ~= false) then M.Log("game look sync setting -> " .. tostring(v)) end
      M.cfg.lookSync = v
    end })
  -- game look write
  R({ key = "camera.lookWrite", section = "camera", type = "bool", default = true,
    label = "Game look = camera direction",
    desc = "On: while the world camera is shown in third person (free camera or aiming), the game's own look (aim "
      .. "animation, shots, crosshair, movement heading) is set to the camera's direction every frame (UTPP.dll 0.6.5). "
      .. "Off, or with an older UTPP.dll: the game's look is not written (it turns with V).",
    apply = function(v)
      v = v ~= false
      if v ~= (M.cfg.lookWrite ~= false) then M.Log("game look write setting -> " .. tostring(v)) end
      M.cfg.lookWrite = v
    end })
  R({ key = "camera.aim.fov", section = "camera", type = "float", default = 30, min = 30, max = 90, step = 1, format = "%.0f",
    label = "Aim FOV", desc = "Used when the weapon's own aim FOV (stat AimFOV) cannot be read.",
    apply = function(v) M.cfg.aimFov = v end })
  R({ key = "camera.aim.autoSwap", section = "camera", type = "bool", default = true, label = "Aim: swap shoulder at walls",
    desc = "A wall within 0.5 m on the shoulder side for 0.3 s moves the aim camera to the other shoulder.",
    apply = function(v) M.cfg.aimAutoSwap = v end })
  R({ key = "camera.aim.occlusion", section = "camera", type = "bool", default = true, label = "Aim: detect a blocked muzzle",
    desc = "While aiming over the shoulder, checks whether cover between the weapon muzzle and the aim point blocks the "
      .. "muzzle although the view is clear (2 rays, 30 per second). Detection only: the shot is not changed.",
    apply = function(v) M.cfg.aimOcclusion = v end })
  R({ key = "camera.aim.range", section = "camera", type = "float", default = 60, min = 10, max = 200, step = 5,
    format = "%.0f", label = "Aim: occlusion range (m)", desc = "Length of the camera ray that finds the aim point.",
    apply = function(v) M.cfg.aimRange = v end })
  -- unarmed zoom (RMB with empty hands)
  R({ key = "camera.zoomUnarmed.enabled", section = "camera", type = "bool", default = true,
    label = "Zoom with empty hands (hold aim)",
    desc = "Holding aim (RMB) with empty hands in third person moves the camera close over the shoulder; the mouse wheel "
      .. "zooms further in steps (all steps stay in third person). Releasing aim returns to third person.",
    apply = function(v) M.cfg.zoomUnarmedEnabled = v end })
  R({ key = "camera.zoomUnarmed.steps", section = "camera", type = "int", default = 3, min = 1, max = 6,
    label = "Empty-hands zoom: steps", desc = "Third-person zoom steps (aim = step 1, wheel up / down while aiming).",
    apply = function(v) M.cfg.zoomUnarmedSteps = v end })
  R({ key = "camera.zoomUnarmed.distance", section = "camera", type = "float", default = 1.75, min = 0.6, max = 2.0, step = 0.05,
    label = "Empty-hands zoom: first distance (m)", apply = function(v) M.cfg.zoomUnarmedDistance = v end })
  R({ key = "camera.zoomUnarmed.lastDistance", section = "camera", type = "float", default = 0.6, min = 0.6, max = 2.0,
    step = 0.05, label = "Empty-hands zoom: last distance (m)", apply = function(v) M.cfg.zoomUnarmedLastDistance = v end })
  R({ key = "camera.zoomUnarmed.fov", section = "camera", type = "float", default = 45, min = 20, max = 90, step = 1,
    format = "%.0f", label = "Empty-hands zoom: first FOV", apply = function(v) M.cfg.zoomUnarmedFov = v end })
  R({ key = "camera.zoomUnarmed.lastFov", section = "camera", type = "float", default = 25, min = 15, max = 90, step = 1,
    format = "%.0f", label = "Empty-hands zoom: last FOV", apply = function(v) M.cfg.zoomUnarmedLastFov = v end })
  R({ key = "camera.zoomUnarmed.fppHandoff", section = "camera", type = "bool", default = false,
    label = "Empty-hands zoom: FPP past last step",
    desc = "On: past the last step the view goes to first person and the game's own (Kiroshi) zoom takes over. "
      .. "Off: the wheel stops at the last third-person step.", apply = function(v) M.cfg.zoomUnarmedFppHandoff = v end })
  R({ key = "camera.collision", section = "camera", type = "bool", default = true, label = "Camera collision",
    apply = function(v) M.cfg.collision = v end })
  R({ key = "camera.enterTime", section = "camera", type = "float", default = 0.45, min = 0, max = 2, step = 0.05,
    label = "Enter time (s)", apply = function(v) M.cfg.enterTime = v end })
  R({ key = "camera.exitTime", section = "camera", type = "float", default = 0.35, min = 0, max = 2, step = 0.05,
    label = "Exit time (s)", apply = function(v) M.cfg.exitTime = v end })
  R({ key = "camera.poseNudge", section = "camera", type = "bool", default = true, label = "Pose nudge after the full body appears",
    desc = "Standing still, re-sends the game's own stand animation event 0.4 s and 0.9 s after the third-person body is shown "
      .. "(against a T-pose after loading) and 0.1 s / 0.5 s after a game menu closes.", apply = function(v) M.rt.poseNudge = v end })
  R({ key = "camera.fppWithWeapon", section = "camera", type = "bool", default = true, label = "First person with a weapon",
    desc = "Only without the automatic perspective module: a drawn weapon switches the camera to first person. With the "
      .. "module installed its weapon and aiming options decide instead.",
    apply = function(v) M.rt.fppWithWeapon = v end })
  do local i, dp = M.FIXED_PROFILE, CM.DEFAULT_PROFILES[M.FIXED_PROFILE]
    for _, f in ipairs(CM.PROFILE_FIELDS) do
      local lim = CM.PROFILE_LIMITS[f]
      -- only the fixed profile's raw fields get a section (view "all"); the other slots stay registered
      -- without a widget so old settings files load and save unchanged
      local fixed = i == M.FIXED_PROFILE
      R({ key = M.ProfileKey(f), section = fixed and "camera_placement" or nil, type = "float",
        default = dp[f], min = lim[1], max = lim[2], step = lim[3], format = (lim[3] >= 1) and "%.0f" or "%.2f",
        label = fixed and ("Camera placement: " .. FIELD_LABEL[f]) or string.format("%d %s: %s (unused)", i, dp.name, FIELD_LABEL[f]),
        apply = function(v)
          M.profiles[i][f] = v
          if i == M.rt.profileIndex then
            CM.blendRetarget(M.blend, M.profiles[i])
            -- the editor view of this field shows the new value (console Set, reset, raw widget in the "all" view)
            if type(core.RefreshView) == "function" then pcall(core.RefreshView, "camera.edit." .. f) end
          end
        end })
    end
  end
  -- Camera placement views (S3 editor, single fixed profile): the fields of the fixed profile as presentation
  -- views (not settings: no new key, nothing stored; each edit is core.Set of exactly one camera.pN.<field>, N = the
  -- active slot = M.FIXED_PROFILE). Limits / step / format = the raw fields (CM.PROFILE_LIMITS, same units; FOV in camera
  -- component units). Distance, shoulder offset, FOV and height open "Camera placement" (ranks 1-4, before "Orbit
  -- sensitivity"). The profile sensitivity has no view: with one profile it only multiplies "Orbit sensitivity"
  -- (cfg.sens = camera.sens * profile sens), so camera.framingSens stays a raw field (view "all", default 1.0).
  -- Without utpp_core.RegisterView (older core) only the raw fields exist, as before.
  if type(core.RegisterView) == "function" then
    local EDIT = { distance = { "player", 1, "camera distance" }, side = { "player", 2, "shoulder offset" },
                   fov = { "player", 3, "field of view" }, height = { "player", 4, "camera height" } }
    for _, f in ipairs(CM.PROFILE_FIELDS) do
      local lim, e = CM.PROFILE_LIMITS[f], EDIT[f]
      if lim and e then
        local okV, errV = pcall(core.RegisterView, {
          id = "camera.edit." .. f, group = "frame", level = e[1], rank = e[2], type = "float",
          label = EDIT_LABEL[f],
          desc = "The " .. e[3] .. " of the third-person camera.",
          min = lim[1], max = lim[2], step = lim[3], format = (lim[3] >= 1) and "%.0f" or "%.2f",
          get = function() return M.profiles[M.rt.profileIndex][f] end,
          set = function(v) core.Set(M.ProfileKey(f), v) end,
          default = function()
            local dp = CM.DEFAULT_PROFILES[M.rt.profileIndex]
            return dp and dp[f]
          end })
        if not okV then M.Log("profile editor view " .. f .. " FAILED: " .. tostring(errV)) end
      end
    end
  end
  M.Log(string.format("settings registered in utpp_core %s: camera %s, profile %d/%d, perspective %s",
    tostring(core.VERSION), M.Enabled() and "on" or "off", M.rt.profileIndex, M.rt.profileCount,
    M.persp and "utpp_perspective" or "own"))
  return true
end

function M.Status()
  local T, S = M.T, M.S
  local p = CM.blendCurrent(M.blend)
  return string.format("utpp_camera %s enabled=%s state=%s w=%.2f want=%s quick=%s blocked=%s epoch=%d enters=%d "
    .. "exits=%d | profile=%d/%d (%s d=%.2f h=%.2f side=%.2f fov=%.0f) zoom=%.2f shoulder=%.2f free=%s pitchSrc=%s "
    .. "mouseUnit=%.4f (cal %s, %d samples) | arm=%s hits=%d rays=%d writes=%d errors=%d reactivations=%d comp=%s rep=%s "
    .. "| perspective=%s body=%s | tppSignal=%s via %s changes=%d failures=%d nudges=%s | pivot=%s followMaxRate=%.0f "
    .. "holdOnFlip=%s flips=%d aimReturns=%d menuNudges=%d | aim=%s aimT=%.2f aimW=%.2f aimFov=%s aimSwaps=%d "
    .. "aimBlocked=%d aimBlockD=%s | zoomUnarmed=%s k=%d/%d fpp=%s zw=%.2f handoffs=%d",
    M.VERSION, tostring(M.Enabled()), T.state, T.w, tostring(T.want), tostring(T.quick),
    tostring(T.blocked), T.epoch, T.enters, T.exits, M.rt.profileIndex, M.rt.profileCount, tostring(p.name), p.distance,
    p.height, p.side, p.fov, M.rt.zoom, M.shoulder.s, tostring(S.look.free), tostring(M.cfg.pitchSource),
    CM.mouseUnit(M.cal, M.cfg, M.rt.mouseScale), tostring(M.cal.value), M.cal.samples,
    S.col.len and string.format("%.2f", S.col.len) or "-", S.col.hits, M.stats.rays, M.stats.writes, M.stats.errors,
    M.stats.reactivations, tostring(M.G and M.G.compState), tostring(M.G and M.G.repOwned),
    M.Perspective() and "utpp_perspective" or "own",
    M.BodyModule() and "utpp_body" or "own representation",
    M.tppSig.sent == nil and "-" or (M.tppSig.sent and "on" or "off"), tostring(M.tppSig.route), M.tppSig.changes,
    M.tppSig.failures, tostring(M.G and M.G.nudgeCount or 0), tostring(M.cfg.pivotSource), M.cfg.followMaxRate or 0,
    tostring(M.cfg.followHoldOnFlip), M.stats.flips, M.stats.aimReturns, M.stats.menuNudges,
    M.aim.on and "on" or "off", CM.aimTime(M.aim), M.aim.w, M.aimFov and string.format("%.0f", M.aimFov) or "-",
    M.stats.aimSwaps, M.occ.blocked and 1 or 0,
    (M.occ.info and M.occ.info.dMuzzleHit) and string.format("%.2f", M.occ.info.dMuzzleHit) or "-",
    M.zoomU.on and "on" or "off", M.zoomU.k, CM.zoomUnarmedSteps(M.cfg), tostring(M.zoomU.fpp), M.zoomU.w, M.zoomU.handoffs)
    .. " | " .. M.VehicleStatus() .. " | " .. M.WorldCameraStatus() .. " | " .. M.LookSyncStatus()
    .. " | " .. M.LookWriteStatus() .. " | " .. M.ShoulderInputStatus()
end

-- ================================================================ game adapter (skipped by the offline math tests)
if type(registerForEvent) == "function" then
  local G = { clock = 0, player = nil, playerKey = nil, comp = nil, fpp = nil, compState = "unknown", repOwned = false,
              layout = nil, rayForm = nil, lockSet = false, present = 0, prevBodyYaw = nil, frozen = 0,
              watch = 0, misses = 0, reactWindow = 0, reactCount = 0, diagUntil = -1, diagNext = 0, last = nil,
              sessionStarted = false, ls = CM.newLookSync(), lookUsed = false, lwState = "off" }
  M.G = G
  M.sink = function(line)
    print(line)
    if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
  end
  local IN = { mouseX = 0, mouseY = 0, padX = 0, padY = 0, moveX = 0, moveY = 0, held = {}, events = 0 }
  M.input = IN

  local function vec(v) return CM.v3(v.x, v.y, v.z) end
  local function V4(p) return Vector4.new(p.x, p.y, p.z, 1) end

  local function nameOf(n)
    local ok, s = pcall(function() return Game.NameToString(n) end)
    if ok and type(s) == "string" then return s end
    return tostring(n)
  end

  -- "pressed" | "released" | nil (type unreadable) of a button action (gameinputActionType BUTTON_PRESSED 0,
  -- BUTTON_RELEASED 1, TOGGLE_PRESSED 8, TOGGLE_RELEASED 9; the CET form is a number, an enum value or a name).
  -- the numbers follow RED4ext.SDK (PRESSED 0, RELEASED 1, HOLD_PROGRESS 2, HOLD_COMPLETE 3, MULTITAP_BEGIN_LAST 4,
  -- MULTITAP_END_LAST 5, AXIS_CHANGE 6, RELATIVE_CHANGE 7, TOGGLE_PRESSED 8, TOGGLE_RELEASED 9, REPEAT 10); before, 7
  -- (RELATIVE_CHANGE) counted as pressed and 8 (TOGGLE_PRESSED) as released. Game logs so far show the value edge
  -- ("no GetType"), so the numeric path was not live.
  local function actionEdge(action)
    if type(ListenerAction) ~= "table" or type(ListenerAction.GetType) ~= "function" then return nil, "no GetType" end
    local okT, t = pcall(ListenerAction.GetType, action)
    if not okT or t == nil then return nil, okT and "GetType nil" or tostring(t) end
    local num = tonumber(t)
    if num == nil and type(t) ~= "string" then
      local okV, v = pcall(function() return t.value end)
      if okV then num = tonumber(v); if num == nil and type(v) == "string" then t = v end end
    end
    if num ~= nil then
      if num == 0 or num == 8 then return "pressed", "number" end
      if num == 1 or num == 9 then return "released", "number" end
      return "other", "number"
    end
    local s = tostring(t)
    if type(t) ~= "string" then
      local okS, ns = pcall(function() return Game.NameToString(t) end)
      if okS and type(ns) == "string" and ns ~= "" then s = ns end
    end
    if string.find(s, "RELEASED", 1, true) then return "released", "name" end
    if string.find(s, "PRESSED", 1, true) then return "pressed", "name" end
    return "other", "name"
  end
  -- unarmed zoom wheel: the game's ZoomIn / ZoomOut actions (input context "Aiming": IK_MouseWheelUp / Down, pad
  -- d-pad up / down) -> one step per press (type edge; value edge when the type is unreadable)
  IN.zoomHeld = {}
  local function onZoomAction(name, action, v)
    local edge, form = actionEdge(action)
    local pressed
    if edge == nil then
      pressed = v > 0.5 and not IN.zoomHeld[name]
      IN.zoomHeld[name] = v > 0.5
      form = "value edge (" .. tostring(form) .. ")"
    else
      pressed = edge == "pressed"
    end
    if not M.zuIn.seen[name] then
      M.zuIn.seen[name] = true
      M.Log(string.format("zoom input: game action %s seen through PlayerPuppet.OnAction (press from %s)", name, tostring(form)))
    end
    if pressed then M.ZoomWheel(name == "ZoomIn" and 1 or -1, "wheel " .. name) end
  end

  local function onAction(action)
    local okN, n = pcall(function() return ListenerAction.GetName(action) end)
    if not okN then return end
    local name = nameOf(n)
    local okV, v = pcall(function() return ListenerAction.GetValue(action) end)
    v = (okV and tonumber(v)) or 0
    IN.events = IN.events + 1
    if name == "ZoomIn" or name == "ZoomOut" then return onZoomAction(name, action, v) end
    if name == "CameraMouseX" then IN.mouseX = IN.mouseX + v
    elseif name == "CameraMouseY" then IN.mouseY = IN.mouseY + v
    elseif name == "CameraX" then IN.padX = v
    elseif name == "CameraY" then IN.padY = v
    elseif name == "MoveX" then IN.moveX = v
    elseif name == "MoveY" then IN.moveY = v
    elseif name == "Forward" or name == "Back" or name == "Left" or name == "Right" then IN.held[name] = v > 0.5 end
  end

  local function getSystem(name)
    local ok, s = pcall(function() return Game.GetScriptableSystemsContainer():Get(name) end)
    if ok then return s end
    return nil
  end

  local function psm(player)
    local r = {}
    pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if not bb then return end
      r.vehicle = bb:GetInt(defs.Vehicle)
      r.workspot = bb:GetInt(defs.IsInWorkspot)
      r.high = bb:GetInt(defs.HighLevel)
      pcall(function() r.loco = bb:GetInt(defs.LocomotionDetailed) end)
      pcall(function() r.device = bb:GetBool(defs.IsControllingDevice) end)
    end)
    return r
  end
  -- stance for the capsule pivot: PSM Locomotion Crouch 1 / Slide 9 / SlideFall 10 / CrouchSprint 11 / CrouchDodge 12,
  -- LocomotionDetailed Crouch 3 / Slide 5 (vanilla gamePSMLocomotionStates / gamePSMDetailedLocomotionStates)
  -- what the game writes (vanilla locomotionTransitions.script): Crouch = Locomotion 1 +
  -- Detailed 3, CrouchSprint = Locomotion 1 + Detailed 30 (CrouchSprintEvents.OnEnter), Slide = 9 / 10 + Detailed 5, a workspot = 9 +
  -- Detailed 1, the walk toggle = Locomotion stand + Detailed 3. CrouchSprint is NOT a crouch for the body: the DLL plays it as Stand +
  -- Sprint (adapter::PsmCrouchStance = Detailed 3 only), so Detailed 30 is a stand here too.
  local CROUCH_LOCO = { [1] = true, [9] = true, [10] = true, [11] = true, [12] = true }
  local function crouchState(player)
    local loco, det = 0, 0
    pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if not bb then return end
      if defs.Locomotion ~= nil then loco = tonumber(bb:GetInt(defs.Locomotion)) or 0 end
      det = tonumber(bb:GetInt(defs.LocomotionDetailed)) or 0
    end)
    -- (research, verified in locomotionTransitions.script l.930-931 / 756-758): the walk toggle (G)
    -- writes LocomotionDetailed Crouch 3 while Locomotion stays stand, and a workspot writes Locomotion 9 with detailed
    -- Stand 1 - neither is a crouch. Crouch = PSM Locomotion crouch states; Slide 9 / SlideFall 10 only when the detailed
    -- state is not Stand (workspot)
    local slide = (loco == 9 or loco == 10) and det ~= 1
    -- the seated slide (LocomotionDetailed Slide 5; Locomotion 10 / Detailed 5) has its own, lower
    -- pivot (camera.slideHeight); it stays a crouch for the look-up cap. 4th result = slide pivot wanted
    return (loco == 1 and det ~= 30) or loco == 11 or loco == 12 or slide, loco, det, slide and det == 5
  end
  local function bbBool(defName, field)
    local ok, v = pcall(function()
      local d = GetAllBlackboardDefs()[defName]
      return Game.GetBlackboardSystem():Get(d):GetBool(d[field])
    end)
    return ok and v == true
  end
  local function inMenu()
    if bbBool("UI_System", "IsInMenu") then return true end
    local okP, paused = pcall(function() return Game.GetSystemRequestsHandler():IsGamePaused() end)
    return okP and paused == true
  end
  local function photoMode()
    local ok, v = pcall(function() return Game.GetPhotoModeSystem():IsPhotoModeActive() end)
    return ok and v == true
  end
  -- shoulder-swap input gate: side-effect-free PSM read (no gameCameraSet / lock state, unlike lockGameOwned).
  -- Returns "vehicle" | a reason | nil. PSM Vision ~= 0 = scanner (the R3 press tags there); Takedown / carrying: an R3
  -- hold also picks up a body; swimming: R3 = dive (Alternative scheme).
  local function shoulderPsmBlock(player)
    local why = nil
    local okAll = pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if not bb then error("no PSM blackboard") end
      local function int(f)
        if defs[f] == nil then return 0 end
        local okI, v = pcall(function() return bb:GetInt(defs[f]) end)
        return (okI and tonumber(v)) or 0
      end
      local function flag(f)
        if defs[f] == nil then return false end
        if int(f) ~= 0 then return true end
        local okB, v = pcall(function() return bb:GetBool(defs[f]) end)
        return okB and v == true
      end
      local high = int("HighLevel")
      if flag("Vehicle") or flag("MountedToVehicle") then why = "vehicle"
      elseif flag("IsInWorkspot") then why = "workspot"
      elseif high == 6 or int("Swimming") ~= 0 then why = "swimming"
      elseif high < 0 or high > 2 then why = "scene"
      elseif flag("Takedown") then why = "takedown"
      elseif flag("BodyCarrying") or flag("Carrying") then why = "carrying"
      elseif flag("IsControllingDevice") then why = "device"
      elseif int("Vision") ~= 0 then why = "scanner" end
    end)
    if not okAll then return "PSM unreadable" end
    if why == nil and bbBool("Braindance", "IsActive") then why = "braindance" end
    return why
  end
  -- drains the reds request counter every frame (own pcall: a missing function never counts as an update error), then
  -- gates and applies (M.ShoulderInputApply). nil function and -1 (system missing) are the same case: one log line.
  local function shoulderInputPoll(player, menu, photo)
    local si = M.shIn
    if si.retryAt ~= nil and G.clock < si.retryAt then return end
    local okT, n = pcall(function() return Game.UTPPShoulderSwapTake() end)
    n = okT and tonumber(n) or nil
    if n == nil or n < 0 then
      if si.missing ~= true then
        si.missing = true
        M.Log("shoulder input: " .. (okT and "system UTPPShoulderInput missing" or "Game.UTPPShoulderSwapTake unavailable")
          .. " (r6\\scripts\\UTPP\\utpp_shoulder_input.reds not loaded) - the swap key / controller gesture does nothing")
      end
      si.retryAt = G.clock + M.SHOULDER_IN_RETRY
      return
    end
    if si.missing == true then M.Log("shoulder input: UTPPShoulderInput found") end
    si.missing, si.retryAt = false, nil
    if n <= 0 then return end
    local okS, src = pcall(function() return Game.UTPPShoulderSwapTrigger() end)
    src = (okS and type(src) == "string" and src ~= "") and src or "input"
    local vehicle = M.vh.held == true
    local psmWhy = nil
    if not (menu or photo) then
      psmWhy = shoulderPsmBlock(player)
      if psmWhy == "vehicle" then vehicle, psmWhy = true, nil end
    end
    local why = M.ShoulderInputBlock({ enabled = M.Enabled(), menu = menu, photo = photo, loading = G.present < 1.5,
      vehicle = vehicle, psm = psmWhy, tpp = M.T.state == "tpp" or M.T.state == "entering" })
    M.ShoulderInputApply(n, src, why, G.clock)
  end
  local function weaponDrawn(player)
    local ok, item = pcall(function()
      return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
    end)
    return ok and item ~= nil
  end
  -- aim: weapon AimFOV stat (vanilla zoomTransitions.script SendZoomAnimFeatureData:
  -- stats.GetStatValue(Cast<StatsObjectID>(weapon.GetEntityID()), gamedataStatType.AimFOV)); the CET call form that
  -- works is kept (logged once); unreadable / outside cfg.aimFovMin..aimFovMax -> nil (camera.aim.fov is used)
  local FOV_FORMS = {
    { "GetStatValue(EntityID, gamedataStatType.AimFOV)",
      function(eid) return Game.GetStatsSystem():GetStatValue(eid, gamedataStatType.AimFOV) end },
    { "GetStatValue(EntityID, \"AimFOV\")", function(eid) return Game.GetStatsSystem():GetStatValue(eid, "AimFOV") end },
  }
  local function readAimFov(item, rec)
    local okE, eid = pcall(function() return item:GetEntityID() end)
    if not okE or eid == nil then
      M.LogOnce("aimfov", "aim FOV: weapon entity id unreadable (" .. tostring(eid) .. ") - camera.aim.fov used")
      return nil
    end
    local errs = {}
    for i, f in ipairs(FOV_FORMS) do
      if G.fovForm == nil or G.fovForm == i then
        local ok, v = pcall(f[2], eid)
        local n = ok and tonumber(v) or nil
        if n ~= nil then
          if G.fovForm == nil then G.fovForm = i; M.Log("aim FOV: weapon stat read via StatsSystem " .. f[1]) end
          if n >= M.cfg.aimFovMin and n <= M.cfg.aimFovMax then return n end
          M.Log(string.format("aim FOV: weapon %s AimFOV=%.2f outside %.0f-%.0f - camera.aim.fov %.0f used", tostring(rec), n,
            M.cfg.aimFovMin, M.cfg.aimFovMax, M.cfg.aimFov))
          return nil
        end
        errs[#errs + 1] = f[1] .. ": " .. (ok and ("returned " .. tostring(v)) or tostring(v))
      end
    end
    M.LogOnce("aimfov", "aim FOV: weapon stat AimFOV unreadable (" .. table.concat(errs, "; ") .. ") - camera.aim.fov used")
    return nil
  end
  -- WeaponRight item -> { record (TweakDB name for the log), type (gamedataItemType name | "" fists | nil unreadable),
  -- fov }; cached per record (the stat / record reads only on a weapon change)
  local function weaponInfo(item)
    if item == nil then return { record = "fists", type = "" } end
    local okR, tdbid = pcall(function() return ItemID.GetTDBID(item:GetItemID()) end)
    if not okR then tdbid = nil end
    local rec = "?"
    if tdbid ~= nil then
      local okS, s = pcall(function() return TDBID.ToStringDEBUG(tdbid) end)
      rec = (okS and type(s) == "string" and s ~= "") and s or tostring(tdbid)
    end
    local c = G.aimWeapon
    if c and c.record == rec then return c end
    c = { record = rec }
    if tdbid ~= nil then
      local okT, t = pcall(function()
        local r = TweakDBInterface.GetWeaponItemRecord(tdbid)
        if r == nil then return nil end
        local ty = r:ItemType():Type()
        if type(ty) == "string" then return ty end
        local okV, nm = pcall(function() return ty.value end)
        if okV and type(nm) == "string" then return nm end
        return tostring(ty)
      end)
      if okT then c.type = t end
    end
    if CM.weaponRanged(true, c.type, nil) then c.fov = readAimFov(item, rec) end -- no stat read for melee / fists
    G.aimWeapon = c
    return c
  end
  -- fills o.upperBody / o.meleeState (PSM, like utpp_perspective readObs) and, only while UpperBody = Aim (6), the
  -- WeaponRight item: o.weapon (record), o.weaponRanged, o.fov
  local function aimObserve(player, o)
    local okB, errB = pcall(function()
      local D = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), D)
      if not bb then return end
      o.upperBody = tonumber(bb:GetInt(D.UpperBody))
      if D.MeleeWeapon ~= nil then o.meleeState = tonumber(bb:GetInt(D.MeleeWeapon)) end
    end)
    if not okB then M.LogOnce("aimpsm", "aim: PSM UpperBody read FAILED (aim state stays off): " .. tostring(errB)) end
    if o.upperBody ~= 6 then return o end
    local okI, item = pcall(function()
      return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
    end)
    if not okI then item = nil end
    local info = weaponInfo(item)
    o.weapon, o.fov = info.record, info.fov
    o.weaponRanged = CM.weaponRanged(item ~= nil, info.type, o.meleeState)
    return o
  end
  -- unarmed zoom: PSM UpperBody / MeleeWeapon / CombatGadget / LeftHandCyberware / Locomotion / ZoomLevel (float, log
  -- only) and, only while UpperBody = Aim (6), whether WeaponRight holds an item (o.weaponInHand; nil = unreadable).
  -- Plain values only: the item handle is dropped at once.
  local function zoomObserve(player, o)
    local okB, errB = pcall(function()
      local D = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), D)
      if not bb then return end
      o.upperBody = tonumber(bb:GetInt(D.UpperBody))
      if D.MeleeWeapon ~= nil then o.meleeState = tonumber(bb:GetInt(D.MeleeWeapon)) end
      if D.CombatGadget ~= nil then o.gadget = tonumber(bb:GetInt(D.CombatGadget)) end
      if D.LeftHandCyberware ~= nil then o.lhcw = tonumber(bb:GetInt(D.LeftHandCyberware)) end
      if D.Locomotion ~= nil then o.loco = tonumber(bb:GetInt(D.Locomotion)) end
      if D.ZoomLevel ~= nil then pcall(function() o.gameZoom = tonumber(bb:GetFloat(D.ZoomLevel)) end) end
    end)
    if not okB then M.LogOnce("zupsm", "zoomUnarmed: PSM read FAILED (zoom state stays off): " .. tostring(errB)) end
    if o.upperBody ~= 6 then return o end
    local okI, item = pcall(function()
      return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
    end)
    if okI then o.weaponInHand = item ~= nil
    else M.LogOnce("zuslot", "zoomUnarmed: WeaponRight read FAILED (zoom state stays off): " .. tostring(item)) end
    item = nil
    return o
  end
  -- aim occlusion: muzzle world position of the WeaponRight object (WeaponObject.GetMuzzleSlotWorldTransform
  -- [cyberpunk/items/weapon.script:135]) -> v3 | nil, why.
  -- the object is re-resolved from the slot on every call and never stored (no handle outlives the call)
  local function muzzlePos(player)
    local okI, item = pcall(function()
      return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
    end)
    if not okI or item == nil then item = nil; return nil, "no weapon object in WeaponRight" end
    local ok, p = pcall(function() return item:GetMuzzleSlotWorldTransform().position end)
    item = nil
    if not ok then return nil, "GetMuzzleSlotWorldTransform: " .. tostring(p) end
    if p == nil then return nil, "GetMuzzleSlotWorldTransform: no position" end
    local v = vec(p)
    if v.x == 0 and v.y == 0 and v.z == 0 then return nil, "GetMuzzleSlotWorldTransform: zero position" end
    return v
  end
  -- active camera world position (CET forms of the Transform parameter differ between versions; form logged once)
  local function activeCameraPos(camSys)
    if G.camForm == nil or G.camForm == "ret" then
      local ok, a, b = pcall(function() return camSys:GetActiveCameraWorldTransform() end)
      for _, r in ipairs({ b, a }) do
        if ok and r ~= nil and type(r) ~= "boolean" then
          local okP, p = pcall(function() return r.position end)
          if okP and p ~= nil then G.camForm = "ret"; return vec(p) end
        end
      end
    end
    if G.camForm == nil or G.camForm == "arg" then
      local ok, p = pcall(function()
        local xf = Transform.new()
        camSys:GetActiveCameraWorldTransform(xf)
        return xf.position
      end)
      if ok and p ~= nil and (p.x ~= 0 or p.y ~= 0 or p.z ~= 0) then G.camForm = "arg"; return vec(p) end
    end
    if G.camForm == nil then G.camForm = "none"; M.Log("active camera position unreadable - watchdog off") end
    return nil
  end
  local function vehicleTppActive(player)
    local ok, v = pcall(function()
      local veh = Game.GetMountedVehicle(player)
      return veh ~= nil and veh:GetCameraManager():IsTPPActive()
    end)
    return ok and v == true
  end
  -- the game's vehicle camera renders in third person for a driver -> no FPP camera activation.
  -- Mounted + IsTPPActive, and the driver seat (mounting slot seat_front_left) or PSM Vehicle Driving 1 / Combat 2 /
  -- Transition 4 / Turret 5 / DriverCombat 6 (not Passenger 3 / Scene 7: their cinematics keep the game's own rules).
  -- Returns true, detail | false.
  local GUARD_VEH_STATES = { [1] = true, [2] = true, [4] = true, [5] = true, [6] = true }
  local function vehicleTppGuard(player)
    if not vehicleTppActive(player) then return false end
    local vs = 0
    pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if bb then vs = tonumber(bb:GetInt(defs.Vehicle)) or 0 end
    end)
    if GUARD_VEH_STATES[vs] then return true, "vehicle state " .. vs end
    local driver = false
    pcall(function()
      local info = Game.GetMountingFacility():GetMountingInfoSingleWithObjects(player)
      local slot = info and info.slotId and info.slotId.id
      if slot ~= nil then
        local okN, s = pcall(function() return Game.NameToString(slot) end)
        slot = okN and s or tostring(slot)
        driver = string.find(tostring(slot), "seat_front_left", 1, true) ~= nil
      end
    end)
    if driver then return true, "driver seat (vehicle state " .. vs .. ")" end
    return false
  end
  M.VehicleTppGuard = function(player) return vehicleTppGuard(player) end

  -- components of the current player (cached per entity)
  local function components(player)
    local okK, key = pcall(function() return tostring(player:GetEntityID().hash) end)
    key = okK and key or tostring(player)
    if key ~= G.playerKey or G.comp == nil then
      G.playerKey = key
      local okC, c = pcall(function() return player:FindComponentByName("utppCamera") end)
      local okF, f = pcall(function() return player:FindComponentByName("camera") end)
      G.comp = okC and c or nil
      G.fpp = okF and f or nil
      local st = G.comp and "ok" or "missing"
      if st ~= G.compState then
        G.compState = st
        if st == "ok" then
          M.Log("component utppCamera found (ArchiveXL patch active)")
        else
          M.Log("component utppCamera MISSING - install utpp_camera.archive + utpp_camera.archive.xl (archive\\pc\\mod); camera stays first person")
        end
      end
    end
    return G.comp, G.fpp
  end

  -- FPP camera world forward / position; the matrix layout is chosen once against the active camera while in FPP
  local function fppView(fpp, camSys)
    if not fpp then return nil end
    local ok, m = pcall(function() return fpp:GetLocalToWorld() end)
    if not ok or m == nil then return nil end
    if G.layout == nil and camSys and M.T.state == "fpp" then
      local okA, af = pcall(function() return camSys:GetActiveCameraForward() end)
      if okA and af then
        local a = CM.norm(vec(af))
        local best, bestDot = nil, 0.95
        for _, lay in ipairs({ "rows", "cols" }) do
          local okF, f = pcall(CM.matrixForward, m, lay)
          if okF then
            local d = CM.dot(CM.norm(f), a)
            if d > bestDot then best, bestDot = lay, d end
          end
        end
        if best then G.layout = best; M.Log(string.format("FPP camera matrix layout: %s (dot %.3f)", best, bestDot)) end
      end
    end
    local lay = G.layout or "rows"
    local okF, f = pcall(CM.matrixForward, m, lay)
    local okP, p = pcall(CM.matrixPosition, m, lay)
    if not okF or not okP then return nil end
    local yaw, pitch = CM.yawPitchFromForward(f)
    if yaw == nil then return nil end
    return { yaw = yaw, pitch = pitch, pos = p }
  end

  -- raycast: nearest hit distance over the collision groups; the CET return form of the out parameter is detected
  local function makeRaycast(sqs)
    return function(from, to)
      local dir, len = CM.norm(CM.sub(to, from))
      if len < 1e-3 then return nil end
      local best = nil
      for _, group in ipairs(M.rt.groups) do
        local endP = best and CM.add(from, CM.scale(dir, best)) or to
        M.stats.rays = M.stats.rays + 1
        local ok, a, b = pcall(function()
          return sqs:SyncRaycastByCollisionGroup(V4(from), V4(endP), group, false, false)
        end)
        if ok then
          local res = nil
          if a == true and type(b) ~= "boolean" and b ~= nil then res = b
          elseif type(a) ~= "boolean" and a ~= nil then res = a end
          if res ~= nil and a ~= false then
            local okP, hp = pcall(function() return res.position end)
            if okP and hp ~= nil then
              local d = CM.dist(vec(hp), from)
              if d > 1e-4 and d <= len + 1e-3 and (best == nil or d < best) then best = d end
              if G.rayForm == nil then G.rayForm = "ok"; M.Log("raycast form ok (first hit)") end
            end
          end
        elseif G.rayForm ~= "error" then
          G.rayForm = "error"
          M.Log("raycast FAILED: " .. tostring(a))
        end
      end
      return best
    end
  end

  local function representation(player, on)
    local ok, err = pcall(function()
      local evt = NewObject(on and "gameActivateTPPRepresentationEvent" or "gameDeactivateTPPRepresentationEvent") -- RTTI names (aliases cannot be created in CET)
      player:QueueEvent(evt)
    end)
    local okF, errF = pcall(function()
      local feat = NewObject("gameAnimFeature_TPPRepresentation")
      feat.IsActive = on and true or false
      AnimationControllerComponent.ApplyFeature(player, "TPPRepresentation", feat, 0)
    end)
    M.Log(string.format("TPP representation %s: event %s, feature %s", on and "ON" or "OFF",
      ok and "ok" or ("FAILED " .. tostring(err)), okF and "ok" or ("FAILED " .. tostring(errF))))
    return ok
  end

  -- heading lock re-assert. Observed (native GameLook counters every 2 s): notLocked 0 until
  -- a while, then 12432 of 15966 ticks (78 %) and writes 0 for minutes; the lock came back only inside slides. Cause (game
  -- behaviour, fact from the counters + the vanilla set table in utpp_bodyturn.reds): the PSM queues a camera parameter set at
  -- the change out of the slide / jump (HoverJump) / vault - Default, Vault and HoverJump have headingLocked = false, Slide has
  -- it true - and the apply writes headingLocked from the set (GameLook.cpp evidence 1). G.lockSet is a cache that was never read
  -- back, only a lock edge (aim / free look / Locomotion) wrote it again. While the lock is wanted and cached the component is now
  -- read every frame; false = the game ended it: write it again (camera.lockReassert), count and log it, hold the mouse
  -- calibration (the game turned V onto its camera heading when the lock ended).
  -- Scope: only where UTPP steers the view. utpp_bodyturn.reds (UTPPLookSyncSwappable) leaves the game's own set - and so the
  -- game's lock decision - in vehicles, on ladders, in climbs, workspots, scenes, knockdowns, while carrying a body and while
  -- controlling a device; the same states utpp.reds hands back to the game (UTPPPsmBlockMask: MountedToVehicle, HighLevel not
  -- 0..2 incl. unknown values, Swimming, Takedown) are left alone, and so are the vault and the hover jump (their sets have no
  -- lock; the PSM queues Default once they end: a lock set before that would be a true -> false edge again, i.e. V turned onto
  -- the camera by the game). Any other set the PSM queued (the last one: Game.UTPPLookWorldStatus gameSet=, e.g. consumables) than
  -- Default / Slide / UseConsumable is the game's too, for lockReassertSetHold s. No read-back write there; the first plain frame writes it, lockReassertGrace s after the
  -- first plain one (the queued set is applied after the state changed).
  -- At most lockReassertMax writes per lockReassertWindow s, then a pause without any (a state that resets the lock every frame
  -- must not make the game rotate V every frame): lockReassertPause s, doubled with each pause in a row up to lockReassertPauseMax
  -- (2 / 4 / 8 / 16 / 30 s: a state that never lets go costs 8 writes per pause, not per 2 s for ever). Logged: the first three
  -- pauses, then one line "keeps being reset". The backoff starts again when the PSM state (Locomotion / LocomotionDetailed)
  -- changes - the pause ends at once - after lockReassertStable s without a lost lock, and with the session.
  -- gamePSMDetailedLocomotionStates: Climb 8, Vault 9, Ladder 10..13, HoverJump 21, Knockdown 29, Felled 31 (utpp_locomotion.reds,
  -- MotionAdapter.hpp PsmDetailed)
  local LOCK_GAME_DETAILED = { [8] = "climb", [9] = "vault", [10] = "ladder", [11] = "ladder", [12] = "ladder", [13] = "ladder",
                               [21] = "hoverJump", [29] = "knockdown", [31] = "felled" }
  -- camera parameter sets that are UTPP's to steer (UTPPLookSyncSwappable without Vault / HoverJump, excluded by their states)
  local LOCK_OWN_SETS = { Default = true, Slide = true }
  -- and the sets that only inherit Default's headingLocked = false and change something else (fppCameraParamSets.tweak): UseConsumable
  -- (pitch limits), SpaceShuttleInterior (the staged-gameplay area Tier2: defaultTransition.script queues it at every locomotion
  -- state enter there instead of Default, so the name never goes back to Default in the whole area). The vanilla PSM queues no new
  -- set when a consumable ends either, so such a name stays in "gameSet=" afterwards. They are plain for the heading lock as long
  -- as the game look sync is OFF (camera.lookSync, default). With it ON utpp_bodyturn.reds (UTPPLookSyncSwappable) leaves them
  -- the game's ("consumables keep the game's set"): the lock without the world heading would break the sync evidence (look turning
  -- with V -> resends -> "broken" for the session) - there they count like any unlisted set (3 s rule below) and the lock is
  -- restored with the sync set (lockVerify). Sets derived from Default that are the game's on purpose are excluded by their PSM
  -- states (carrying: BodyPickUp / BodyCarry ...).
  local LOCK_PLAIN_SETS = { UseConsumable = true, SpaceShuttleInterior = true }
  -- the last camera parameter set the PSM queued (utpp_bodyturn.reds LookStatus "gameSet="); nil = unknown / no such native
  local function gameCameraSet()
    local okS, s = pcall(function() return Game.UTPPLookWorldStatus() end)
    if not okS or type(s) ~= "string" then return nil end
    local name = string.match(s, "gameSet=(%S+)")
    if name == nil or name == "None" then return nil end
    return name
  end
  -- PSM blackboard of the player: the reason the game's own camera set stays (nil = plain locomotion), Locomotion, Detailed
  local function lockGameOwned(player)
    local why, loco, det = nil, 0, 0
    local okAll = pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if not bb then error("no PSM blackboard") end
      local function int(f)
        if defs[f] == nil then return 0 end
        local okI, v = pcall(function() return bb:GetInt(defs[f]) end)
        return (okI and tonumber(v)) or 0
      end
      local function flag(f) -- the PSM exposes these as int or bool depending on the field
        if defs[f] == nil then return false end
        if int(f) ~= 0 then return true end
        local okB, v = pcall(function() return bb:GetBool(defs[f]) end)
        return okB and v == true
      end
      loco, det = int("Locomotion"), int("LocomotionDetailed")
      local high = int("HighLevel") -- gamePSMHighLevel: Default 0, SceneTier1..5 = 1..5, Swimming 6; 1 / 2 are no block (utpp.reds)
      if flag("Vehicle") or flag("MountedToVehicle") then why = "vehicle"
      elseif flag("IsInWorkspot") then why = "workspot"
      elseif high == 6 or int("Swimming") ~= 0 then why = "swimming"
      elseif high < 0 or high > 2 then why = "scene"
      elseif flag("Takedown") then why = "takedown"
      elseif LOCK_GAME_DETAILED[det] then why = LOCK_GAME_DETAILED[det]
      elseif flag("BodyCarrying") or flag("Carrying") then why = "carrying"
      elseif flag("IsControllingDevice") then why = "device" end
    end)
    if not okAll then why = "PSM unreadable" end
    if not why then
      -- any other set the PSM queued is the game's - for lockReassertSetHold s after it was first seen (the vanilla PSM queues no
      -- set at the end of every state, e.g. consumables: the name of a finished state stays; the PSM states above are the exact
      -- ones, this is the catch-all for sets nobody listed)
      local set = gameCameraSet()
      if set then
        -- the 3 s start with a new name or a new loss: the lock was read as lost without a break (the reads follow each other at
        -- <= lockReassertGrace while it lasts); a gap (the lock was back, or not wanted) means the game applied the set again
        if G.lockSetName ~= set or G.lockSetSeen == nil or G.clock - G.lockSetSeen > (M.cfg.lockReassertSetGap or 0.5) then
          G.lockSetName, G.lockSetSince = set, G.clock
        end
        G.lockSetSeen = G.clock
        if not (LOCK_OWN_SETS[set] or (LOCK_PLAIN_SETS[set] and M.cfg.lookSync == false)) then
          if G.clock - G.lockSetSince < (M.cfg.lockReassertSetHold or 3.0) then why = "game set " .. set
          else
            M.LogOnce("relockStale:" .. set, string.format("game set %s unchanged for %.0f s: treated as plain locomotion for the "
              .. "heading lock (the PSM queues no set at the end of every state)", set, M.cfg.lockReassertSetHold or 3.0))
          end
        end
      end
    end
    return why, loco, det
  end
  local function lockVerify(fpp, player)
    if M.cfg.lockReassert == false then return end
    -- ragdoll camera: the game's camera parameter sets during a knockdown / get-up (PSM Locomotion 0, LocomotionDetailed 1 in
    -- the log) are not ours to undo; the first frames after the episode wait ragLockGrace s (G.lockHold below)
    if G.ragPhase ~= nil then G.lockHold = G.clock + (M.cfg.ragLockGrace or 0.5); return end
    local ok, cur = pcall(function() return fpp.headingLocked end)
    if not ok or cur ~= false then return end -- unreadable (older build) / still locked
    if G.lockHold ~= nil and G.clock < G.lockHold then return end -- just left a game-owned state: wait for the queued set
    local why, loco, det = lockGameOwned(player)
    if why == nil and G.lockOwned then
      -- the first plain frame after a game-owned one: the set the game queued for the state change is applied about now
      G.lockOwned, G.lockHold = false, G.clock + (M.cfg.lockReassertGrace or 0.05)
      return
    end
    if why then
      G.lockOwned = true
      G.lockHold = G.clock + (M.cfg.lockReassertGrace or 0.05)
      M.stats.lockSkips = (M.stats.lockSkips or 0) + 1
      M.LogOnce("relockSkip:" .. why, string.format("heading lock lost while %s (PSM Locomotion %d, LocomotionDetailed %d): "
        .. "the game's own camera set stays, not set again there", why, loco, det))
      return
    end
    local R = G.relock
    if not R then R = { n = 0, winStart = G.clock, total = 0, paused = nil, pauses = 0 }; G.relock = R end
    -- a new PSM state or a long quiet time is new information: no backoff is carried over (the pause ends at once)
    local key = loco * 1000 + det
    if (R.key ~= nil and R.key ~= key) or (R.lostAt ~= nil and G.clock - R.lostAt > (M.cfg.lockReassertStable or 10.0)) then
      R.paused, R.n, R.winStart, R.pauses = nil, 0, G.clock, 0
    end
    R.key, R.lostAt = key, G.clock
    if R.paused ~= nil then
      if G.clock < R.paused then return end
      R.paused, R.n, R.winStart = nil, 0, G.clock
    end
    if G.clock - R.winStart > (M.cfg.lockReassertWindow or 1.0) then R.n, R.winStart = 0, G.clock end
    if R.n >= (M.cfg.lockReassertMax or 8) then
      local pause = math.min((M.cfg.lockReassertPause or 2.0) * 2 ^ R.pauses, M.cfg.lockReassertPauseMax or 30.0)
      R.paused, R.pauses = G.clock + pause, R.pauses + 1
      if R.pauses <= 3 then
        M.Log(string.format("heading lock lost %d times within %.1f s -> not set again for %.1f s (the game resets it every frame; "
          .. "PSM Locomotion %d, LocomotionDetailed %d)", R.n, M.cfg.lockReassertWindow or 1.0, pause, loco, det))
      else
        M.LogOnce("relockStuck", string.format("heading lock keeps being reset (%d pauses in a row, PSM Locomotion %d, "
          .. "LocomotionDetailed %d): paused up to %.0f s each time, no further pause is logged (relocks= in the status)",
          R.pauses, loco, det, M.cfg.lockReassertPauseMax or 30.0))
      end
      return
    end
    local okW, errW
    local ls = G.ls
    if M.cfg.lookSync ~= false and ls and ls.sent == true and ls.route ~= nil and not ls.broken and M.LookSyncSend then
      -- look sync ON: a bare headingLocked = true would leave the world heading off (the set the game applied has none) and the sync
      -- evidence would count the look turning with V; the sync set (headingLocked + maintainWorldSpaceHeading) is sent again
      okW, errW = M.LookSyncSend(G.player or player, true)
      if okW then ls.sentAt = G.clock else okW = false end -- (the evidence skips 0.2 s after a send: the set applies in a later tick)
    else
      okW, errW = pcall(function() fpp.headingLocked = true end)
    end
    if not okW then M.LogOnce("relock", "headingLocked write FAILED (re-lock): " .. tostring(errW)); return end
    R.n, R.total = R.n + 1, R.total + 1
    M.stats.lockRelocks = (M.stats.lockRelocks or 0) + 1
    G.unlockAt = G.clock
    if R.total <= 3 or R.total % 25 == 0 then
      M.Log(string.format("heading lock lost (the game applied a camera parameter set: PSM Locomotion %d, LocomotionDetailed %d) "
        .. "-> set again (#%d, stance %s)", loco, det, R.total, G.stance or "stand"))
    end
  end

  -- verifyPlayer: the player = the lock may be read back and written again (nil: no read-back, the old behaviour). Menu / photo
  -- mode frames have no solve result (res = nil): the only call site is not reached there, nothing is written while frozen.
  local function setHeadingLock(fpp, on, verifyPlayer)
    if not fpp then return end
    if on == G.lockSet then
      if on == true and verifyPlayer then lockVerify(fpp, verifyPlayer) end
      return
    end
    local ok, err = pcall(function() fpp.headingLocked = on end)
    if ok then
      -- the game turns V onto its camera heading after the lock ends (no mouse calibration then)
      if G.lockSet == true and not on then G.unlockAt = G.clock end
      G.lockSet = on
    else M.LogOnce("lock", "headingLocked write FAILED: " .. tostring(err)) end
    if M.LookSyncReconcile then M.LookSyncReconcile() end -- param set follows the lock
  end

  -- game look sync: the FPP camera parameter set M.cfg.lookSyncSet (vanilla fppCameraParamSets.tweak
  -- VehicleCombat: headingLocked + maintainWorldSpaceHeading, yaw +-180, no rubber band) while the heading lock is set,
  -- the game's own set back when it ends. Route: utpp_bodyturn.reds Game.UTPPLookWorldSet (also swaps the PSM's
  -- locomotion sets while on), fallback the event queued from here.
  local function lookSyncSend(player, on)
    local set = tostring(M.cfg.lookSyncSet or "VehicleCombat")
    local okG, r = pcall(function() return Game.UTPPLookWorldSet(on, set) end)
    if okG and r == true then return true, "Game.UTPPLookWorldSet" end
    if not player then return false, "no player (" .. tostring(r) .. ")" end
    local okE, errE = pcall(function()
      local evt = NewObject("gameSetCameraParamsEvent")
      local name = on and set or "Default"
      if type(CName) == "table" and type(CName.new) == "function" then name = CName.new(name) end
      evt.paramsName = name
      player:QueueEvent(evt)
    end)
    if okE then return true, "CET gameSetCameraParamsEvent" end
    return false, "Game.UTPPLookWorldSet: " .. tostring(r) .. "; CET event: " .. tostring(errE)
  end
  -- on a change of (heading lock set) and (camera.lookSync): one send per change, the route logged once per session
  local function lookSyncReconcile()
    local ls = G.ls
    local want = G.lockSet == true and M.cfg.lookSync ~= false
    if want == (ls.sent == true) then return end
    -- camera.lookSync switched off while the lock is set: the set stays until the lock ends (restoring the game's set
    -- now would also unlock the heading under the free / aiming camera); the camera uses its own yaw at once
    if not want and G.lockSet == true then return end
    local ok, route = lookSyncSend(G.player, want)
    ls.sent, ls.sentAt, ls.carried = want, G.clock, 0
    if want then
      ls.route = ok and route or nil
      -- resends are counted per session (a reset per lock never reached "broken": aim enter / exit
      -- re-locks every few seconds and the camera kept flipping between own yaw and the carried game look)
      if ok then
        M.LogOnce("lsRoute", string.format("game look sync: FPP camera param set %s via %s (headingLocked + "
          .. "maintainWorldSpaceHeading: the game's look keeps its world heading while V turns) - the camera shows the "
          .. "game's look once confirmed", tostring(M.cfg.lookSyncSet), route))
      else
        M.LogOnce("lsNoRoute", "game look sync FAILED: no route (" .. tostring(route) .. ") -> own camera yaw")
      end
    end
  end
  M.LookSyncReconcile = lookSyncReconcile
  M.LookSyncSend = lookSyncSend -- lockVerify (defined above) re-sends the sync set instead of a bare lock
  -- resend after evidence that the look turned with V (the game applied another set)
  local function lookSyncResend()
    local ls = G.ls
    local ok, route = lookSyncSend(G.player, true)
    ls.sentAt = G.clock
    M.Log(string.format("game look sync: the look turned with V -> param set %s sent again (%d/%d) via %s",
      tostring(M.cfg.lookSyncSet), ls.resends, M.cfg.lookSyncResends or 2, ok and route or ("FAILED " .. tostring(route))))
  end

  -- utpp_bodyturn module (GetMod; nil while missing, looked up again every 5 s)
  local function btModule()
    if G.btMod == nil or (G.btMod == false and G.clock >= (G.btRetry or 0)) then
      local okB, bt = pcall(function() return GetMod("utpp_bodyturn") end)
      G.btMod = (okB and type(bt) == "table") and bt or false
      G.btRetry = G.clock + 5
    end
    return G.btMod or nil
  end
  -- While utpp_bodyturn turns V by adjustTransform (Locomotion
  -- apply, aim align) the game overrides its FPP look in the ticks the turn lands (look pulled onto V's yaw, pitch ~0;
  -- the aim pitch read back jumped 52.9 -> -8.7 -> 51.1 on the 'aim align' rows). True when its request counter
  -- (execStats.adjSeq) moved this frame or the last one (both CET orders covered): the camera does not read the game's
  -- look pitch back in such a frame (CM.lookStep adjustPending) and the trace marks it (adj=1).
  local function adjustPendingStep()
    local bt = btModule()
    local seq = (bt and type(bt.execStats) == "table") and tonumber(bt.execStats.adjSeq) or nil
    local new = seq ~= nil and G.adjSeqSeen ~= nil and seq ~= G.adjSeqSeen
    G.adjSeqSeen = seq
    G.adjPend = new or G.adjNewPrev == true
    G.adjNewPrev = new
    if G.adjPend then G.adjFrames = (G.adjFrames or 0) + 1 end
    return G.adjPend
  end

  -- game look write: t = { yaw, pitch } (the camera shown this frame) or nil. One line per
  -- change of the state (on / fail / off); the native logs its own reasons (UTPP log "UTPP gamelook: ...").
  local function lookWriteFrame(t, why)
    -- V's turn requested by utpp_bodyturn earlier in this CET frame (only when it runs BEFORE the camera; else
    -- M.OnBodyLookStep sends it after this write); consumed by this frame either way
    local ahead = (G.btBefore == true and type(G.lwAheadPend) == "number") and G.lwAheadPend or 0
    G.lwAheadPend = nil
    if t then
      if G.lwUnavailable then return end
      -- the native tripped (divergence guard): no calls, the camera keeps the game's look pitch again (lookWriteOn
      -- false; aim / zoom frames with an adjustTransform turn of V pending do not read it - CM.lookStep adjustPending).
      -- the native re-arms the write after its cooldown (3 s, at most 3 trips per game session), so it is tried
      -- again after cfg lookWriteRetry s; only a permanent trip (status tripPermanent=1) ends it for the session
      if G.lwTripped then
        if G.lwTripPermanent or G.clock < (G.lwRetryAt or math.huge) then return end
        G.lwTripped = false
      end
      G.lwSets = (G.lwSets or 0) + 1
      local ok, r = pcall(function() return Game.UTPPSetGameLook(t.yaw, t.pitch, ahead) end)
      if not ok then
        G.lwUnavailable = true
        M.Log("game look write: Game.UTPPSetGameLook unavailable (" .. tostring(r) .. ") - UTPP.dll older than 0.6.5b? "
          .. "The game's look is not written")
        return
      end
      local state = r == true and "on" or "fail"
      if state == "fail" then G.lwFails = (G.lwFails or 0) + 1 end
      if state ~= G.lwState then
        local was = G.lwState
        G.lwState = state
        if state == "on" then
          M.Log(string.format("game look write ON (%s)%s: the game's look = the camera direction each frame (yaw %.1f pitch %.1f)",
            tostring(why), was == "tripped" and " - re-armed after a trip" or "", t.yaw, t.pitch))
        else
          local okS, st = pcall(function() return Game.UTPPGameLookStatus() end)
          if okS and type(st) == "string" and string.find(st, " tripped=1 ", 1, true) then
            -- the loop tripped in the native -> clean stop on this side (status: tripped); tried again
            -- after cfg lookWriteRetry s unless the native reports the trip as permanent (tripPermanent=1)
            G.lwTripped, G.lwState = true, "tripped"
            G.lwTripPermanent = string.find(st, " tripPermanent=1 ", 1, true) ~= nil
            local retry = tonumber(M.cfg.lookWriteRetry) or 3.5
            G.lwRetryAt = G.clock + retry
            -- one line per native trip (tripCount); a retry still inside the native's cooldown is silent
            local tc = tonumber(string.match(st, " tripCount=(%d+) "))
            if tc == nil or tc ~= G.lwTripSeen or G.lwTripPermanent then
              G.lwTripSeen = tc
              G.lwTrips = (G.lwTrips or 0) + 1
              M.Log("game look write TRIPPED (native divergence guard) -> "
                .. (G.lwTripPermanent and "off for this session" or string.format("off, tried again in %.1f s", retry))
                .. ", the camera follows the game's look pitch meanwhile: " .. st)
            end
          else
            M.Log("game look write not possible (no write): " .. (okS and tostring(st) or "status unavailable"))
          end
        end
      end
      -- (with the camera yaw / pitch it was made for: the parallax correction is taken out of the read-back)
      G.lwSent = state == "on" and { yaw = t.yaw, pitch = t.pitch, camYaw = t.camYaw or t.yaw, camPitch = t.camPitch or t.pitch,
        ahead = ahead } or nil
      G.lwSentAt = state == "on" and G.clock or nil -- M.OnBodyLookStep re-sends only this frame's target
      if state == "on" and ahead ~= 0 then G.lwAheadSends, G.lwAheadLast = (G.lwAheadSends or 0) + 1, ahead end
    elseif G.lwState == "on" or G.lwState == "fail" then
      local was = G.lwState
      G.lwState, G.lwSent = "off", nil
      pcall(function() Game.UTPPStopGameLook() end)
      if was == "on" then
        M.Log("game look write OFF (" .. tostring(why) .. ")")
        G.lwOffAt = G.clock -- own pitch for cfg.ownPitchHold s after this edge
      end
    end
  end

  -- look write ramp (cfg.lookWriteRamp): the first write after a vehicle hold starts from the game's current look pitch and reaches the camera pitch
  -- on a smoothstep over lookWriteRamp s (counted in frame time, a stalled frame adds at most 0.05 s). Only t.pitch (what is sent) changes: t.camPitch stays
  -- the camera's, so the read-back correction (viewPitch = read back - (sent - camPitch)) keeps the camera on its own pitch. t = nil (no write wanted) ends it.
  local function lookRampStep(t, view, dt)
    local dur = tonumber(M.cfg.lookWriteRamp) or 0
    local R = G.lwRamp
    if dur <= 0 or t == nil then G.lwRamp = nil; return end
    if R == nil then
      if G.lwState ~= "on" and G.afterHoldNow == true and view and tonumber(view.pitch) then
        R = { t = 0, from = view.pitch, to = t.pitch }
        G.lwRamp = R
        M.Log(string.format("game look write ramp: pitch %.1f -> %.1f over %.2f s (first write after a vehicle hold)", R.from, t.pitch, dur))
      else
        return
      end
    else
      R.t = R.t + math.min(math.max(dt or 0, 0), 0.05)
    end
    local u = CM.smoothstep(R.t / dur)
    t.pitch = R.from + (t.pitch - R.from) * u
    if R.t >= dur then G.lwRamp = nil end
  end

  -- parallax-correct game look target (review): while aiming / zooming the camera sits off V's eye
  -- line (shoulder side 0.45 m, the partial boom up to ~0.9 m vertically), so a game look parallel to the camera forward
  -- missed the crosshair point by several degrees at 5-10 m. Aiming: the look goes from the eye (FPP camera position,
  -- the eyePos of the solve) to P = the first hit of the camera forward ray (from cfg aimLookNear m past the camera - not
  -- V - to camera.aim.range m; no hit = the far end; P at least aimLookMinAhead m in front of the eye). One ray per frame;
  -- without a ray the last P distance is reused for aimLookHold s, then the camera yaw / pitch. G.lwp = distance of P (trace).
  -- Returns { yaw, pitch, camYaw, camPitch } (camYaw / camPitch = the camera the target was made for).
  -- (review): the correction (eye -> P minus the camera yaw / pitch) is limited per axis to
  -- cfg aimLookMaxDeg and moves at <= aimLookRate deg/s (CM.lookParallaxStep, state G.lwPar): a floor / ceiling hit at
  -- steep pitch swung the look yaw 45-130 deg, near / far edges jumped it in one frame. No P (no ray, hold over, eye
  -- unknown) = the correction glides back to 0; not aiming (also the release flush) = the camera yaw / pitch at once.
  local function aimLookAngles(res, eye, ray, aiming, dt)
    local t = { yaw = res.yaw, pitch = res.pitch, camYaw = res.yaw, camPitch = res.pitch }
    G.lwp = nil
    if not aiming or M.cfg.aimLookParallax == false or not res.world then
      G.lwpDist, G.lwpAt, G.lwPar = nil, nil, nil
      return t
    end
    local fwd = CM.forward(res.yaw, res.pitch)
    local range = math.max(tonumber(M.cfg.aimRange) or 60, 1)
    local near = CM.clamp(tonumber(M.cfg.aimLookNear) or 0.3, 0, range * 0.5)
    local d = nil
    if eye and ray and G.lwpAt ~= G.clock then
      local hit = ray(CM.add(res.world, CM.scale(fwd, near)), CM.add(res.world, CM.scale(fwd, range)))
      d = hit and math.min(range, near + math.max(0, hit)) or range
      G.lwpDist, G.lwpAt = d, G.clock
    elseif eye and G.lwpDist and G.clock - (G.lwpAt or -1e9) <= (tonumber(M.cfg.aimLookHold) or 0.5) + 1e-6 then
      d = G.lwpDist
    end
    local y, p = nil, nil
    if d then
      d = math.max(d, CM.dot(CM.sub(eye, res.world), fwd) + (tonumber(M.cfg.aimLookMinAhead) or 1.0))
      y, p = CM.anglesEyeToPoint(eye, CM.add(res.world, CM.scale(fwd, d)))
      if y ~= nil then G.lwp = d end
    end
    G.lwPar = G.lwPar or { dy = 0, dp = 0 }
    t.yaw, t.pitch = CM.lookParallaxStep(G.lwPar, M.cfg, dt, res.yaw, res.pitch, y, p)
    return t
  end

  -- release flush (review: the unlock makes the game turn V by YawInput = the last written look yaw -
  -- the entity yaw, RotateEntityToCamera, GameLook.cpp evidence 1 / 3 - so with the parallax correction every RMB release
  -- turned V by the correction, and the camera pitch jumped onto the corrected pitch a frame later): when the lock would
  -- end while the last write carried a correction, the lock is held and the plain camera yaw / pitch written (aimLookAngles,
  -- not aiming) until the read-back matches it (|lwerr|, |lwperr| < 0.5 deg), at most cfg aimLookFlush frames; only while
  -- the look write runs on the shown world camera in third person. prev = the target written last frame (G.lwSent).
  local function lookWriteFlush(lock, prev, held)
    if lock or G.lockSet ~= true or held or prev == nil or G.lwState ~= "on" or M.cfg.lookWrite == false
        or M.T.state ~= "tpp" or G.use ~= "world" then
      G.lwFlush = nil
      return lock
    end
    local corrected = math.abs(CM.wrapDeg(prev.yaw - (prev.camYaw or prev.yaw))) > 1e-3
      or math.abs(prev.pitch - (prev.camPitch or prev.pitch)) > 1e-3
    local matched = G.lwErr ~= nil and math.abs(G.lwErr) < 0.5 and math.abs(G.lwPErr or 0) < 0.5
    if corrected and G.lwFlush == nil then G.lwFlush = 0 end
    if G.lwFlush ~= nil and (corrected or not matched) and G.lwFlush < (tonumber(M.cfg.aimLookFlush) or 3) then
      G.lwFlush = G.lwFlush + 1
      return true
    end
    G.lwFlush = nil
    return lock
  end

  -- TPP signal route: script global first, the scriptable system as the fallback
  M.tppSender = function(on)
    local okG, r = pcall(function() return Game.UTPPSetTppOverride(on) end)
    if okG and r == true then return true, "Game.UTPPSetTppOverride" end
    local okS, r2 = pcall(function()
      return Game.GetScriptableSystemsContainer():Get("UTPPPerspectiveState"):SetTppOverride(on)
    end)
    if okS and r2 ~= nil then return true, "UTPPPerspectiveState:SetTppOverride" end
    return false, string.format("Game.UTPPSetTppOverride %s; UTPPPerspectiveState %s",
      okG and ("returned " .. tostring(r)) or ("error " .. tostring(r)), okS and ("returned " .. tostring(r2)) or ("error " .. tostring(r2)))
  end
  -- (C2): HeadGuard's TPP value (utpp_loothead.reds UTPPHeadGuardSetValue, squared m; M.InsideHideStep)
  -- -> ok, info, missing (the call raised: no such native in this UTPP.dll / CET - not a failure of the native)
  M.headGuardValue = function(v)
    local okG, r = pcall(function() return Game.UTPPHeadGuardSetValue(v) end)
    if okG and r == true then return true, "Game.UTPPHeadGuardSetValue" end
    if not okG then return false, "Game.UTPPHeadGuardSetValue " .. tostring(r), true end
    return false, "Game.UTPPHeadGuardSetValue returned " .. tostring(r)
  end

  -- ============================================================ world camera (separate entity)
  -- Only plain values are kept across frames (M.wc.id is an EntityID value); the entity and its camera component are
  -- re-resolved through the spawner on every use.
  local function idDefined(id)
    if id == nil then return false end
    local okD, d = pcall(function() return EntityID.IsDefined(id) end)
    if okD and type(d) == "boolean" then return d end
    local okH, h = pcall(function() return id.hash end)
    if not okH or h == nil then return false end
    local s = tostring(h)
    return s ~= "0" and s ~= "0ULL"
  end
  local function idText(id)
    local ok, h = pcall(function() return id.hash end)
    return (ok and h ~= nil) and tostring(h) or tostring(id)
  end
  local function staticSys()
    local ok, s = pcall(function() return Game.GetStaticEntitySystem() end)
    if ok and s ~= nil then return s end
    return nil, ok and "Game.GetStaticEntitySystem() returned nil" or tostring(s)
  end
  -- templatePath (ResRef) forms tried in order; the working one is named in the spawn line
  local RESREF_FORMS = {
    { "ResRef.FromString", function(p) return ResRef.FromString(p) end },
    { "string", function(p) return p end },
  }
  -- Codeware StaticEntitySystem (scripts/World/StaticEntitySystem.reds, src/App/World/StaticEntitySystem.cpp): spawned
  -- by the world entity spawner (no population / streaming), attached on creation, disposed on world detach.
  -- -> id, how | nil, why, retry
  local function spawnStatic()
    local sys, why = staticSys()
    if not sys then return nil, "Codeware StaticEntitySystem unavailable (" .. tostring(why) .. ")" end
    local okR, ready = pcall(function() return sys:IsReady() end)
    if okR and ready == false then return nil, "Codeware StaticEntitySystem not ready", true end
    pcall(function() sys:DespawnTagged(M.WC_TAG) end) -- a camera entity left by an earlier CET instance (mod reload)
    local errs = {}
    for _, f in ipairs(RESREF_FORMS) do
      local ok, id = pcall(function()
        local spec = StaticEntitySpec.new()
        spec.templatePath = f[2](M.WC_TEMPLATE)
        spec.position = Vector4.new(0, 0, 0, 1)
        spec.orientation = Quaternion.new(0, 0, 0, 1)
        spec.attached = true
        pcall(function() spec.tags = { M.WC_TAG } end)
        return sys:SpawnEntity(spec)
      end)
      if ok and idDefined(id) then return id, "Codeware StaticEntitySystem (templatePath via " .. f[1] .. ")", false, "static" end
      errs[#errs + 1] = f[1] .. ": " .. (ok and ("no entity id (" .. tostring(id) .. ")") or tostring(id))
    end
    return nil, "StaticEntitySystem.SpawnEntity failed (" .. table.concat(errs, "; ") .. ")"
  end
  -- CET exEntitySpawner (used by other CET mods) - only without Codeware
  local function spawnEx()
    if exEntitySpawner == nil then return nil, "CET exEntitySpawner unavailable" end
    local ok, id = pcall(function()
      local tr = WorldTransform.new()
      tr:SetPosition(Vector4.new(0, 0, 0, 1))
      tr:SetOrientation(Quaternion.new(0, 0, 0, 1))
      return exEntitySpawner.Spawn(M.WC_TEMPLATE, tr, "")
    end)
    if ok and idDefined(id) then return id, "CET exEntitySpawner", false, "ex" end
    return nil, "exEntitySpawner.Spawn failed (" .. (ok and ("no entity id (" .. tostring(id) .. ")") or tostring(id)) .. ")"
  end
  local function wcSpawn()
    local id, how, retry, route = spawnStatic()
    if id ~= nil or retry then return id, how, retry, route end
    local id2, how2, retry2, route2 = spawnEx()
    if id2 ~= nil then return id2, how2, retry2, route2 end
    return nil, how .. "; " .. how2, false
  end
  local function wcEntity()
    local W = M.wc
    if W.id == nil then return nil, "no entity" end
    if W.route == "static" then
      local sys, why = staticSys()
      if not sys then return nil, why end
      local ok, e = pcall(function() return sys:GetEntity(W.id) end)
      if ok and e ~= nil then return e end
      return nil, ok and "StaticEntitySystem.GetEntity: nil" or tostring(e)
    end
    local ok, e = pcall(function() return Game.FindEntityByID(W.id) end)
    if ok and e ~= nil then return e end
    return nil, ok and "Game.FindEntityByID: nil" or tostring(e)
  end
  -- camera component of the spawned entity, resolved now (never stored)
  local function wcComp()
    local e, why = wcEntity()
    if e == nil then return nil, why end
    local ok, c = pcall(function() return e:FindComponentByName("utppCamera") end)
    e = nil
    if ok and c ~= nil then return c end
    return nil, ok and "component utppCamera not found on the camera entity" or tostring(c)
  end
  local function wcDespawn()
    local W = M.wc
    if W.id == nil then return "no entity" end
    local info
    if W.route == "static" then
      local sys, why = staticSys()
      if sys then
        local ok, r = pcall(function() return sys:DespawnEntity(W.id) end)
        info = ok and ("StaticEntitySystem.DespawnEntity " .. tostring(r)) or ("DespawnEntity error " .. tostring(r))
      else info = "no StaticEntitySystem (" .. tostring(why) .. ")" end
    else
      local ok, r = pcall(function()
        local e = Game.FindEntityByID(W.id)
        if e == nil then return false end
        exEntitySpawner.Despawn(e)
        return true
      end)
      info = ok and ("exEntitySpawner.Despawn " .. tostring(r)) or ("Despawn error " .. tostring(r))
    end
    W.id = nil
    return info
  end
  -- forget the entity without any game call (session end / shutdown without a player: the world is going away and
  -- Codeware's StaticEntitySystem disposes its entities on the world detach)
  local function wcForget(why)
    local W = M.wc
    if W.id ~= nil then
      W.releases = W.releases + 1
      M.Log(string.format("world camera: released (%s): entity %s forgotten - no game calls", tostring(why), tostring(W.idStr)))
    end
    M.wc = newWc()
    M.wc.spawns, M.wc.fails, M.wc.activations = W.spawns, W.fails, W.activations
    M.wc.switches, M.wc.releases = W.switches, W.releases
    G.use = nil
  end
  -- release with game calls (live world): deactivate when it is the camera in use, then despawn
  local function wcRelease(why)
    M.PoseWriteTpp(false)
    local W = M.wc
    if W.id == nil then return end
    if G.use == "world" then
      local c = wcComp()
      if c then pcall(function() c:Deactivate(0, false) end) end
      G.use = nil
      M.Log(string.format("world camera: deactivated (%s)", tostring(why)))
    end
    local info = wcDespawn()
    W.releases = W.releases + 1
    M.Log(string.format("world camera: released (%s): %s", tostring(why), tostring(info)))
    if W.state ~= "failed" then W.state = "off" end
  end
  -- hand the view between the world camera and utppCamera at the last shown pose (vh.last). ctx = { comp, wcomp, ppos,
  -- playerQ }. Activation only while the camera is shown (third person, not handed to a vehicle camera).
  local function switchCam(to, why, ctx)
    local from = G.use or "comp"
    if from == to then return true end
    G.use = to
    M.wc.switches = M.wc.switches + 1
    local vh = M.vh
    local live = (CM.trActive(M.T) or M.T.state == "settle") and not vh.handed
    local tgt = (to == "world") and ctx.wcomp or ctx.comp
    local old = (to == "world") and ctx.comp or ctx.wcomp
    local p = vh.last
    local okA, errA = true, nil
    if live then
      if tgt == nil then
        okA, errA = false, (to == "world") and "no world camera" or "utppCamera missing"
      else
        okA, errA = pcall(function()
          if p then
            local wq = CM.qFromYawPitch(p.yaw or 0, p.pitch or 0)
            if to == "world" then
              tgt:SetLocalTransform(V4(p), Quaternion.new(wq.i, wq.j, wq.k, wq.r))
            elseif ctx.ppos and ctx.playerQ then
              local lp, lq = CM.toLocal(ctx.ppos, ctx.playerQ, CM.v3(p.x, p.y, p.z), wq)
              tgt:SetLocalTransform(V4(lp), Quaternion.new(lq.i, lq.j, lq.k, lq.r))
            end
          end
          tgt:Activate(0, M.rt.audio)
          if vh.prio then tgt:SetIsHighPriority(true) end
        end)
      end
      if old ~= nil then
        pcall(function()
          if vh.prio then old:SetIsHighPriority(false) end
          old:Deactivate(0, false)
        end)
      end
      G.watch, G.misses = 0, 0
    end
    if to == "world" then
      if okA and live then M.wc.activations = M.wc.activations + 1; M.Log("world camera: active (" .. tostring(why) .. ")") end
    elseif from == "world" and live then
      M.Log(string.format("world camera: deactivated (%s)", tostring(why)))
    end
    M.Log(string.format("camera switch %s -> %s (%s)%s", from == "world" and "world camera" or "utppCamera",
      to == "world" and "world camera" or "utppCamera", tostring(why),
      live and (", Activate(0) " .. okStr(okA, errA)) or " - not shown now (activated on the next entry / take-over)"))
    return okA
  end
  -- failure: one line, the component path takes over, the entity is released
  -- permanent = true: a failure that cannot change within the session (no Codeware / no entity id, the entity space is
  -- not the world, a write failed): no retry. Otherwise (not rendering, spawn timeout, entity lost, Activate failed) the
  -- camera is spawned again after WC_FAIL_RETRY s (camera.worldCameraRetry).
  local function wcFail(why, ctx, permanent)
    M.PoseWriteTpp(false)
    local W = M.wc
    W.state, W.why, W.fails = "failed", tostring(why), W.fails + 1
    local retry = M.rt.worldCameraRetry ~= false and permanent ~= true and W.failRetries < M.WC_FAIL_MAX
    W.failAt, W.failRetryAt = G.clock, retry and (G.clock + M.WC_FAIL_RETRY) or nil -- retried by wcFrame
    M.Log("world camera: FAILED " .. tostring(why) .. " -> component camera"
      .. (retry and string.format(" (retry %d/%d in %.0f s)", W.failRetries + 1, M.WC_FAIL_MAX, M.WC_FAIL_RETRY) or ""))
    if G.use == "world" and ctx then switchCam("comp", "world camera failed", ctx) end
    if W.id ~= nil then
      local info = wcDespawn()
      W.releases = W.releases + 1
      M.Log(string.format("world camera: released (failed): %s", tostring(info)))
    end
    if G.use == "world" then G.use = nil end
  end
  M.WorldCameraFail = function(why) wcFail(why, nil, true) end -- console / tests: force the component path for this session
  -- one gameplay frame: spawn / resolve / space check / release. ctx = { comp, ppos, playerQ, freeze, dt }.
  -- Returns the world camera component when it is ready (resolved this frame), else nil.
  local function wcFrame(ctx)
    local W = M.wc
    local want = M.rt.worldCamera ~= false and M.Enabled()
    if not want then
      if W.id ~= nil then
        local why = M.rt.worldCamera == false and "camera.worldCamera off" or "UTPP camera off"
        if G.use == "world" and (CM.trActive(M.T) or M.T.state == "settle") then
          if ctx.comp == nil then return (wcComp()) end -- no utppCamera to take over: kept until this third person ends
          switchCam("comp", why, { comp = ctx.comp, wcomp = wcComp(), ppos = ctx.ppos, playerQ = ctx.playerQ })
        end
        wcRelease(why)
      end
      W.state = "off" -- turned on again later = a new attempt (also after a failure)
      W.firstTry, W.retryAt = nil, nil
      return nil
    end
    if W.state == "failed" then
      -- (camera.worldCameraRetry): a new spawn attempt WC_FAIL_RETRY s after the failure, WC_FAIL_MAX per session
      if M.rt.worldCameraRetry == false or ctx.freeze or W.failRetries >= M.WC_FAIL_MAX
         or G.clock < (W.failRetryAt or math.huge) then
        return nil
      end
      W.failRetries = W.failRetries + 1
      local why = W.why
      W.state, W.why, W.firstTry, W.retryAt, W.failRetryAt = "off", nil, nil, nil, nil
      M.Log(string.format("world camera: retry %d/%d - new spawn %.0f s after the failure (%s)%s", W.failRetries, M.WC_FAIL_MAX,
        G.clock - (W.failAt or G.clock), tostring(why),
        (G.lwState ~= "on" and M.cfg.lookWrite ~= false) and " - the game look write is off meanwhile (world camera not shown)" or ""))
    end
    if ctx.freeze then -- menu / photo mode: no spawn work, no failure on a missing entity
      if W.state == "ready" then return (wcComp()) end
      return nil
    end
    if W.state == "off" then
      if G.present < 1.5 or G.clock < (W.retryAt or -1) then return nil end
      W.firstTry = W.firstTry or G.clock
      local id, how, retry, route = wcSpawn()
      if id ~= nil then
        W.id, W.idStr, W.route, W.how, W.state, W.since, W.waited, W.check = id, idText(id), route, how, "spawning",
          G.clock, 0, nil
        W.spawns = W.spawns + 1
        M.Log(string.format("world camera: spawned %s entity=%s (template %s at the world origin, identity orientation: "
          .. "its local transform is the world pose)", how, W.idStr, M.WC_TEMPLATE))
      elseif retry and G.clock - W.firstTry < M.WC_READY_TIMEOUT then
        W.retryAt = G.clock + 0.5
      else
        wcFail(how, nil, not retry) -- no spawn route / no id: permanent; spawner never ready: retried
      end
      return nil
    end
    local wc, why = wcComp()
    if W.state == "spawning" then
      W.waited = W.waited + (ctx.dt or 0)
      if wc == nil then
        if W.waited > M.WC_SPAWN_TIMEOUT then
          wcFail(string.format("spawn timeout: entity %s not resolved after %.1f s (%s)", tostring(W.idStr), W.waited,
            tostring(why)), ctx)
        end
        return nil
      end
      W.state, W.check = "verify", nil
      M.Log(string.format("world camera: entity resolved after %.2f s (component utppCamera found)", W.waited))
    end
    if wc == nil then
      wcFail("entity lost (" .. tostring(why) .. ")", ctx)
      return nil
    end
    if W.state == "verify" then
      -- space check while the camera is not in use: a probe pose is written and read back through GetLocalToWorld
      local c = W.check
      if c == nil then
        local p = CM.add(ctx.ppos, CM.v3(0, 0, 2.5))
        local okW, errW = pcall(function() wc:SetLocalTransform(V4(p), Quaternion.new(0, 0, 0, 1)) end)
        if not okW then wcFail("probe write: " .. tostring(errW), ctx, true); return nil end
        W.check = { pos = p, t = 0 }
        return nil
      end
      c.t = c.t + (ctx.dt or 0)
      local okM, m = pcall(function() return wc:GetLocalToWorld() end)
      if not okM or m == nil then
        W.state = "ready"
        M.Log(string.format("world camera: ready (space check skipped: GetLocalToWorld %s) - the watchdog checks the "
          .. "rendered position", okM and "nil" or tostring(m)))
      else
        local best = math.huge
        for _, lay in ipairs({ "rows", "cols" }) do
          local okP, pos = pcall(CM.matrixPosition, m, lay)
          if okP and pos then best = math.min(best, CM.dist(pos, c.pos)) end
        end
        if best < 0.05 then
          W.state = "ready"
          M.Log(string.format("world camera: ready (space check ok: component world position = written local position, "
            .. "d=%.3f m after %.2f s)", best, c.t))
        elseif c.t > M.WC_CHECK_TIME then
          wcFail(string.format("space check: component %.2f m away from the written position after %.2f s (the entity "
            .. "space is not the world)", best, c.t), ctx, true)
          return nil
        end
      end
      if W.state ~= "ready" then return nil end
    end
    return wc
  end

  -- vehicle hold: component operations for the API above (utppCamera of the current player)
  local function vehComp()
    if G.comp == nil then
      local okP, player = pcall(function() return Game.GetPlayer() end)
      if okP and player ~= nil then pcall(components, player) end
    end
    return G.comp
  end
  -- the camera in use (world camera entity component, resolved now | utppCamera) -> comp, isWorld | nil, why
  local function curCam()
    if G.use == "world" then
      local c, why = wcComp()
      if c == nil then return nil, "world camera: " .. tostring(why) end
      return c, true
    end
    local comp = vehComp()
    if comp == nil then return nil, "utppCamera missing" end
    return comp, false
  end
  M.vehOps = {
    activate = function(b)
      local comp, why = curCam()
      if comp == nil then return false, why end
      comp:Activate(b or 0, M.rt.audio)
      G.watch, G.misses = 0, 0
      return true
    end,
    deactivate = function(b)
      local comp, why = curCam()
      if comp == nil then return false, why end
      comp:Deactivate(b or 0, false)
      return true
    end,
    prio = function(on)
      local comp, why = curCam()
      if comp == nil then return false, why end
      comp:SetIsHighPriority(on and true or false)
      return true
    end,
    -- the override pose for the player pose of this moment (also before an activation)
    writeNow = function()
      local comp, isWorld = curCam()
      local o = M.vh.override
      if comp == nil then return false, isWorld end
      if o == nil then return false, "no override" end
      if o.fov then pcall(function() comp:SetFOV(o.fov) end) end -- optional override FOV
      -- the exit framing pitch (VehicleCompose) belongs to the written pose as well, else the release starts one step away
      local pitch = o.pitch + (M.vh.frameOff or 0)
      if isWorld then -- world camera: the override is the world pose itself
        local wq = CM.qFromYawPitch(o.yaw, pitch)
        comp:SetLocalTransform(V4(o), Quaternion.new(wq.i, wq.j, wq.k, wq.r))
        M.vh.last = { x = o.x, y = o.y, z = o.z, yaw = o.yaw, pitch = pitch, fov = o.fov }
        return true
      end
      local player = Game.GetPlayer()
      local pp = vec(player:GetWorldPosition())
      local okQ, pq = pcall(function() return player:GetWorldOrientation() end)
      local q = (okQ and pq) and CM.q(pq.i, pq.j, pq.k, pq.r) or
        CM.qFromYawPitch(CM.yawPitchFromForward(vec(player:GetWorldForward())) or 0, 0)
      local lp, lq = CM.toLocal(pp, q, CM.v3(o.x, o.y, o.z), CM.qFromYawPitch(o.yaw, pitch))
      comp:SetLocalTransform(V4(lp), Quaternion.new(lq.i, lq.j, lq.k, lq.r))
      M.vh.last = { x = o.x, y = o.y, z = o.z, yaw = o.yaw, pitch = pitch, fov = o.fov }
      return true
    end,
    -- the player is mounted on a vehicle (TakeForVehicleExit)
    mounted = function()
      local player = Game.GetPlayer()
      if player == nil then return false, "no player" end
      if Game.GetMountedVehicle(player) == nil then return false, "no mounted vehicle" end
      return true
    end,
    -- the camera for an exit take-over from the first person state: the world camera when it
    -- is ready (and enabled), else utppCamera
    useCam = function()
      local wc = (M.rt.worldCamera ~= false and M.wc.state == "ready") and wcComp() or nil
      G.use = wc ~= nil and "world" or "comp"
      if G.use == "comp" and vehComp() == nil then G.use = nil; return false, "utppCamera missing" end
      G.watch, G.misses = 0, 0
      return true
    end,
  }

  -- pose nudge: the vanilla stand-entry animation event (StandEvents.OnEnter pushes "StandEnter"), only standing still on
  -- the ground (PSM LocomotionDetailed Stand = 1) while the third-person body is up
  local function nudgePose(player, speed, why)
    local p = psm(player)
    local skip = nil
    if not (CM.trActive(M.T) or M.T.state == "rep_wait") then skip = "not in third person"
    elseif (p.loco or 0) ~= 1 then skip = "PSM locomotion " .. tostring(p.loco) .. " is not stand"
    elseif (speed or 0) > 0.5 then skip = string.format("moving %.2f m/s", speed or 0)
    elseif (p.vehicle or 0) ~= 0 or (p.workspot or 0) ~= 0 or ((p.high or 0) >= 3 and (p.high or 0) <= 6) then skip = "vehicle / workspot / scene / swim" end
    if skip then
      M.Log(string.format("pose nudge (%s) skipped: %s", tostring(why), skip))
      return false
    end
    local ok, err = pcall(function() AnimationControllerComponent.PushEvent(player, "StandEnter") end)
    G.nudgeCount = (G.nudgeCount or 0) + (ok and 1 or 0)
    M.Log(string.format("pose nudge (%s): StandEnter %s", tostring(why), ok and "pushed" or ("FAILED " .. tostring(err))))
    return ok
  end
  function M.NudgePose()
    local okP, player = pcall(function() return Game.GetPlayer() end)
    if not okP or player == nil then return false end
    local v = player:GetVelocity()
    return nudgePose(player, math.sqrt((v.x or 0) ^ 2 + (v.y or 0) ^ 2), "console")
  end

  -- ctx (may be nil) = { comp, wcomp (world camera ready this frame), ppos, playerQ }
  local function execute(cmds, player, comp, fpp, firstFrame, ctx)
    for _, c in ipairs(cmds) do
      if c == "fpp_on" or c == "cam_off" or c == "rep_off" then M.PoseWriteTpp(false) end
      M.stats.cmds[c] = (M.stats.cmds[c] or 0) + 1
      if c == "rep_on" then
        if M.rt.body and M.rt.poseNudge then
          G.nudges = { { t = G.clock + 0.4, why = "after full body" }, { t = G.clock + 0.9, why = "after full body" } }
        end
        if M.rt.body then
          local B = M.BodyModule()
          if B then
            local okB, acc, why = pcall(B.SetTppBody, true, "utpp_camera")
            M.Log(string.format("full body via utpp_body: %s", (okB and acc) and "requested" or ("refused " .. tostring(why or acc))))
            if okB and acc then G.repOwned = "utpp_body" end
          elseif representation(player, true) then
            G.repOwned = true
          end
        end
      elseif c == "cam_on" then
        -- the camera for this third person: the world camera when it is ready, else utppCamera
        local wcomp = ctx and ctx.wcomp or nil
        if wcomp ~= nil then
          G.use = "world"
          if firstFrame then
            pcall(function()
              wcomp:SetLocalTransform(V4(firstFrame.world), Quaternion.new(firstFrame.q.i, firstFrame.q.j, firstFrame.q.k,
                firstFrame.q.r))
            end)
          end
          local ok, err = pcall(function() wcomp:Activate(0, M.rt.audio) end)
          M.Log(string.format("enter epoch=%d: world camera Activate %s", M.T.epoch, ok and "ok" or ("FAILED " .. tostring(err))))
          if ok then
            M.wc.activations = M.wc.activations + 1
            M.wc.reacts = 0
            M.Log(string.format("world camera: active (enter epoch=%d, entity=%s)", M.T.epoch, tostring(M.wc.idStr)))
          else
            G.use = "comp" -- not shown yet: utppCamera is activated below at the same first pose
            wcFail("Activate: " .. tostring(err), nil)
            wcomp = nil
          end
        end
        if wcomp == nil then
          G.use = "comp"
          if firstFrame then
            pcall(function()
              comp:SetLocalTransform(V4(firstFrame.localPos), Quaternion.new(firstFrame.localQ.i, firstFrame.localQ.j,
                firstFrame.localQ.k, firstFrame.localQ.r))
            end)
          end
          local ok, err = pcall(function() comp:Activate(0, M.rt.audio) end)
          M.Log(string.format("enter epoch=%d: utppCamera.Activate %s", M.T.epoch, ok and "ok" or ("FAILED " .. tostring(err))))
        end
        G.watch, G.misses = 0, 0
      elseif c == "fpp_on" then
        setHeadingLock(fpp, false)
        -- safety net - while the game's vehicle camera renders in third person for the driver,
        -- the FPP camera is not activated (cam_off / rep_off still run: the vehicle camera keeps the view)
        local guard, gwhy = false, nil
        if M.cfg.vehicleFppGuard ~= false then guard, gwhy = vehicleTppGuard(player) end
        if guard then
          M.stats.fppGuards = (M.stats.fppGuards or 0) + 1
          if G.fppGuardEpoch ~= M.T.epoch then
            G.fppGuardEpoch = M.T.epoch
            M.Log(string.format("exit epoch=%d: vehicle TPP camera renders (%s) - no FPP camera activation", M.T.epoch,
              tostring(gwhy)))
          end
        else
          local ok, err = pcall(function() fpp:Activate(0, M.rt.audio) end)
          M.Log(string.format("exit epoch=%d (%s): camera.Activate %s", M.T.epoch, tostring(M.T.blocked or M.T.last),
            ok and "ok" or ("FAILED " .. tostring(err))))
        end
      elseif c == "cam_drop" then
        -- the camera was deactivated with an engine blend (HoldForVehicle(false, .., "game")):
        -- forgotten without a second Deactivate (it would cut the blend)
        G.use = nil
      elseif c == "rep_off" then
        G.nudges = nil
        if G.repOwned == "utpp_body" then
          pcall(M.bodyMod.SetTppBody, false, "utpp_camera") -- utpp_body keeps game-owned representations itself
          G.repOwned = false
        elseif G.repOwned then
          if vehicleTppActive(player) then
            M.Log("TPP representation left to the vehicle camera (game owns it)")
          else
            representation(player, false)
          end
          G.repOwned = false
        end
      elseif c == "cam_off" then
        if G.use == "world" then
          local wc = (ctx and ctx.wcomp) or wcComp()
          local ok, err = pcall(function() wc:Deactivate(0, false) end)
          M.Log(string.format("world camera: deactivated (%s)%s", tostring(M.T.blocked or M.T.last or "third person ended"),
            ok and "" or (" - Deactivate FAILED " .. tostring(err))))
        else
          pcall(function() comp:Deactivate(0, false) end)
        end
        G.use = nil
      end
    end
  end

  -- drop every cached game object (camera components, player). Plain data (G.aimWeapon, G.last, ...) stays.
  local function dropHandles()
    G.comp, G.fpp, G.player = nil, nil, nil
  end
  M.DropHandles = dropHandles

  local function resetSession(why)
    M.FeedCameraTarget(nil)
    if G.sessionStarted then M.Log("session reset: " .. tostring(why)) end
    local want = M.T.want
    M.T = CM.newTransition(M.rt.body)
    M.T.want = want
    M.S = CM.newSolver(M.cfg)
    G.comp, G.fpp, G.playerKey, G.repOwned, G.lockSet, G.present, G.prevBodyYaw = nil, nil, nil, false, false, 0, nil
    G.sf, G.sfOut, G.stanceFast, G.sfN = nil, nil, false, nil -- the new solver's pivot starts on the current stance, so does its stance filter
    G.mshMounted, G.mshN, G.mshSince, G.mshPrev, G.mshLastStep = nil, nil, nil, nil, nil -- mount pivot continuity (new solver: no blend)
    G.nudges, G.relock, G.lockHold, G.lockOwned, G.lockSetName, G.lockSetSeen = nil, nil, nil, nil, nil, nil -- the heading lock re-assert starts again
    G.ragEp, G.ragPhase, G.ragNaAt, G.ragBoom, G.ragN = nil, nil, nil, nil, nil -- ragdoll camera (the new solver has no episode)
    G.ls, G.lookUsed = CM.newLookSync(), false -- the new session's FPP camera starts on the game's set
    -- game look write: the native target expires by itself (0.25 s); a stop is sent while the world is alive
    if G.lwState == "on" and not G.shuttingDown then pcall(function() Game.UTPPStopGameLook() end) end
    G.lwState, G.lwSent, G.lwErr, G.lwPErr, G.lwTarget = "off", nil, nil, nil, nil
    G.lwRamp = nil
    G.lwp, G.lwpDist, G.lwpAt = nil, nil, nil -- parallax look target (crosshair point distance)
    G.lwPar, G.lwFlush = nil, nil -- parallax correction state, release flush frames
    wcForget("session reset: " .. tostring(why)) -- no game call (the world is going away)
    M.AimReset("session reset: " .. tostring(why))
    M.AimOccReset("session reset: " .. tostring(why), G.clock)
    M.ZoomUnarmedReset("session reset: " .. tostring(why))
    dropHandles()
    local vh = M.vh
    if vh.held then M.Log("vehicle hold dropped (session reset: " .. tostring(why) .. ")") end
    vh.held, vh.handed, vh.prio, vh.override, vh.release, vh.last, vh.pending = false, false, false, nil, nil, nil, nil
    vh.onFoot = false
    vh.endedAt, vh.aimSoftAt, vh.frame, vh.frameOff, vh.composeAt, vh.frameVel = nil, nil, nil, 0, nil, nil
    G.lwOffAt, G.ownPitchWhy, G.ownPitchLoud = nil, nil, false
    M.SignalTpp(false, "session reset: " .. tostring(why), G.clock)
    pcall(M.InsideHideRelease, "session reset: " .. tostring(why)) -- (C2)
  end

  function M.Diag(seconds)
    G.diagUntil = G.clock + (tonumber(seconds) or 5)
    M.Log(string.format("diag for %.1f s", tonumber(seconds) or 5))
  end

  registerForEvent("onInit", function()
    M.PoseWriteTpp(false)
    G.shuttingDown = false
    -- parallel UTPP modules (optional): perspective decisions and full-body layers
    local okPm, pm = pcall(function() return GetMod and GetMod("utpp_perspective") end)
    if okPm and type(pm) == "table" and type(pm.Desired) == "function" then M.persp = pm end
    -- a first person request of an earlier camera instance (CET reload during the handoff) is cleared
    if M.persp and type(M.persp.RequestFppZoom) == "function" and not M.zuIn.requested then
      pcall(M.persp.RequestFppZoom, false, "utpp_camera onInit (clears a stale unarmed zoom request)")
    end
    local okBm, bm = pcall(function() return GetMod and GetMod("utpp_body") end)
    if okBm and type(bm) == "table" and type(bm.SetTppBody) == "function" then M.bodyMod = bm end
    local okC, core = pcall(function() return GetMod and GetMod("utpp_core") end)
    if okC and type(core) == "table" and type(core.Register) == "function" then
      local ok, err = pcall(M.RegisterSettings, core)
      if not ok then M.Log("settings registration FAILED: " .. tostring(err)) end
    else
      M.Log("utpp_core not found - defaults only")
    end
    local observe = Observe or ObserveAfter
    local okO, errO = pcall(observe, "PlayerPuppet", "OnAction", function(_, action) onAction(action) end)
    M.Log(string.format("onInit %s: camera %s, input observer %s, utpp_perspective %s, utpp_body %s", M.VERSION,
      M.Enabled() and "on" or "off (setting)",
      okO and "ok" or ("FAILED " .. tostring(errO)), M.persp and "found" or "absent", M.bodyMod and "found" or "absent"))
  end)

  registerForEvent("onShutdown", function()
    M.PoseWriteTpp(false)
    if M.tppSig.sent ~= false then
      M.tppSig.retryAt = -1
      pcall(M.SignalTpp, false, "shutdown", G.clock)
    end
    -- game exit: the player is already gone, cached handles (G.comp / G.fpp / G.player) point at dead objects -> no method calls
    local okP, p = pcall(Game.GetPlayer)
    local alive = okP and p ~= nil and (pcall(function() return p:GetEntityID() end))
    G.shuttingDown = true
    if not alive then
      M.Log("[utpp_camera] shutdown: player gone - game calls skipped")
      pcall(resetSession, "shutdown")
      dropHandles()
      pcall(collectgarbage, "collect")
      return
    end
    pcall(M.ZoomUnarmedReset, "shutdown") -- no first person request outlives the camera
    pcall(M.InsideHideRelease, "shutdown") -- (C2)
    if M.vh.prio then
      if G.use == "world" then
        local wc = wcComp() -- resolved now from the live world (player alive)
        if wc then pcall(function() wc:SetIsHighPriority(false) end) end
      elseif G.comp then pcall(function() G.comp:SetIsHighPriority(false) end) end
    end
    if CM.trActive(M.T) or M.T.state == "settle" then
      pcall(function()
        if G.fpp then G.fpp:Activate(0, M.rt.audio) end
        if G.comp then G.comp:Deactivate(0, false) end
        if G.repOwned == "utpp_body" then pcall(M.bodyMod.SetTppBody, false, "utpp_camera shutdown")
        elseif G.repOwned and G.player then representation(G.player, false) end
        setHeadingLock(G.fpp, false)
      end)
    end
    -- the game's look input applies again at once
    if G.lwState == "on" then pcall(function() Game.UTPPStopGameLook() end); G.lwState = "off" end
    -- the world camera entity is deactivated (when in use) and despawned while the world is alive
    pcall(wcRelease, "shutdown")
    G.use = nil
    -- the world is going away: no cached object may outlive the engine's teardown
    dropHandles()
    pcall(collectgarbage, "collect")
  end)

  registerForEvent("onUpdate", function(delta)
    if G.shuttingDown then return end -- after onShutdown nothing is cached or called any more
    local dt = math.max(tonumber(delta) or 0, 0)
    G.clock = G.clock + dt
    G.updN = (G.updN or 0) + 1 -- update counter: the stance filter below must have run in the PREVIOUS update to count as continuous
    local okP, player = pcall(function() return Game.GetPlayer() end)
    if not okP or player == nil then
      M.PoseWriteTpp(false)
      dropHandles()
      if G.present > 0 then resetSession("no player")
      elseif M.tppSig.sent == true then M.SignalTpp(false, "no player", G.clock) end
      G.present = 0
      IN.mouseX, IN.mouseY = 0, 0
      M.zuIn.wheel = 0 -- no wheel step survives a frame without a player
      return
    end
    if G.present == 0 and not G.sessionStarted then
      G.sessionStarted = true
      if M.rt.startTpp and M.Enabled() and not M.Perspective() then CM.trRequest(M.T, true, "start in TPP") end
    end
    G.present = G.present + dt
    G.player = player
    local ok, err = pcall(function()
      local comp, fpp = components(player)
      local camSys = Game.GetCameraSystem()
      local menu, photo = inMenu(), photoMode()
      shoulderInputPoll(player, menu, photo) -- shoulder-swap input (key / pad requests from utpp_shoulder_input.reds)
      local freeze = menu or photo
      if freeze then M.PoseWriteTpp(false) end
      -- game menu closed (pause / inventory / map) -> pose nudge 0.1 s and 0.5 s later (T-pose for a moment)
      if menu then G.menuSince = G.menuSince or G.clock
      elseif G.menuSince ~= nil then
        local dur = G.clock - G.menuSince
        G.menuSince = nil
        if M.rt.body and M.rt.poseNudge and CM.trActive(M.T) and G.repOwned then
          local list = G.nudges or {}
          list[#list + 1] = { t = G.clock + 0.1, why = "after menu close" }
          list[#list + 1] = { t = G.clock + 0.5, why = "after menu close" }
          table.sort(list, function(a, b) return a.t < b.t end)
          G.nudges = list
          M.stats.menuNudges = M.stats.menuNudges + 1
          M.Log(string.format("menu closed after %.2f s: pose nudge scheduled +0.10 s / +0.50 s (camera.poseNudge)", dur))
        end
      end
      -- blocked reasons (5N step 4 owns the full rule set; MVP: cuts that keep the game consistent)
      local blocked, cut = nil, true
      local P = M.Perspective()
      local vh = M.vh
      vh.clock = G.clock
      if vh.pending then -- hold handed to the game's vehicle camera (rep_off / cam_off, no FPP activation)
        local pend = vh.pending
        vh.pending = nil
        execute(pend, player, comp, fpp, nil)
      end
      if vh.held and not M.Enabled() then M.HoldForVehicle(false, "UTPP camera switched off") end
      if P then
        local okD, d = pcall(P.Desired)
        if okD and type(d) == "table" then
          local want = d.tpp == true or d.perspective == "TPP"
          if vh.held then
            -- utpp_vehiclecam drives the camera; the decision (FPP for the vehicle) is ignored, logged per change
            if want ~= vh.lastWant then
              local prevWant = vh.lastWant
              vh.lastWant = want
              if not want then
                vh.ignored = vh.ignored + 1
                M.Log(string.format("vehicle hold: decision FPP (%s) ignored - utpp_vehiclecam drives utppCamera", tostring(d.reason)))
              elseif prevWant == false then
                -- "TPP <- user" during an exit take-over / after the vehicle FPP decision: no
                -- second enter (the camera is already third person; no FPP -> TPP blend from the eyes)
                vh.ignored = vh.ignored + 1
                M.Log(string.format("vehicle hold: decision TPP (%s) ignored - already third person, utpp_vehiclecam drives "
                  .. "utppCamera", tostring(d.reason)))
              end
            end
          else
            if not want and M.T.want and string.find(tostring(d.reason), "vehicle", 1, true) then
              M.AskVehicleDelegate("utpp_perspective: " .. tostring(d.reason))
            end
            -- aim return: the last first-person decision came from the aiming rule
            -- (also after the unarmed zoom handoff, rule "zoom")
            if want and not M.T.want and not vh.held and G.lastFpp and (G.lastFpp.rule == "weapon" or G.lastFpp.rule == "zoom"
                or string.find(tostring(G.lastFpp.detail), "aim", 1, true)) then
              M.AimReturn(string.format("utpp_perspective %s: %s", tostring(G.lastFpp.rule or G.lastFpp.reason),
                tostring(G.lastFpp.detail or G.lastFpp.reason)))
            end
            if not vh.held and want ~= M.T.want then CM.trRequest(M.T, want, "utpp_perspective: " .. tostring(d.reason)) end
            if not want then G.lastFpp = { rule = d.rule, reason = d.reason, detail = d.detail } end
          end
          cut = d.style == "instant" or d.style == "combat"
        end
      end
      if G.present < 1.5 then blocked, cut = "loading", true
      elseif P then
        -- utpp_perspective already applies the vehicle / scene / weapon ... rules to its decision
      elseif not freeze then
        local p = psm(player)
        if (p.vehicle or 0) ~= 0 then blocked = "vehicle"
        elseif (p.high or 0) >= 3 and (p.high or 0) <= 5 then blocked = "scene"
        elseif (p.workspot or 0) ~= 0 then blocked = "workspot"
        elseif p.device then blocked = "device"
        elseif bbBool("UI_PlayerStats", "isReplacer") then blocked = "replacer"
        elseif bbBool("Braindance", "IsActive") then blocked = "braindance"
        elseif M.rt.fppWithWeapon and weaponDrawn(player) then blocked = "weapon" end
        if blocked == "vehicle" and not vh.held and M.T.want then M.AskVehicleDelegate("vehicle (own rules)") end
      end
      -- unarmed zoom handoff without utpp_perspective: own block rule (cut to first person)
      if not P and blocked == nil and M.zuIn.requested then blocked, cut = "unarmed zoom", true end
      if vh.held and blocked ~= "loading" then blocked, cut = nil, false end -- no FPP rule while held
      -- aim return (own rules): the weapon block ends while third person is wanted (also the zoom handoff)
      if not P and (G.prevBlocked == "weapon" or G.prevBlocked == "unarmed zoom") and blocked == nil and M.T.want
          and not freeze then
        M.AimReturn(G.prevBlocked == "weapon" and "own rule: weapon put away" or "own rule: unarmed zoom released")
      end
      if not freeze then G.prevBlocked = blocked end
      -- player and FPP view
      local ppos = vec(player:GetWorldPosition())
      -- V's position and the mount state for the framing band of the hold (M.VehicleCompose)
      if vh.held then
        local okV, veh = pcall(function() return Game.GetMountedVehicle(player) end)
        vh.frame = { ppos = ppos, unmounted = okV and veh == nil }
      else
        vh.frame = nil
      end
      local okQ, pq = pcall(function() return player:GetWorldOrientation() end)
      local playerQ = (okQ and pq) and CM.q(pq.i, pq.j, pq.k, pq.r) or nil
      local f = player:GetWorldForward()
      local bodyYaw = CM.yawPitchFromForward(vec(f)) or 0
      if playerQ == nil then playerQ = CM.qFromYawPitch(bodyYaw, 0) end
      -- world camera: spawn / resolve / space check / release; wcomp = its component when ready (this frame)
      local wctx = { comp = comp, ppos = ppos, playerQ = playerQ, freeze = freeze, dt = dt }
      local wcomp = wcFrame(wctx)
      wctx.wcomp = wcomp
      -- Shooter mode (utpp_bodyturn.ShooterModeActive, cached module lookup): the camera always follows V's capsule - no
      -- idle orbit, no Locomotion decoupling (and no decouple hold)
      G.shooterMode = false
      do
        local sbt = btModule()
        if sbt ~= nil and type(sbt.ShooterModeActive) == "function" then
          local okF, vF = pcall(sbt.ShooterModeActive)
          G.shooterMode = okF and vF == true
        end
      end
      -- Locomotion (body turns toward the movement) -> the camera keeps its own world yaw (camera_math lookStep
      -- "decoupled"); read per frame, one log line per change
      do
        local okA, act = pcall(Game.UTPPLocomotionActive)
        act = okA and act == true
        if act ~= (G.freeMoveActive == true) then
          G.freeMoveActive = act
          M.Log(act and "follow: UTPP Locomotion active -> camera decoupled from the body yaw (mouse / stick only)"
            or string.format("follow: UTPP Locomotion inactive -> camera follows the body yaw again (after %.2f s without "
              .. "Locomotion; aiming / unarmed zoom align at once)", M.cfg.decoupleHold or 0.35))
        end
        -- flickers (menu close, a UpperBody blip: three toggles within 0.1 s) keep the free
        -- camera; only Locomotion off for decoupleHold s switches to the follow mode
        if act then G.freeMoveSeen = G.clock end
        G.freeMoveHold = act or (G.freeMoveSeen ~= nil and G.clock - G.freeMoveSeen < (M.cfg.decoupleHold or 0.35))
      end
      local view = fppView(fpp, camSys)
      -- (c9 E3) unmount yaw diagnostic, read only: after the vehicle edge the game turns V (feed yawDeg 10.9 ->
      -- -8.3 in 0.2 s) onto its FPP camera heading (feed camDeg -8.35, held at the vehicle heading). One line at the first
      -- unmounted frame of a hold and one 0.4 s later: entity yaw, the yaw shown by this camera, the FPP view yaw / pitch
      -- and the FPP heading lock now and on the last mounted frame (the game look write needs the lock).
      if vh.held and vh.frame then
        local okL, lk = pcall(function() return fpp and fpp.headingLocked end)
        lk = okL and tostring(lk) or "unreadable" -- true / false / nil (no FPP component) / unreadable
        local function viewStr() return view and string.format("%.1f/%.1f", view.yaw, view.pitch) or "-" end
        if not vh.frame.unmounted then
          G.vuYaw, G.vuLock, G.vuAt = bodyYaw, lk, nil
        elseif G.vuAt == nil and G.vuYaw ~= nil then -- a mounted frame was seen in this hold
          G.vuAt, G.vuEdgeYaw = G.clock, bodyYaw
          M.Log(string.format("vehicle unmount yaw: entity %.1f (last mounted frame %s), shown camera %s, FPP view %s, "
            .. "heading lock %s (last mounted frame %s)", bodyYaw, G.vuYaw and string.format("%.1f", G.vuYaw) or "-",
            vh.last and string.format("%.1f", vh.last.yaw or 0) or "-", viewStr(), tostring(lk), tostring(G.vuLock)))
        elseif type(G.vuAt) == "number" and G.clock - G.vuAt >= 0.4 then
          M.Log(string.format("vehicle unmount yaw +%.2f s: entity %.1f (%.1f deg since the edge), shown camera %s, FPP view "
            .. "%s, heading lock %s", G.clock - G.vuAt, bodyYaw, CM.wrapDeg(bodyYaw - (G.vuEdgeYaw or bodyYaw)),
            vh.last and string.format("%.1f", vh.last.yaw or 0) or "-", viewStr(), tostring(lk)))
          G.vuAt = false
        end
      elseif not vh.held then
        G.vuAt, G.vuYaw, G.vuLock = nil, nil, nil
      end
      -- game look write read-back: the game's look now (FPP camera world yaw / pitch) minus the target written
      -- last frame (deg; ~0 when the game applied it); nil when nothing was written last frame
      G.lwErr, G.lwPErr = nil, nil
      -- while written the game's look is our own target of the last frame (echo: the native replaces
      -- the game's mouse delta) - the solve gets the camera-equivalent look (read back minus the parallax correction
      -- written) and lookEcho (CM.lookStep: the aim pitch integrates the mouse itself; no feedback of the correction)
      local lwEcho, viewYaw, viewPitch = false, view and view.yaw or nil, view and view.pitch or nil
      -- the target written last frame (release flush, AimOffset feed); nothing written = the parallax
      -- correction starts from 0 again (no stale correction after a pause of the write)
      local lwPrev = G.lwSent
      if not lwPrev and not freeze then G.lwPar = nil end
      if G.lwSent and view and not freeze then
        G.lwErr, G.lwPErr = CM.wrapDeg(view.yaw - G.lwSent.yaw), view.pitch - G.lwSent.pitch
        lwEcho = true
        viewYaw = CM.wrapDeg(view.yaw - CM.wrapDeg(G.lwSent.yaw - G.lwSent.camYaw))
        viewPitch = view.pitch - (G.lwSent.pitch - G.lwSent.camPitch)
      end
      if not freeze then G.lwSent, G.lwTarget, G.lwp = nil, nil, nil end
      -- trace: due this frame (one line per 0.1 s in third person); camErr = yaw of the camera that
      -- rendered the last frame (active camera now) minus the yaw written last frame (turn of V carried into the camera)
      local wroteYaw = G.wroteYaw
      G.wroteYaw = nil
      G.trDue = M.rt.trace == true and not freeze and CM.trActive(M.T) and G.clock >= (G.trNext or 0) - 1e-6
      G.trErr = nil
      if G.trDue and wroteYaw ~= nil and camSys then
        local okF, af = pcall(function() return camSys:GetActiveCameraForward() end)
        if okF and af then
          local ay = CM.yawPitchFromForward(vec(af))
          if ay then G.trErr = CM.wrapDeg(ay - wroteYaw) end
        end
      end
      if M.aimReturnPending then
        local why = M.aimReturnPending
        M.aimReturnPending = nil
        M.AimReturn(why, bodyYaw)
      end
      local vel = player:GetVelocity()
      local speed = math.sqrt((vel.x or 0) ^ 2 + (vel.y or 0) ^ 2)
      local moving = speed > M.cfg.idleSpeed or math.abs(IN.moveX) > 0.15 or math.abs(IN.moveY) > 0.15
      for _, h in pairs(IN.held) do if h then moving = true end end
      -- mouse calibration from the game's own body turn (only while V follows the mouse)
      local bodyDelta = G.prevBodyYaw and CM.wrapDeg(bodyYaw - G.prevBodyYaw) or 0
      G.prevBodyYaw = bodyYaw
      -- an adjustTransform turn of V landed / lands around this frame (the game's look is overridden)
      if not freeze then adjustPendingStep() end
      -- the game's look can be written this frame (camera.lookWrite on, the native live - not unavailable /
      -- refused / tripped -, the world camera shown in third person, no vehicle hold): the camera then never reads the
      -- game's look back (own aim / zoom pitch and yaw, no look sync yaw); the write itself needs the heading lock (free
      -- look; aiming / zooming with the own aim yaw or the align hold - setHeadingLock below)
      local lwPossible = M.cfg.lookWrite ~= false and not G.lwUnavailable and not G.lwTripped and G.lwState ~= "fail"
        and G.use == "world" and M.T.state == "tpp" and not vh.held
      G.lwOwn = lwPossible and (M.cfg.aimOwnYaw ~= false or M.rt.alignHold == true)
      -- game look sync: evidence (look moved by the mouse while V stood / kept its world heading while
      -- V turned; skipped for 0.2 s after a send - the set applies in a later game tick) and the look used this frame
      local lookYaw = nil
      do
        local ls = G.ls
        local lookDelta = (view and ls.prevLook) and CM.wrapDeg(view.yaw - ls.prevLook) or nil
        ls.prevLook = view and view.yaw or nil
        if M.cfg.lookSync ~= false and ls.sent and ls.route and not freeze then
          local okL, locked = pcall(function() return fpp.headingLocked end)
          local ev = CM.lookSyncSample(ls, M.cfg, {
            want = G.lockSet == true and G.clock - (ls.sentAt or -1e9) >= 0.2, locked = okL and locked == true,
            mouseX = IN.mouseX, pad = math.abs(IN.padX) > 0.15 or math.abs(IN.padY) > 0.15,
            bodyDelta = bodyDelta, lookDelta = lookDelta })
          if ev == "ok" then
            M.Log(string.format("game look sync confirmed (look moved with the mouse while V stood: %d, kept its world "
              .. "heading while V turned: %d) -> the camera shows the game's look while free / aiming", ls.confirm,
              ls.maintained))
          elseif ev == "resend" then
            lookSyncResend()
          elseif ev == "broken" then
            M.Log(string.format("game look sync FAILED: the game's look still turned with V after %d resends of %s -> "
              .. "own camera yaw (mouse unit) for this session; setting camera.lookSync", ls.resends,
              tostring(M.cfg.lookSyncSet)))
          end
        end
        local used = M.cfg.lookSync ~= false and ls.sent == true and ls.route ~= nil and ls.ok and not ls.broken
          and G.lockSet == true and view ~= nil
        if used ~= G.lookUsed then
          G.lookUsed = used
          if M.rt.trace == true then M.Log("game look sync -> " .. (used and "camera = game look" or "own camera yaw")) end
        end
        -- not while the game look write is on (the look follows the camera; following it back would freeze it)
        -- (nor while it can be written this frame)
        if used and G.lwState ~= "on" and not lwPossible then lookYaw = view.yaw end
      end
      -- no samples while the camera is free / aiming / zooming, nor within calibUnlockHold s after the
      -- heading lock ended (the game then turns V onto its camera heading, not by the mouse: such samples inflated the
      -- mouse scale = over-sensitive free orbit); outliers rejected once calibrated (camera cfg calibBand)
      local noCal = (G.clock - (G.unlockAt or -1e9)) < (M.cfg.calibUnlockHold or 0.6)
        or (CM.trActive(M.T) and (M.S.look.free or M.aim.on or M.zoomU.on))
        or G.lwFlush ~= nil -- release flush (heading still locked, V does not follow the mouse)
        or G.ragPhase ~= nil -- ragdoll camera: the body tumbles with no mouse input
      if not freeze and not blocked and not vh.held and not noCal then
        CM.calibrate(M.cal, IN.mouseX, bodyDelta, M.cfg.calibBand, M.cfg.calibStreak)
      end
      -- FOV of the FPP view as rendered (transition start / end value), sampled while the FPP camera is active
      if M.T.state == "fpp" and not freeze then
        local okA, fov = pcall(function() return camSys:GetActiveCameraFOV() end)
        if okA and tonumber(fov) and tonumber(fov) > 1 then G.fppFov = tonumber(fov) end
      end
      -- state machine
      local camOK = comp ~= nil or wcomp ~= nil -- utppCamera or the world camera
      local env = { enabled = M.Enabled(), blocked = blocked, cut = cut, freeze = freeze, canEnter = camOK,
                    repReady = true }
      if M.T.want and not camOK and M.T.state == "fpp" then M.LogOnce("nocomp", "TPP requested but utppCamera is missing") end
      local cmds = CM.trStep(M.T, M.cfg, dt, env)
      local active = CM.trActive(M.T) or M.T.state == "settle"
      -- aim state: after the perspective state machine (third person of this frame known), before the solve. The
      -- vehicle hold / pose override win (CM.aimWanted refuses), a menu / photo mode freezes it; the PSM / weapon reads
      -- only when the cheap conditions hold.
      if not freeze then
        local T = M.T
        local ao = { enabled = M.cfg.aimEnabled, aimingSetting = M.AimingSetting(), vehicleHeld = vh.held,
                     poseOverride = vh.override ~= nil,
                     tpp = M.Enabled() and camOK and CM.trActive(T) and T.want and T.state ~= "leaving" }
        if ao.enabled ~= false and ao.tpp and not ao.vehicleHeld and (ao.aimingSetting == "tpp" or ao.aimingSetting == false) then
          aimObserve(player, ao)
        end
        M.AimUpdate(ao, dt)
        -- unarmed zoom: read while third person (entry) or while the state is on (also its first person handoff)
        local zo = { vehicleHeld = vh.held, poseOverride = vh.override ~= nil, tpp = ao.tpp }
        if M.cfg.zoomUnarmedEnabled ~= false and not vh.held and (zo.tpp or M.zoomU.on) then zoomObserve(player, zo) end
        M.ZoomUnarmedUpdate(zo, dt)
      end
      local A = M.aim
      local ZU = M.zoomU
      local res = nil
      local rayFn = nil
      if (active or #cmds > 0) and camOK and not freeze then
        local unit = CM.mouseUnit(M.cal, M.cfg, M.rt.mouseScale)
        local p = CM.aimCompose(CM.blendStep(M.blend, dt), A, M.cfg, M.aimFov) -- aim pose over the live profile
        p = CM.zoomUnarmedCompose(p, ZU) -- unarmed zoom step pose (never together with the weapon aim)
        local poseW = math.max(A.w, ZU.w)
        M.cfg.sens = (M.rt.sens or 1) * (p.sens or 1)
        local dyaw, dpitch = CM.inputDelta(IN, M.cfg, dt, unit)
        if M.rt.trace == true then G.trDyaw, G.trDyawSum = dyaw, (G.trDyawSum or 0) + dyaw end -- trace
        -- FPP pitch frozen while our camera is active -> own pitch (logged once). Not while the game's look is
        -- written (its pitch is our own target then - no evidence about the game's mouse pitch)
        if M.cfg.pitchSource == "fpp" and view and M.T.state == "tpp" and not M.S.look.free and G.lwState ~= "on" then
          if G.lastPitch and math.abs(view.pitch - G.lastPitch) < 1e-3 and math.abs(IN.mouseY) > 0 then
            G.frozen = G.frozen + math.abs(IN.mouseY)
            if G.frozen > 300 then
              M.cfg.pitchSource = "input"
              M.Log("FPP camera pitch does not follow the mouse while utppCamera is active -> own pitch (camera.pitchSource=input)")
            end
          elseif G.lastPitch and math.abs(view.pitch - G.lastPitch) >= 1e-3 then
            G.frozen = 0
          end
        end
        G.lastPitch = view and view.pitch or nil
        local okRay, sqs = pcall(function() return Game.GetSpatialQueriesSystem() end)
        -- capsule pivot: stance from the PSM (one log line per change)
        local crouch, loco, det, slide = false, 0, 0, false
        if M.cfg.pivotSource ~= "fpp" then
          crouch, loco, det, slide = crouchState(player)
          -- the same stance filter as the DLL (CM.stanceFilterStep): the PSM flicker Crouch 3 <-> CrouchSprint 30
          -- does not pump the pivot, a clean crouch / stand-up passes at once; a clean one at standstill takes the body's clip timing
          local rawCrouch = crouch
          local bypassed
          -- the filter only sees frames the solve runs on: after FPP / a menu / a session reset the state is stale (the first frame back
          -- would read a crouch that began outside as a fresh edge: the pivot waited crouchDelay and blended 0.4 s for a V that already
          -- crouched) -> a new filter, no clip timing; the pivot blends 0.25 s as before
          if G.sf == nil or G.sfN ~= (G.updN or 0) - 1 then G.sf, G.sfOut, G.stanceFast = CM.newStanceFilter(), nil, false end
          G.sfN = G.updN
          crouch, bypassed = CM.stanceFilterStep(G.sf, M.cfg, rawCrouch, det, speed, dt)
          if crouch ~= G.sfOut then
            G.sfOut = crouch
            G.stanceFast = bypassed == true and speed < (M.cfg.stanceFastSpeed or 0.5)
          end
        end
        -- slide = its own pivot height while it is lower than the crouch one (camera.slideHeight)
        slide = slide == true and (M.cfg.slideHeight or M.cfg.crouchHeight) < M.cfg.crouchHeight
        local stance = slide and "slide" or (crouch and "crouch" or "stand")
        if stance ~= (G.stance or "stand") then
          G.stance, G.crouch = stance, crouch
          -- the slide pivot is scaled in by the aim weight: aimed = camera.slideHeight, free = the crouch height
          local height = slide and string.format("%.2f m aimed / %.2f m free", M.cfg.slideHeight, M.cfg.crouchHeight)
            or string.format("%.2f m", crouch and M.cfg.crouchHeight or M.cfg.standHeight)
          local blendS = M.cfg.stanceBlend
          if slide then blendS = M.cfg.slideBlend
          elseif G.stanceFast then blendS = crouch and M.cfg.crouchBlend or M.cfg.standBlend end
          M.Log(string.format("pivot stance -> %s (PSM Locomotion %d, LocomotionDetailed %d): pivot %s over %.2f s",
            stance, loco, det, height, blendS))
        end
        rayFn = (okRay and sqs) and makeRaycast(sqs) or nil
        -- vehicle: the camera's own pitch (fppPitch not read) while held for a vehicle, in a
        -- traversal (PSM LocomotionDetailed climb 8 / vault 9 / ladder 10-13, the heading-lock classification
        -- LOCK_GAME_DETAILED) and for cfg.ownPitchHold s after the game look write turned off or a vehicle hold ended
        -- After a vehicle hold the 0.5 s are extended (at most cfg.ownPitchVehMax s) while the game's look pitch is outside
        -- the release range by more than cfg.ownPitchVehMargin deg: the exit animation still drives it (-60 at the release
        -- end in the recording). A one-frame look write ("seed") cannot fix it in the follow mode: the native write skips
        -- every tick without the heading lock (GameLookCore.hpp Step: Skip::NotLocked); the next write ON (aim / free look)
        -- writes the camera pitch.
        local cfgC = M.cfg
        local hold = cfgC.ownPitchHold or 0.5
        local trav = LOCK_GAME_DETAILED[det] == "climb" or LOCK_GAME_DETAILED[det] == "vault" or LOCK_GAME_DETAILED[det] == "ladder"
        -- (passenger exit without a hold): the camera was not held at the exit, so nothing started the
        -- own pitch and the follow pitch took the game's exit look pitch (-60 at the unmount: V seen from straight above for
        -- 8 s). A mounted camera that is not held keeps its own pitch (mouse only), and the unmount starts the same after-hold
        -- phase as the end of a hold (vh.endedAt: 0.5 s, longer while the game's look pitch is outside the release range)
        local okMt, mtv = pcall(function() return Game.GetMountedVehicle(player) end)
        local mountedFree = okMt and mtv ~= nil and not vh.held
        if G.mountedFree == true and not mountedFree and not vh.held and vh.endedAt == nil and not (okMt and mtv ~= nil) then
          vh.endedAt = G.clock
          M.Log("unmounted without a vehicle hold: the camera keeps its own pitch until the game's look pitch is back in range")
        end
        G.mountedFree = mountedFree
        local sinceHold = vh.endedAt ~= nil and (G.clock - vh.endedAt) or nil
        local mg = cfgC.ownPitchVehMargin or 5
        local afterHold = sinceHold ~= nil and (sinceHold < hold or (sinceHold < (cfgC.ownPitchVehMax or 3) and viewPitch ~= nil
          and (viewPitch < (cfgC.vehReleasePitchMin or -25) - mg or viewPitch > (cfgC.vehReleasePitchMax or 5) + mg)))
        if sinceHold ~= nil and not afterHold and sinceHold >= hold then vh.endedAt = nil end
        G.afterHoldNow = afterHold == true -- the look write ramp (lookRampStep) starts on the first write after a vehicle hold
        local opWhy = (vh.held and "vehicle hold") or (trav and "traversal") or (afterHold and "after a vehicle hold")
          or (mountedFree and "mounted without a vehicle hold")
          or ((G.lwOffAt ~= nil and G.clock - G.lwOffAt < hold) and "after the look write") or nil
        local ownPitch = opWhy ~= nil
        if opWhy ~= G.ownPitchWhy then
          -- one line per change; a phase that is only the plain look-write edge (every aim / free look end) only with
          -- camera.trace
          if opWhy ~= nil and opWhy ~= "after the look write" then G.ownPitchLoud = true end
          if M.rt.trace == true or G.ownPitchLoud then
            M.Log(opWhy and string.format("own pitch ON (%s%s)", opWhy, trav and (": LocomotionDetailed " .. tostring(det)) or "")
              or "own pitch OFF -> the follow pitch glides onto the game's look pitch")
          end
          if opWhy == nil then G.ownPitchLoud = false end
          G.ownPitchWhy = opWhy
        end
        -- mount pivot continuity (M.MountShiftGate, CM.mountShiftStep): own mount read in third person only
        local mshMounted = false
        if M.cfg.mountBlend ~= false and (M.T.state == "tpp" or M.T.state == "leaving") then
          local okMV, mv = pcall(function() return Game.GetMountedVehicle(player) end)
          mshMounted = okMV and mv ~= nil
        end
        local mshCam = nil
        if G.last and G.last.world and vh.override == nil then
          mshCam = { x = G.last.world.x, y = G.last.world.y, z = G.last.world.z, arm = G.last.arm, fullLen = G.last.fullLen }
          -- V's position and the clock of THAT solve, and only when it is the previous update (G.lastSolve); otherwise the sample is skipped
          local ls = G.lastSolve
          if ls ~= nil and ls.n == (G.updN or 0) - 1 then
            mshCam.px, mshCam.py, mshCam.pz, mshCam.t = ls.x, ls.y, ls.z, ls.t
          else
            mshCam.skip = true
          end
        elseif M.cfg.mountPullWatch ~= false and vh.held and vh.override ~= nil and G.last and G.last.world then
          -- the pose the pull-out override shows this frame (vh.override, set by utpp_vehiclecam at the end of the last frame and written by this
          -- update) against V of this frame (px / t unset = ppos): the watch goes on through the pull-out and its hand-over from the orbit
          local o = vh.override
          mshCam = { x = o.x, y = o.y, z = o.z, shown = true }
        end
        local mshArm, mshKeep, mshLine, mshCamLine = M.MountShiftGate(G, M.cfg, G.clock, G.updN or 0, M.T.state, mshMounted, vh, ppos, mshCam)
        if mshLine then M.Log(mshLine) end
        if mshCamLine then M.Log(mshCamLine) end
        -- ragdoll camera: frames without a solve (first person, menu, photo mode) end a running episode (like the stance filter above:
        -- a spring that resumes from a stale position would sweep); a state that is still on starts a new one from the last shown pivot
        if M.S.rag ~= nil and G.ragN ~= (G.updN or 0) - 1 then
          if M.S.rag.on or M.S.rag.out ~= nil then
            M.Log("ragdoll camera: frames without a camera solve (first person / menu) - the episode state is dropped")
          end
          M.S.rag, G.ragPhase, G.ragEp = nil, nil, nil
        end
        G.ragN = G.updN
        local ragIn = M.RagdollRead(G, det, vh.held) -- ragdoll camera (nil = natives absent / switched off)
        res = CM.solve(M.S, {
          ragdoll = ragIn,
          dt = dt, playerPos = ppos, playerQ = playerQ, bodyYaw = bodyYaw,
          -- (fppPitch / refYaw = the camera-equivalent look while the game's look echoes our write)
          eyeZ = view and view.pos.z or nil, eyePos = view and view.pos or nil, fppPitch = viewPitch, lookEcho = lwEcho,
          lookWriteOn = G.lwOwn == true, -- own aim / zoom pitch and yaw while the game's look is written
          adjustPending = G.adjPend == true, -- the game's look of this frame is the adjustTransform override
          fppFov = G.fppFov,
          dyaw = dyaw, dpitch = dpitch, freeHeld = M.rt.freeHeld, moving = moving,
          shooterMode = G.shooterMode == true,
          -- (Locomotion off for < decoupleHold s keeps it)
          decoupled = M.cfg.decoupleLocomotion ~= false and G.freeMoveHold == true and not A.on and not ZU.on and G.shooterMode ~= true,
          refYaw = viewYaw, -- aim reference = the game's look yaw (FPP camera)
          lookYaw = lookYaw, -- the game's look while synced (free / own aim yaw source)
          profile = p, side = CM.shoulderStep(M.shoulder, dt, M.cfg.shoulderTime), w = M.T.w,
          zoom = M.rt.zoom * (1 - poseW), -- zoom masked by the aim pose (and the unarmed zoom pose)
          raycast = rayFn, crouch = crouch, slide = slide, stanceFast = G.stanceFast == true,
          -- (P7 F2): LocomotionDetailed SlideFall 6 / Fall 14 / Jump 18 ... DodgeAir 22 (adapter::PsmAir): the slide pivot's exit blend
          -- into the air is short and the pivot base follows the take-off (CM.pivotStep)
          air = det == 6 or (det >= 14 and det <= 22),
          -- the orbit is hidden while held (override / release blend): no limiter lag there. Vehicle hold:
          -- only then - a hold that still shows the orbit (mount start, before the pull-out) keeps the world yaw like free
          -- look (holdYaw: no whip when the mount animation turns V to the door)
          -- also under a mount pull-out override (keepYaw) - the orbit does not chase the mount spin
          followSnap = vh.held and (vh.override ~= nil or vh.release ~= nil) and not M.VehicleHoldYaw(vh),
          holdYaw = M.VehicleHoldYaw(vh),
          ownPitch = ownPitch,
          -- forced follow, FOV on the aim tween; the same for the unarmed zoom (body behind the camera)
          aim = A.on or ZU.on, aimW = poseW, fovDirect = CM.aimActive(A) or CM.zoomUnarmedActive(ZU),
          -- the unarmed zoom's own boom orbit (no framing limit by default). (review: the
          -- on flags switched the rule set in one frame when aim / zoom tweens overlapped): blended by the pose weights
          zoomShare = (ZU.w or 0) / math.max((A.w or 0) + (ZU.w or 0), 1e-6),
          mountArm = mshArm, mountKeep = mshKeep, -- mount pivot continuity
          jumpOut = vh.held and vh.exitHold == true, -- jump-out landing: the exit hold arms the pivot's jump-out mode (CM.jumpOutStep)
          -- a hold that still shows the orbit (vehicle mount, before the pull-out override / release): the boom's
          -- pull-in by a car body is rate limited (cfg.vehHoldColRate m/s, 0 = instant)
          -- capped at vehHoldColMountRate on both gate paths (the immediate one ran at 12 m/s while V was still moving), lengthening at vehHoldColLift
          colInRate = (vh.held and vh.car and vh.override == nil and (vh.release == nil or vh.release.remount == true) and (M.cfg.vehHoldColRate or 0) > 0)
            and (((M.cfg.vehHoldColMountRate or 0) > 0) and math.min(M.cfg.vehHoldColRate, M.cfg.vehHoldColMountRate) or M.cfg.vehHoldColRate) or nil,
          colInLift = (vh.held and vh.car and vh.override == nil and (vh.release == nil or vh.release.remount == true) and (M.cfg.vehHoldColRate or 0) > 0
            and (M.cfg.vehHoldColLift or 0) > 0) and M.cfg.vehHoldColLift or nil,
          -- the same hold: a pull-in starts after vehHoldColDelay s of pressure (an opening door crossing the boom is no occluder) -
          -- only while the pivot AND the camera (yaw / pitch of the solve) have been at rest for vehHoldColQuiet s: a hit that comes with the pivot
          -- moving (V sliding into the seat next to a body, the mount blend) or with the camera turning (mouse-look swings the boom into the car body)
          -- is an occluder and keeps the rate limited pull-in at once
          colInDelay = (vh.held and vh.car and vh.override == nil and (vh.release == nil or vh.release.remount == true) and (M.cfg.vehHoldColRate or 0) > 0
            and (M.cfg.vehHoldColDelay or 0) > 0 and (G.pivMoveAt == nil or G.clock - G.pivMoveAt >= (M.cfg.vehHoldColQuiet or 0.3)))
            and M.cfg.vehHoldColDelay or nil,
          -- the same hold: the pull-in does not come closer to the pivot than vehHoldColMin (a V close-up from the far side of the cab)
          colInMin = (vh.held and vh.car and vh.override == nil and (vh.release == nil or vh.release.remount == true) and (M.cfg.vehHoldColRate or 0) > 0)
            and (M.cfg.vehHoldColMin or 0) or nil,
        })
        G.last = res
        G.lastSolve = { n = G.updN or 0, t = G.clock, x = ppos.x, y = ppos.y, z = ppos.z or 0 } -- V's position / clock / update of this solve (mount camera watch)
        do -- pivot speed (xy) and camera turn rate (yaw / pitch) of the solve, for the quiet test above; a running mount pivot blend counts as moving
          local pp, pv = G.pivPrev, res.pivot
          if pv ~= nil then
            if pp ~= nil and G.clock > pp.t + 1e-4 and G.clock - pp.t < 0.25 then
              local sp = math.sqrt((pv.x - pp.x) ^ 2 + (pv.y - pp.y) ^ 2) / (G.clock - pp.t)
              if sp > (M.cfg.vehHoldColQuietSpeed or 0.7) then G.pivMoveAt = G.clock end
              local qr = M.cfg.vehHoldColQuietRot or 0
              if qr > 0 and pp.yaw ~= nil and type(res.yaw) == "number" and type(res.pitch) == "number" then
                local rate = math.max(math.abs(CM.wrapDeg(res.yaw - pp.yaw)), math.abs(res.pitch - pp.pitch)) / (G.clock - pp.t)
                if rate > qr then G.pivMoveAt = G.clock end
              end
            elseif pp == nil or G.clock - pp.t >= 0.25 then
              G.pivMoveAt = G.clock -- no recent solve: unknown, not quiet
            end
            if M.S ~= nil and M.S.mshift ~= nil then G.pivMoveAt = G.clock end
            G.pivPrev = { x = pv.x, y = pv.y, t = G.clock, yaw = res.yaw, pitch = res.pitch }
          end
        end
        -- ragdoll camera: episode flags for the freeze below (heading lock, look write, calibration, turn prediction) and
        -- one log line per enter / exit (at most one per kind per ragLogGap s)
        do
          local rg = res.ragdoll
          local was = G.ragPhase
          G.ragPhase = rg and rg.phase or nil
          G.ragBoom = rg ~= nil and (rg.w or 0) > 0 -- the boom bias still fades (ragBlendOut > ragExitBlend): no game look write meanwhile
          G.ragEp = G.ragPhase == "on"
          if rg and rg.ev then
            local ev = rg.ev
            G.ragLog = G.ragLog or {}
            local last = G.ragLog[ev.kind]
            if ev.kind == "exit" then G.unlockAt, G.lockHold = G.clock, G.clock + (M.cfg.ragLockGrace or 0.5) end
            if last == nil or G.clock - last >= (M.cfg.ragLogGap or 2.0) then
              G.ragLog[ev.kind] = G.clock
              if ev.kind == "enter" then
                local t = rg.target
                M.Log(string.format("ragdoll camera ON (state %d, PSM LocomotionDetailed %d): pivot follows the pelvis + %.2f m (spring %.2f / %.2f s), "
                  .. "boom +%.2f m, pitch %+.0f deg; look kept free, heading lock frozen%s", ev.state or 0, det, M.cfg.ragLift or 0,
                  M.cfg.ragSmoothActive or 0, M.cfg.ragSmoothGetUp or 0, M.cfg.ragDistAdd or 0, M.cfg.ragPitchBias or 0,
                  t and string.format(" (pelvis target %.1f / %.1f / %.1f)", t.x, t.y, t.z) or ""))
              elseif ev.kind == "return" then
                M.Log(string.format("ragdoll camera RETURN (native get-up state %d%s) after %.1f s: pivot / boom%s return to the normal framing in %.2f s",
                  ev.state or 0, ev.frac and string.format(", clip %.0f%%", ev.frac * 100) or "", ev.t or 0,
                  M.cfg.ragRecenter == true and " / yaw (until the first look input)" or " (the view stays with the player's look)", ev.time or 0))
              else
                M.Log(string.format("ragdoll camera OFF (%s) after %.1f s: pivot crosses over to the capsule pivot in %.2f s", ev.why or "?",
                  ev.t or 0, ev.blend or 0))
              end
            end
          end
          if was ~= nil and G.ragPhase == nil and not (rg and rg.ev) then G.unlockAt = G.clock end
        end
        -- fast fall: the pivot base was snapped / clamped (CM.pivotStep snapEv: a teleport or a lag over cfg.pivotLagMax);
        -- one line per event, at most one per second (a clamp can repeat every frame)
        do
          local sev = M.S.pivot.snapEv
          if sev ~= nil then
            M.S.pivot.snapEv = nil
            G.snapN = (G.snapN or 0) + 1
            if G.snapLogAt == nil or G.clock - G.snapLogAt >= 1.0 then
              G.snapLogAt = G.clock
              M.Log(string.format("pivot %s: lag %.2f m, V vz %.1f m/s, step %.2f m, dt %.3f s (event %d)", sev.kind, sev.lag or 0,
                sev.vz or 0, sev.dz or 0, sev.dt or 0, G.snapN))
            end
          end
        end
        -- (C2): vanilla near-camera head hide while the boom is inside the body clearance - only while utppCamera
        -- renders (entering / tpp / leaving; not the rep_wait frame) and on foot: not under a vehicle hold / hand-over, not
        -- within vehicleQuiet s after one, not mounted (the mount read only on inside frames)
        do
          local ih = res.inside == true and CM.trActive(M.T) and not vh.held and not vh.handed
            and not (vh.endedAt ~= nil and G.clock - vh.endedAt < (M.INSIDE_HIDE.vehicleQuiet or 0))
          if ih then
            local okMV, mv = pcall(function() return Game.GetMountedVehicle(player) end)
            ih = not (okMV and mv ~= nil)
          end
          M.InsideHideStep(ih, G.clock, res.arm, res.insideWall == true)
        end
        do -- mount pivot continuity: one line per start / restart / abort, the end only with camera.trace
          local ev = res.mountShift
          if ev then
            if ev.kind == "start" or ev.kind == "restart" then
              G.mshStarted = true
              M.Log(string.format("mount pivot blend %s: V moved %.2f m in one frame %.2f s after the mount -> pivot xy from "
                .. "the last shown position to V over %.2f s%s", ev.kind, ev.d or 0, G.clock - (G.mshSince or G.clock),
                M.cfg.mountBlendTime or 0, vh.held and "" or " (before the vehicle hold)"))
            elseif ev.kind == "abort" then
              M.Log(string.format("mount pivot blend aborted (%s%s) - pivot on V", tostring(ev.why),
                ev.d and string.format(" %.2f m", ev.d) or ""))
            elseif M.rt.trace == true then
              M.Log("mount pivot blend end - pivot on V")
            end
          end
        end
        -- aim shoulder auto-swap: horizontal ray from the pivot to the shoulder side (camera.aim.autoSwap)
        if A.on and M.cfg.aimAutoSwap and rayFn and res.pivot then
          local right = CM.qrot(CM.qFromYawPitch(res.yaw, 0), CM.v3(1, 0, 0))
          local sgn = (M.shoulder.target or 1) >= 0 and 1 or -1
          local len = M.cfg.aimSwapWall or 0.5
          local function cast(s) return rayFn(res.pivot, CM.add(res.pivot, CM.scale(right, s * len))) end
          local swap, d = CM.aimSwapStep(A.swap, M.cfg, dt, cast(sgn), function() return cast(-sgn) end)
          if swap then
            local s = CM.shoulderSwap(M.shoulder)
            A.swaps = A.swaps + 1
            M.stats.aimSwaps = M.stats.aimSwaps + 1
            M.Log(string.format("cam aim swap -> %s (wall %.2f m)", s > 0 and "R" or "L", d))
          end
        end
        -- aim occlusion (level A): after the aim tween (M.AimUpdate above), the solve (collision rays) and the
        -- auto-swap ray, before the pose write; the pose solved this frame is the one written = the shot origin
        -- one crosshair point P per frame for both native targets (CM.AIM_POINT; nil = the rays as before)
        local apD = M.AimPointUpdate(res, view and view.pos or nil, rayFn, dt,
          function() return A.on == true or weaponDrawn(player) end)
        M.FeedCameraTarget(res, apD)
        M.AimOccFrame(G.clock, res.world, CM.forward(res.yaw, res.pitch), function() return muzzlePos(player) end, rayFn)
        -- aim offset (DLL AimOffset): the camera pose written this frame is the aim ray; the DLL rotates the
        -- weapon arm so the barrel points at camPos + camFwd * 60 m while the upper-body aim mask is active. One
        -- enabled=false after the aim state ends; errors logged once (older DLL without the native).
        do
          local on = A.on == true and (A.w or 0) > 0.5
          if on or M.aimTargetSent then
            -- (review: the barrel aimed along the camera ray while the game's look goes eye -> P): with
            -- a parallax corrected game look written last frame, the target is 60 m along the same (bounded) corrected
            -- direction from the eye (the correction moves <= aimLookRate deg/s, so one frame old is close).
            -- with a crosshair point P (apD) the target is P itself (CM.aimTargetRay)
            local org, fwd = CM.aimTargetRay(res, lwPrev, view and view.pos or nil, apD)
            local okT, errT = pcall(function()
              Game.UTPPSetAimTarget(Vector4.new(org.x, org.y, org.z, 1), Vector4.new(fwd.x, fwd.y, fwd.z, 0), on)
            end)
            if okT then
              if on ~= (M.aimTargetSent == true) then
                M.aimTargetSent = on
                M.Log(on and "aim offset: target feed ON (UTPPSetAimTarget each frame)" or "aim offset: target feed OFF")
              end
            elseif not M.aimTargetUnavailable then
              M.aimTargetUnavailable = true
              M.Log("aim offset: UTPPSetAimTarget unavailable (" .. tostring(errT) .. ") - DLL too old?")
            end
          end
        end
        if res.flip then
          M.stats.flips = M.stats.flips + 1
          M.Log(string.format("follow: body turned %.0f deg within %.2f s without the mouse -> camera holds its world yaw "
            .. "%.2f s, then follows at <= %.0f deg/s (camera.followHoldOnFlip)", res.flip.deg, res.flip.window, res.flip.hold,
            res.flip.rate))
        end
      end
      if res == nil and not freeze then G.ragPhase, G.ragEp, G.ragBoom = nil, nil, nil end -- ragdoll camera: no stale freeze
      if res == nil and not freeze then M.AimOccReset("camera inactive", G.clock) end -- no stale aim occlusion flag
      if res == nil and not freeze then M.InsideHideRelease("camera inactive") end -- (C2)
      if res == nil and not freeze then CM.aimPointReset(M.apt) end -- no stale crosshair point P
      -- Revoke before a camera handoff. Positive permission is renewed only
      -- after the update actually wrote a full TPP camera pose, never entering/leaving/FPP.
      local poseWriteTpp = M.PoseWriteTppWanted(M.T, freeze, camOK, res, vh.held, cmds, vh.onFoot)
      -- diagnostic: which clause vetoes the write; one line per change, only during a vehicle hold or within 3 s after it
      do
        local why = poseWriteTpp and "ok" or M.PoseWriteDeniedReason(M.T, freeze, camOK, res, vh.held, vh.onFoot) or "command"
        if why ~= M.rt.pwWhy and (vh.held or (vh.endedAt ~= nil and G.clock - vh.endedAt < 3.0)) then
          M.rt.pwWhy = why
          M.Log(string.format("pose write permission: %s (held=%s onFoot=%s)", why, tostring(vh.held), tostring(vh.onFoot)))
        end
      end
      if not poseWriteTpp then M.PoseWriteTpp(false) end
      execute(cmds, player, comp, fpp, res, wctx)
      -- game-side TPP signal (also the first frame: clears a stale value after a CET reload)
      M.SignalTpp(M.TppSignalState(M.T), #cmds > 0 and (table.concat(cmds, "+") .. " epoch=" .. M.T.epoch) or M.T.state, G.clock)
      if G.nudges and not freeze and G.clock >= G.nudges[1].t then
        local n = table.remove(G.nudges, 1)
        nudgePose(player, speed, n.why)
        if #G.nudges == 0 then G.nudges = nil end
      end
      -- free camera while utpp_bodyturn turns V (its last run applied a step): the turn of the coming
      -- game tick is predicted from the body step just measured and taken out of the local transform (camera cfg
      -- predictBodyTurn; the component is in the entity space and this CET frame runs before the tick)
      G.pred = 0
      -- (only for utppCamera - the world camera does not turn with V)
      if res and M.cfg.predictBodyTurn ~= false and M.S.look.free and M.T.state == "tpp" and not vh.held
          and G.use ~= "world" and G.ragPhase == nil then
        if G.btMod == nil or (G.btMod == false and G.clock >= (G.btRetry or 0)) then
          local okB, bt = pcall(function() return GetMod("utpp_bodyturn") end)
          G.btMod = (okB and type(bt) == "table") and bt or false
          G.btRetry = G.clock + 5
        end
        local bt = G.btMod
        if bt and type(bt.exec) == "table" and (bt.exec.why == "applied" or bt.exec.why == "shooterMode align") then
          -- (foliage / buildings shimmer on fast direction changes, not visible in the 60 fps
          -- recording; the game runs at ~350 fps): the step predicted from the last measured body step misses by up
          -- to ~0.8 deg per frame when the turn speed changes -> a one-frame camera wobble at a high frame rate. When
          -- utpp_bodyturn runs before the camera in the CET frame, the step it just requested for the coming tick is
          -- used exactly; otherwise the prediction as before.
          -- The CET mod order is not guaranteed, so it is measured: with utpp_bodyturn BEFORE the camera the body step
          -- measured now equals the step seen on the previous camera frame; with it AFTER, the step seen now.
          local xs = bt.execStats
          local s = type(xs) == "table" and type(xs.lastStep) == "number" and xs.lastStep or nil
          if s ~= nil and math.abs(bodyDelta) > 0.3 then
            G.ordN = (G.ordN or 0) + 1
            G.ordPrev = (G.ordPrev or 0) + math.abs(bodyDelta - (G.sPrev or 0))
            G.ordNow = (G.ordNow or 0) + math.abs(bodyDelta - s)
            if G.ordN == 40 then
              G.btBefore = G.ordPrev < 0.5 * G.ordNow
              M.Log(string.format("predict: utpp_bodyturn runs %s the camera in the CET frame (40 turn frames: err prev-step %.1f, "
                .. "this-step %.1f deg) -> %s", G.btBefore and "BEFORE" or "after", G.ordPrev, G.ordNow,
                G.btBefore and "exact requested step used" or "predicted step kept"))
            end
          end
          G.sPrev = s
          if G.btBefore == true and s ~= nil and math.abs(s) <= (M.cfg.predictMaxStep or 15) then
            G.pred, G.predExact = s, true
          else
            G.pred, G.predExact = CM.predictTurn(bodyDelta, dt, G.prevDt, M.cfg.predictMaxStep), false
          end
        end
      end
      G.prevDt = dt
      G.wroteWorld = false
      local poseWriteSucceeded = false
      local poseWriteCamera = nil
      if res and camOK and (CM.trActive(M.T)) then
        -- also while aiming / zooming with the utpp_bodyturn hold (aim align turns V onto the game's look):
        -- without the lock the FPP camera turned with V and the align chased it (541 deg, stopped by its guard)
        -- (held a few frames past the aim / zoom end by the release flush - lookWriteFlush)
        -- ragdoll camera: the lock is frozen as it is for the episode (the free yaw of the episode must not set it, the game's
        -- parameter sets of a knockdown must not be answered); the lock edge of an ended episode comes afterwards as for any free look
        if G.ragPhase == nil then
          setHeadingLock(fpp, lookWriteFlush((not vh.held) and (M.S.look.free or ((A.on or ZU.on) and (M.cfg.aimOwnYaw ~= false
            or M.rt.alignHold == true))) and M.T.state ~= "leaving", lwPrev, vh.held), player)
        end
        lookSyncReconcile() -- also a camera.lookSync change while the lock is set
        -- world camera: ready while utppCamera shows this third person -> it takes over at the shown pose
        -- (not while held for a vehicle: the hold's priority / hand-over stay on one camera)
        if wcomp ~= nil and G.use ~= "world" and M.T.state == "tpp" and not vh.held and not vh.handed then
          if not switchCam("world", "world camera ready in third person", wctx) then
            wcFail("Activate failed at the take-over in third person", wctx)
          end
        end
        local useW = G.use == "world" and wcomp ~= nil
        local cam = useW and wcomp or comp
        -- while held for a vehicle the override / release blend is written (nothing while handed over)
        local wpos, wyaw, wpitch, kind, wfov = res.world, res.yaw, res.shownPitch or res.pitch, "orbit", nil
        if vh.held then wpos, wyaw, wpitch, kind, wfov = M.VehicleCompose(res, G.clock) end
        G.jpShown = vh.held and (vh.jpShown or 0) or (res.joPitch or 0) -- the landing framing tilt in the written pitch
        wfov = wfov or res.fov
        if wpos and cam then
          local okW, errW = pcall(function()
            if useW then
              -- world camera: the world pose as is (no entity space, no turn prediction)
              local wq = (kind == "orbit") and res.q or CM.qFromYawPitch(wyaw, wpitch)
              cam:SetLocalTransform(V4(wpos), Quaternion.new(wq.i, wq.j, wq.k, wq.r))
            else
              local lp, lq = res.localPos, res.localQ
              if kind ~= "orbit" then lp, lq = CM.toLocal(ppos, playerQ, wpos, CM.qFromYawPitch(wyaw, wpitch))
              elseif (G.pred or 0) ~= 0 then lp, lq = CM.toLocal(ppos, CM.qmul(CM.qFromYawPitch(G.pred, 0), playerQ), wpos, res.q) end
              cam:SetLocalTransform(V4(lp), Quaternion.new(lq.i, lq.j, lq.k, lq.r))
            end
            cam:SetFOV(wfov)
          end)
          if okW then
            poseWriteSucceeded = true
            poseWriteCamera = G.use
            M.stats.writes = M.stats.writes + 1
            -- kept for M.OnBodyStep (exact rewrite after utpp_bodyturn requested its step in this frame)
            G.lastWrite = (not useW and kind == "orbit" and M.S.look.free) and { comp = comp, ppos = ppos, q = playerQ,
              wpos = wpos, wq = res.q, clock = G.clock } or nil
            vh.last = { x = wpos.x, y = wpos.y, z = wpos.z, yaw = wyaw, pitch = wpitch,
                        fov = (wfov ~= res.fov) and wfov or nil } -- an override FOV blends back on release
            G.wroteYaw = wyaw -- trace (camErr next frame)
            G.wroteKind = kind -- (c9) trace: override / release / orbit
            G.wroteWorld = useW
          else
            M.stats.errors = M.stats.errors + 1
            if M.stats.errors <= 3 then M.Log("write FAILED: " .. tostring(errW)) end
            G.lastWrite = nil
            if useW then wcFail("write: " .. tostring(errW), wctx, true) end
          end
        end
        -- game look write: the shown world camera's yaw / pitch while free / aiming (heading lock set)
        if CM.lookWriteWant(M.cfg, { world = G.wroteWorld, kind = kind, state = M.T.state, held = vh.held or G.ragPhase ~= nil or G.ragBoom == true,
            locked = G.lockSet }) then
          -- aiming / zooming: from the eye to the crosshair point P (parallax), else the camera yaw / pitch
          G.lwTarget = aimLookAngles(res, view and view.pos or nil, rayFn, A.on or ZU.on, dt)
        end
        lookRampStep(G.lwTarget, view, dt)
        if kind == "release_end" then M.HoldForVehicle(false, "release to orbit done") end
        -- watchdog: is utppCamera really the rendering camera? (off while held: the vehicle camera may render)
        G.watch = G.watch + dt
        G.reactWindow = G.reactWindow + dt
        if G.reactWindow > 10 then G.reactWindow, G.reactCount = 0, 0 end
        if vh.held then G.watch, G.misses = 0, 0 end
        if M.T.state == "tpp" and G.watch >= 0.5 then
          G.watch = 0
          local apos = activeCameraPos(camSys)
          local wW = G.use == "world" -- the world camera is the camera in use
          if apos and CM.dist(apos, res.world) > 0.3 then
            G.misses = G.misses + 1
            if G.misses >= 2 and wW and M.wc.reacts >= 2 then
              -- two re-activations did not make it the rendering camera: component path
              G.misses = 0
              wcFail(string.format("not rendering: active camera %.2f m away after %d re-activations", CM.dist(apos, res.world),
                M.wc.reacts), wctx)
            elseif G.misses >= 2 and (G.reactCount < 3 or wW) then
              G.misses, G.reactCount = 0, G.reactCount + 1
              if wW then M.wc.reacts = M.wc.reacts + 1 end
              M.stats.reactivations = M.stats.reactivations + 1
              local cam = wW and wcomp or comp
              pcall(function() cam:Activate(0, M.rt.audio) end)
              M.Log(string.format("watchdog: active camera %.2f m away from %s -> Activate again (%d)",
                CM.dist(apos, res.world), wW and "the world camera" or "utppCamera", G.reactCount))
            end
          else
            G.misses = 0
            if wW then M.wc.reacts = 0 end
          end
        end
      elseif G.lockSet and not CM.trActive(M.T) then
        setHeadingLock(fpp, false)
      end
      -- The existing camera owner/watchdog decides perspective. Do not compare
      -- this frame's target against a potentially previous-frame rendered transform.
      M.PoseWriteTpp(poseWriteTpp and poseWriteSucceeded and poseWriteCamera ~= nil and
        G.use == poseWriteCamera and (G.use == "world" or G.use == "comp"))
      if not freeze then
        local look = M.S.look
        lookWriteFrame(G.lwTarget, G.lwTarget and (M.aim.on and "aim" or (M.zoomU.on and "zoom" or (look.free and "free"
          or "locked"))) or (M.cfg.lookWrite == false and "camera.lookWrite off" or (M.T.state ~= "tpp" and ("state " .. tostring(M.T.state))
          or (M.vh.held and "vehicle hold" or (G.lockSet and "world camera not shown" or "heading unlocked (follow)")))))
      else
        G.lwAheadPend = nil -- no body step survives a frozen frame
      end
      -- camera trace (camera.trace): one line per 0.1 s in third person
      if G.trDue then
        -- 10 Hz on a fixed grid (no drift to 8.6 Hz at 60 fps); restarted after a pause (first person, menu)
        G.trNext = (G.trNext and G.clock - G.trNext < 0.1) and (G.trNext + 0.1) or (G.clock + 0.1)
        local look = M.S.look
        local mode = vh.held and "hold" or (A.on and "aim" or (ZU.on and "zoom" or (look.free and "free" or "follow")))
        local bt, btWhy = "-", "-"
        if type(G.btMod) ~= "table" and G.btMod ~= false then
          local okB, m = pcall(function() return GetMod("utpp_bodyturn") end)
          G.btMod = (okB and type(m) == "table") and m or false
        end
        if type(G.btMod) == "table" and type(G.btMod.exec) == "table" then
          local e = G.btMod.exec
          bt = e.nativeOn == true and "1" or (e.nativeOn == false and "0" or "-")
          btWhy = tostring(e.why)
        end
        M.Log(string.format("camTrace t=%.2f body=%.1f out=%s pitch=%s mode=%s freeMove=%d dec=%d bt=%s(%s) aimW=%.2f dist=%s "
          .. "pivotZ=%s blocked=%d dyaw=%.2f dyawSum=%.1f fpp=%s camErr=%s pred=%.2f bdelta=%.2f unit=%.4f hits=%d speed=%.2f "
          .. "wcam=%d sync=%s lerr=%s lsend=%s lw=%d lwerr=%s lwperr=%s lwp=%s lwown=%d lwa=%s adj=%d",
          G.clock, bodyYaw, res and string.format("%.1f", res.yaw) or "-", res and string.format("%.1f", res.pitch) or "-",
          mode, G.freeMoveActive and 1 or 0, (G.freeMoveHold and M.cfg.decoupleLocomotion ~= false) and 1 or 0, bt, btWhy,
          math.max(A.w or 0, ZU.w or 0),
          res and string.format("%.2f/%.2f", res.arm or 0, res.fullLen or 0) or "-",
          (res and res.pivot) and string.format("%.2f(+%.2f)", res.pivot.z, res.pivot.z - ppos.z) or "-",
          M.occ.blocked and 1 or 0, G.trDyaw or 0, G.trDyawSum or 0,
          view and string.format("%.1f/%.1f", view.yaw, view.pitch) or "-",
          G.trErr and string.format("%.2f", G.trErr) or "-", G.pred or 0, bodyDelta,
          CM.mouseUnit(M.cal, M.cfg, M.rt.mouseScale), res and res.hits or 0, speed, G.wroteWorld and 1 or 0,
          -- sync = on (camera = game look) | wait (set sent, not confirmed) | off | broken | noroute;
          -- lerr = shown yaw - game look yaw (deg, ~0 while on); lsend = param set sent (1/0)
          G.lookUsed and "on" or (G.ls.broken and "broken" or (not G.ls.sent and "off"
            or (G.ls.route == nil and "noroute" or "wait"))),
          (res and view) and string.format("%.1f", CM.wrapDeg(res.yaw - view.yaw)) or "-", G.ls.sent and 1 or 0,
          -- lw = the game's look written this frame; lwerr / lwperr = game look read back - target of the last frame
          G.lwState == "on" and 1 or 0, G.lwErr and string.format("%.2f", G.lwErr) or "-",
          G.lwPErr and string.format("%.2f", G.lwPErr) or "-",
          -- lwp = distance of the crosshair point P the game's look aims at (aiming / zooming; - = camera yaw / pitch)
          G.lwp and string.format("%.2f", G.lwp) or "-",
          -- lwown = 1 while the camera's aim / zoom pitch and yaw are its own (the game's look written, not read
          -- back); lwa = V's turn sent with the look target of the last frame (utpp_bodyturn step, deg; - = none)
          G.lwOwn and 1 or 0, (lwPrev and lwPrev.ahead and lwPrev.ahead ~= 0) and string.format("%.2f", lwPrev.ahead) or "-",
          -- adj = 1 while an adjustTransform turn of V is pending / landing (the game overrides its look then;
          -- lwerr / lwperr of such a row are the override, not a loop error; the camera does not read the game's pitch)
          G.adjPend and 1 or 0) .. (G.shooterMode == true and " shooter=1" or "")
          -- (c9): while held, shown = the yaw written to the camera this frame (override / release / orbit) and
          -- the kind (out is the orbit solve, hidden under an override)
          .. ((vh.held and poseWriteSucceeded and G.wroteYaw) and string.format(" shown=%.1f/%s", G.wroteYaw,
            tostring(G.wroteKind)) or "")
          -- ap = filtered distance of the crosshair point P the native aim / camera targets aim at (m)
          .. (M.apt.d and string.format(" ap=%.2f", M.apt.d) or "")
          -- while the pivot's jump-out landing mode runs: jo = ground estimate under V (m, relative to the pivot base; - = none) / landing
          -- brake running (1/0) / recovery brake after a falling estimate (1/0) / base speed (m/s)
          .. ((M.S.pivot and M.S.pivot.joOn) and string.format(" jo=%s/%d/%d/%.1f",
            M.S.pivot.joG and string.format("%.2f", M.S.pivot.joG - (M.S.pivot.base or 0)) or "-", M.S.pivot.joBr and 1 or 0, M.S.pivot.joRec and 1 or 0,
            M.S.pivot.vb or 0) or ""))
        G.trDyawSum = 0
      end
      G.trDyaw = 0
      -- one line per frame while the pivot's jump-out mode runs (camera.trace; camTrace is 10 Hz): the landing at frame rate. Vz / Vvz = V's height and
      -- vertical speed, base / vb / a = the pivot base height, speed, acceleration, G = ground estimate (- = none), br / rec / ret = landing brake / recovery brake /
      -- re-target flags, xy = the pivot xy minus V's xy / the pivot's xy speed (- = the pivot xy is V's)
      if M.rt.trace == true and not freeze and M.S.pivot and M.S.pivot.joOn and M.S.pivot.joDdt then
        local pv = M.S.pivot
        local Sx = pv.xyJ
        -- cam = the world camera position written this frame / yaw / pitch (the rendered pose: the landing step is measured on it, not only on the pivot)
        local wl = M.vh.last
        M.Log(string.format("jo frame t=%.3f dt=%.4f Vz=%.3f Vvz=%.2f base=%.3f vb=%.2f a=%.1f G=%s br=%d rec=%d ret=%d xy=%s cam=%s jp=%.2f",
          G.clock, pv.joDdt, pv.joDz or 0, pv.joDv or 0, pv.base or 0, pv.vb or 0, pv.joA or 0, pv.joG and string.format("%.3f", pv.joG) or "-",
          pv.joBr and 1 or 0, pv.joRec and 1 or 0, pv.joRet and 1 or 0,
          (Sx and pv.xyLx) and string.format("%.3f,%.3f/%.2f,%.2f", Sx.cx - pv.xyLx, Sx.cy - pv.xyLy, Sx.ux, Sx.uy) or "-",
          (wl and poseWriteSucceeded) and string.format("%.3f,%.3f,%.3f/%.1f/%.1f", wl.x, wl.y, wl.z, wl.yaw or 0, wl.pitch or 0) or "-",
          G.jpShown or 0)) -- (jp = the landing framing tilt in the written pitch, CM.jumpOutFrameStep)
      end
      -- game look status (Game.UTPPGameLookStatus): at the write start and every 2 s while writing (trace on)
      if M.rt.trace == true and G.lwState == "on" and not freeze then
        if G.lwNext == nil or G.clock >= G.lwNext then
          G.lwNext = G.clock + 2
          local okS, s = pcall(function() return Game.UTPPGameLookStatus() end)
          if okS and s ~= nil then M.Log("game look status: " .. tostring(s)) end
        end
      elseif G.lwState ~= "on" then
        G.lwNext = nil
      end
      -- aim offset status (Game.UTPPAimOffsetStatus): at the aim start and every 2 s while aiming
      if M.rt.trace == true and A.on and not freeze then
        if G.aoNext == nil or G.clock >= G.aoNext then
          G.aoNext = G.clock + 2
          local okS, s = pcall(function() return Game.UTPPAimOffsetStatus() end)
          if okS and s ~= nil then M.Log("aim offset status: " .. tostring(s))
          else M.LogOnce("aoStatus", "aim offset status unavailable (" .. tostring(s) .. ") - UTPP.dll older than 0.6.3?") end
        end
      elseif not A.on then
        G.aoNext = nil
      end
      if G.clock <= G.diagUntil and G.clock >= G.diagNext then
        G.diagNext = G.clock + 0.1
        M.Log(string.format("diag t=%.2f dt=%.4f state=%s w=%.2f blocked=%s body=%.1f fppPitch=%s yaw=%s pitch=%s arm=%s/%s hits=%d "
          .. "mouse=%.2f,%.2f pad=%.2f,%.2f unit=%.4f moving=%s free=%s speed=%.2f",
          G.clock, dt, M.T.state, M.T.w, tostring(M.T.blocked), bodyYaw, view and string.format("%.1f", view.pitch) or "-",
          res and string.format("%.1f", res.yaw) or "-", res and string.format("%.1f", res.pitch) or "-",
          res and string.format("%.2f", res.arm) or "-", res and string.format("%.2f", res.fullLen) or "-",
          res and res.hits or 0, IN.mouseX, IN.mouseY, IN.padX, IN.padY, CM.mouseUnit(M.cal, M.cfg, M.rt.mouseScale),
          tostring(moving), tostring(M.S.look.free), speed))
      end
    end)
    IN.mouseX, IN.mouseY = 0, 0
    M.zuIn.wheel = 0 -- wheel steps are consumed by ZoomUnarmedUpdate; menu / photo frames drop them
    if not ok then
      M.PoseWriteTpp(false)
      M.stats.errors = M.stats.errors + 1
      if M.stats.errors <= 3 then M.Log("update error: " .. tostring(err)) end
    end
  end)
end

return M

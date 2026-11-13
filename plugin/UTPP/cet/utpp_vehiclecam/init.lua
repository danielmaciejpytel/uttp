-- UTPP utpp_vehiclecam (settings via utpp_core when installed) - vehicle camera on UTPP's own camera (utppCamera):
-- stays in third person while entering a vehicle, hands over to the vehicle TPP camera, holds a vehicle-behind pose on exit.
--
-- Game API (vanilla scripts): vehicleRequestCameraPerspectiveEvent
-- { cameraPerspective: vehicleCameraPerspective FPP 0 / TPPClose 1 / TPPMedium 2 / TPPFar 3 / DriverCombat* 4-6 } queued
-- on the player (DefaultTransition.RequestVehicleCameraPerspective; DriveEvents.OnEnter uses it for the vanilla
-- "requestedTPPCamera" flag), VehicleObject.GetCameraManager() -> VehicleCameraManager.GetActivePerspective() /
-- GetPersistentPerspective() / IsTPPActive(). PassengerEvents.OnEnter forces FPP -> passengers are never touched.
--
-- What this mod does (ctrl.own is set by the wiring when utpp_camera's vehicle API is present; commands are executed by
-- M.RunOwnCmds, the pose comes from M.OwnPose):
--   mount: HoldForVehicle(true) at the mount start (or when the camera asks on the "vehicle" decision) - the camera
--          stays in third person, the TPP signal stays on, no FPP handoff; pull-out around the vehicle (pullStart /
--          pullDist, pullTime, bikePullTime) via SetPoseOverride; high priority before the first vehicle TPP request;
--          handover = HandToVehicle(holdRelease) only when the vehicle reports a TPP perspective AND its camera blend is
--          over (+ optional pose match to the vehicleTPPCamera pose when readable). The vehicle TPP perspective (persistent
--          TPP, else TPPClose, or a fixed one) is requested every requestInterval s while entering/driving as driver, until
--          the manager reports a TPP perspective; then stop for good (the game's own camera toggle keeps FPP available).
--   ride:  the vehicle camera renders, the camera stays held (TPP signal on); the vehicle camera switched to FPP
--          (ownRideFppGrace), combat / passenger / turret / scene states -> the view is left to the game ("game" mode);
--   exit:  exit start = pending mount release flushed, the active (vehicle) camera pose read, SetPoseOverride(matched)
--          + TakeFromVehicle(earlyExitBlend) (high priority), match -> vehicle-behind pose over exitMatchTime, hold
--          until the unmount + exitHoldAfterUnmount (at most exitHoldMax after the exit start), then
--          ReleaseToOrbit(exitPullTime). Exit before the handover (short ride): no activation, blend from the shown pose.
-- Never touches input, PSM, mounting or exit; only queues perspective requests while armed (before exit). Passenger,
-- combat 2, driver combat 6, turret 5, scene 7, exit/seat switch after driving, timeout -> no request (vanilla behaviour).
-- Fail-safe: utpp_camera or its vehicle API missing -> nothing is done (vanilla vehicle camera). Kill switch:
--   GetMod("utpp_vehiclecam").SetEnabled(false)   (also SetPerspective(-1|1|2|3), SetTrace(true))
-- Status: print(GetMod("utpp_vehiclecam").Status())
-- A/B (not persisted): SetOwnHandoverMatch(false) = engine blend only at the handover, SetExitPull, SetHandoverBlend,
-- SetPullPath, SetPriorityHold, SetHoldRelease(s), SetCamTrace, SetExitMatch, SetExitMatchTime(s), SetExitFollow,
-- SetRideWatchdog, SetOwnHandoverGate.
--
-- Design notes:
-- Mount handover: the view is handed to the vehicle camera only after the vehicle camera's own blend (cameraBlendTime, about
-- 0.45 s, read per vehicle; vehBlendDefault when unread) has finished (+ keepHandoverMargin), at least keepHandover (0.25 s)
-- and at most keepHandoverMax; handing it over earlier showed 1-2 frames just behind the roof and a ~0.3 s pull back.
-- A/B (not persisted): SetHandoverBlend(false) = the fixed 0.25 s handover.
-- Mount pull-out: the start pose is frozen in the vehicle frame at the pull start (pullPath) and the camera travels around the
-- vehicle (cylindrical: angle / radius / height) to the target, looking at the vehicle; mixing the camera's LIVE pose - which
-- follows V into the car - with the target made the straight path cross the roof.
-- The vehicle's TPP camera becomes the rendered camera the moment the vehicle takes the perspective and shows its own 0.45 s
-- FPP -> TPPFar transition (start above the roof). Hence (priorityHold) the camera is kept at high priority before the
-- request (vanilla device cameras do the same) and handed over with a blend (holdRelease) - the engine blends from the pull-out
-- pose to the finished vehicle TPPFar camera - after which the priority is cleared.
-- Exit: the vehicle camera's TPPFar -> FPP transition + interior were visible before the third-person camera was back, so the
-- camera takes the view at the exit pull pose in the first exit frame (early exit camera). Active camera trace (camTrace)
-- is logged around both. A/B (not persisted): SetPullPath, SetPriorityHold, SetHoldRelease(s), SetCamTrace.
-- Exit match: the early exit camera first moved the active camera to the vehicle-behind pose while the view still showed the
-- vehicle TPPFar view until the next activation (a cut). (exitMatch) The camera now starts at the game camera's pose - the active
-- camera before ours is activated (the rendered vehicle TPP camera) first, else the player's vehicle TPP camera component
-- "vehicleTPPCamera" (player.script OnRequestComponents; it stays at the seat and follows V out, so it is rejected inside
-- exitMatchMinDist) - and blends from it to the vehicle-behind pose over exitMatchTime (match then blend). The exit hold /
-- release pose is computed for the player pose of each write. A/B (not persisted): SetExitMatch(false), SetExitMatchTime(s).
-- The cam trace also logs the vehicle TPP camera (vtpp=) and utppCamera (utpp=) in the vehicle frame: which camera is rendered.
-- Bike pull-out (0.9 m per 50 ms around the bike) runs over bikePullTime 1.3 s (cars: pullTime).
-- Exit settle: (1) rides shorter than the mount release window: the pending "camera settle" ran 0-0.14 s after the exit start and
-- hid the early exit camera (view at the vehicle centre, then a 4.5-5.3 m jump); the exit start now flushes the pending release
-- (no settle / release / drop runs after it). (2) Quick exits: the unmount dropped the early camera (2.0-2.3 m jumps); it is now
-- kept until at most exitHoldAfterUnmount 0.5 s after the unmount / exitHoldMax 3.0 s after the exit start, and the exit release
-- starts when the hold ends.
-- Vehicle path on utppCamera: utpp_vehiclecam drives utppCamera through mount / ride / exit with the vehicle API of utpp_camera
-- (without it the standalone camera cut to the game's FPP camera at every mount and re-entered from the eyes after the exit).
-- jump-out follow ("jumping out of a car / bike: the camera stays at the
-- vehicle for a moment, V flies out of view, then correct TPP"): the vehicle-behind pose was kept until the unmount +
-- exitHoldAfterUnmount while V was launched away from the (still moving) vehicle. Now, as soon
-- as V is more than exitFollowDist (2.0 m) from the vehicle centre, or the unmount completes after an exit while the
-- vehicle moved faster than exitFollowSpeed (4 m/s = jump-out), the hold target blends over exitFollowBlend (0.5 s) from
-- the vehicle-behind pose to the orbit pose around V (utpp_camera.OrbitPoseAt: orbit yaw, the hold's pitch kept); the
-- hold / release timing is unchanged, normal exits (V stays within 2 m, vehicle standing) keep the vehicle-behind hold.
-- Log "exit follow (...)" / "exit while the vehicle moves ..."; A/B (not persisted): SetExitFollow(false).
-- (1) the seat decision waits until the seat is known (a non-driver
-- mounting slot during the mount transition is "pending"; the passenger rule needs state 3 or a non-driver slot once
-- seated); a hold taken while seated starts the pull-out at once. (2) Driver combat (state 6) keeps mount / ride / chase
-- (the game maps the vehicle camera to DriverCombat TPP); seat states 2/3/5/7 with a TPP vehicle camera hand the mount
-- hold to the game camera ("togame") instead of the first-person rules. (3) Keep "game": the game's vehicle camera has
-- the view while mounted; the exit is taken from it (TakeForVehicleExit, utpp_camera vehicle API >= 2). (4) Setting
-- vehicle.forceTppOnEnter: every driver entry requests a third-person vehicle camera (see M.DEFAULTS).
-- An exit during the hand-over engine blend matches the blended pose (blend 0); bikes accept a close
-- TPP vehicle camera behind the bike as the exit match; the matched camera's FOV blends to the profile FOV; earlyExitBlend
-- 0.30 s (0.20 s from a moving vehicle); the exit follow also starts on a launch after the unmount (capsule above the
-- seat height) and on bikes beyond 1.2 m, blends 0.25 s, clamps its pitch and caps its per-frame change; bikes use a closer
-- pull-out target, a 0.65 s hand-over and a 0.2 s hold after the unmount; the seated TPP perspective is requested again
-- during the exit (exitKeepVehicleTpp); vehicle camera changes in keep "game" call M.onVehiclePerspective; setting
-- vehicle.safeTeleportExit is pushed to Game.UTPPSetSafeTeleportExit.
-- passenger-side entry:
-- V mounts seat_front_right, the vanilla SwitchSeats state moves her to
-- the free driver seat (PSM Vehicle 4 for 3.48 s, slot seat_front_right until the Drive state, the first state-1 frame
-- still reads it). The decision waited for a driver slot, so no hold was taken (no pull-out, the orbit followed V into
-- the car), the first state-1 frame applied the passenger rule and the next one handed the view to the game camera
-- ("keepTpp off -> game") with a cut. Now the hold is taken at the mount start when the switch is expected
-- (seatSwitchHold), PSM states 1/6 decide the driver seat (the slot only before them), and the ride runs as a driver
-- entry: request, hand-over after the vehicle camera blend.
-- mount pivot continuity: the game's bike mount
-- puts V on the seat slot in one frame; utpp_camera now blends its pivot from the last shown position (MountBlendActive,
-- 0.38 s). The near pull-out trigger (0.2 s, V within pullDist) waits for that blend to end, so the path is frozen from a
-- camera at rest (without the wait the shown camera stopped from about 4 m/s in one frame); pullStart is unchanged.
-- car camera (cars only: positively identified by IsA, bikes / unknown class keep the above):
-- (1) entry: the vehicleTPPCamera component reads at V's seat, so the hand-over never matched and the engine blend zoomed out
-- 3.5 m in 0.45 s (:1453-1471). utppCamera now glides (0.9 s, eased, position / yaw / pitch / FOV in the vehicle frame) from the
-- pull-out pose to the vehicle camera's rest pose as soon as the request phase is done, then hands over with a residual
-- blend (0.20 s for a read / learned pose, 0.45 s for the class default); no target (moving vehicle, unknown perspective) =
-- engine blend 0.90 s. The target is learned per vehicle record and perspective from the settled camera after the release
-- (standing vehicle only). The class default (0,-9.00,2.50) pitch -9 FOV 69 is the Archer Hella TPPFar at rest; the
-- (-0.00,-5.55,1.70) pitch 0 exit read at :507 is the motorcycle of ride 1 (:329 'bike mount'), not the same camera.
-- (2) exit: the native exit forces the vehicle camera to FPP 16 ms after the exit start, so Activate(0.30) mixed in a camera
-- moving into the cabin (1.2 m lurch) and the 0.70 s match moved 3.8 m at ~7 m/s. A take from the rendered camera matched in the
-- same frame is a cut; an orientation reading 0 / 0 keeps a 0.12 s blend; the match lasts distance / 4 m/s (0.7-1.0 s). The
-- exit keep requests are off for cars (diagnostic frame count kept). (3) every entry starts in TPP: passengers get the request
-- after the vanilla PassengerEvents.OnEnter FPP one (then keep "game"), a ride whose mount hold timed out gets it at its first
-- Driving frame. A/B (not persisted): SetHandGlide, SetExitTakeBlendMatched, SetExitMatchScale, SetForceTppPassenger.

local M = {}

M.PERSP = { FPP = 0, TPPClose = 1, TPPMedium = 2, TPPFar = 3 }
M.PERSP_NAME = { [0] = "FPP", [1] = "TPPClose", [2] = "TPPMedium", [3] = "TPPFar",
                 [4] = "DriverCombatClose", [5] = "DriverCombatMedium", [6] = "DriverCombatFar" }

M.DEFAULTS = {
  enabled = true,
  perspective = -1,       -- -1: persistent vehicle perspective if it is TPP, else TPPClose; 1..3: fixed
  -- request only once the vehicle PSM is in Driving (Vehicle = 1), like vanilla DriveEvents.OnEnter (which sends
  -- TPPClose there when requestedTPPCamera is set); 10 of the 16 requests were sent during the mount
  -- transition (Vehicle = 4) and none had an effect.
  waitDrive = true,
  -- optional vehicle camera blend override in s (nil = untouched; opt-in, SetBlendTime). Read value is logged per ride.
  blendTime = nil,
  -- keep the third-person camera and full body while entering (UTPP setting, default on)
  keepTpp = true,
  keepTimeout = 6.0,      -- s from the mount start without reaching Driving -> the camera returns to its own rules
  keepHandover = 0.25,    -- s after the vehicle TPP perspective is active (phase done/failed) before the handover
  keepHandoverBlend = true, -- handover after the vehicle camera blend (cameraBlendTime read per ride) + margin, >= keepHandover
  keepHandoverMargin = 0.08,
  keepHandoverMax = 1.0,
  vehBlendDefault = 0.45, -- s: Camera.VehicleCameraManager_Default.cameraBlendTime (0.449 in game) when the read fails
  -- requested every frame from the first Driving frame; ~50 identical requests in 0.3 s,
  -- 139 in one session, while the vehicle accepted ~0.29 s after the FIRST one -> one request, retry every 0.25 s, bounded.
  keepRequestInterval = 0.25,
  keepMaxRequests = 12,
  keepExitTimeout = 4.0,      -- s: exit hold without leaving the vehicle -> the camera returns to its own rules
  -- exit fix: the camera sat INSIDE the
  -- car behind / at V's head for ~1.1 s (collision shortens the boom while V is still seated), after the vehicle camera
  -- had begun its blend to FPP.
  exitPull = true,            -- exit hold: the camera kept at the vehicle-behind pose (as the pull-out) until the unmount
  exitPullTime = 0.8,         -- s: blend from that pose to the orbit after the unmount
  -- pull-out: in the last part of the enter animation the orbit camera followed the character into the car
  -- (head / steering wheel / roof from inside). The camera pose is blended towards
  -- a pose behind and above the vehicle (like the vehicle TPP camera) from pullStart s after the mount start (or when the
  -- player is within pullDist m of the vehicle centre) over pullTime s, until the vehicle camera takes over.
  pullOut = true,
  pullStart = 0.9,
  pullDist = 1.0,
  pullTime = 0.9,         -- mount jump fix: 0.6 -> 0.9 s (the path around the vehicle is ~6 m; done before Driving ~1.9 s)
  -- exit match 2: on a motorcycle the pull-out starts 0.2 s after the mount (V is
  -- next to the bike) and travels ~150 deg around it - 0.9 s gave 0.9 m per 50 ms. Bikes use bikePullTime (the first
  -- request came 1.2 s and the handover 2.0 s after the pull start; the high-priority hold keeps the pull visible).
  bikePullTime = 1.3,
  pullBack = 5.5,         -- m behind the vehicle centre
  pullHeight = 2.4,       -- m above the vehicle origin
  pullLookHeight = 1.0,   -- m, look-at point above the vehicle origin
  -- ('behind' mount): the path from a start on V's side of the car (the boom pulled in to 0.88 m from V) grows its radius
  -- along the angle of V and passed 0.23 m from her head (hair filled the frame). pullClearR (m, 0 = off, the previous path exactly): the path position is
  -- pushed out horizontally to this distance from V's axis while it is between pullClearZ0 and pullClearZ1 m above her feet (a path that stays further
  -- away is unchanged).
  -- the push keeps one side for the whole pull (pullClearSide = 0: the radial push) and the height band fades over pullClearRamp m outside
  -- both edges (0 = the hard band): no step in the pushed point where the raw path passes near V's axis or crosses a band edge.
  -- (a mount from behind): V slides into the seat (about 0.6 m in 0.3 s) while the path starts, and V's live
  -- position moved toward the frozen start: the fade-in of the clearance began from the LIVE distance of the start point to V (0.74 -> 0.1 m), so the camera
  -- passed 0.23 m from her head (0.8 m clearance, 0.28 m measured against V's position in the log). pullClearFreeze (false = the live distance): the fade
  -- starts from the distance of the start point to V at the first frame of the pull, so the camera never comes closer to V than that (nor than the clearance
  -- once the fade is over) while V moves toward the start; the push is radial from V, so the camera keeps its distance to V instead of closing in on her.
  -- pullClearR 1.0 (was 0.8): the mount camera watch (utpp_camera, mountCamNear) wants the camera at least 1 m from V through the hand-over.
  pullClearR = 1.0, pullClearZ0 = -0.3, pullClearZ1 = 2.1, pullClearFade = 0.3, pullClearRamp = 0.25, pullClearSide = 1, pullClearFreeze = true,
  -- (entry review, 'from behind' mounts of utility4 / standard25 / standard3): (1) the shown yaw was a shortest-arc lerp towards a look-at yaw
  -- that turns with the path, so the arc flipped when the look-at yaw passed 180 deg from the frozen orbit yaw (a jump of 360 deg x w: 168 deg at w = 0.5,
  -- 70 deg at w = 0.23); the look-at yaw is now unwrapped along the path. (2) The frozen start can lie inside the car (0.22 / 0.36 m from the origin: the
  -- orbit follows V into the cabin during the first 0.9 s) and the cylinder path (radius linear from there) crossed the body and the roof. pullHull: a keep-out
  -- box per vehicle record (hullRecords, hullDefault: half width hx, half length front / back hyF / hyB, top above the origin; metres in the vehicle frame).
  -- A start inside the box (hullPlanDepth) keeps the cylinder path when that leaves the box first, else it leaves through V's side (a straight leg of pullExitFrac of the
  -- path to a point beside the car, at most pullExitBehind m behind V's position), then runs the arc around the outside. The path radius is kept at the polar radius
  -- of the box (hullClampMargin, from the weight where an inside start has left the box) while the camera is below its top (a hullRamp m fade above);
  -- hullPushMargin: a final push of the clearance result out of the box. pullEscape: the pull-out starts as soon as the orbit camera is deeper than
  -- pullEscapeDepth inside the box (pullEscapeMin s after the mount) instead of pullStart. hullWarnDepth: the rendered camera deeper than this inside the box
  -- is logged (camera inside the vehicle box WARNING) and counted in the continuity summary. Cars only (a motorcycle keeps the path as it was). Box sizes are
  -- estimates (heights from V's feet and the frames); a box that reads a clean entry as inside shows up as a WARNING there.
  pullHull = true, pullEscape = true, pullEscapeMin = 0.2, pullExitFrac = 0.2, pullExitBehind = 1.1, pullExitSide = 0.4,
  hullClampMargin = 0.15, hullPushMargin = 0.05, hullRamp = 0.1, hullWarnDepth = 0.05, hullPlanDepth = 0.05, pullEscapeDepth = 0.1,
  -- ('behind' mounts of standard3 / utility4): the hold camera rests against the closed door (boom shortened by the body), the door opens
  -- right after the mount and the camera follows it into the cabin at about 2.2 m/s; pullEscape fired only once it was pullEscapeDepth deep and
  -- pullEscapeMin s after the mount, the rendered camera was in the box for 0.12 - 0.33 s (0.14 / 0.29 m deep). pullEscapeLead: the orbit camera that
  -- moves toward the box and would be closer than pullEscapeAhead m to its face (or inside) within pullEscapeLead frames at its current per-frame
  -- approach starts the pull-out at once (no pullEscapeMin wait, the pivot blend still waits), so the path is frozen outside the box; 0 = off.
  pullEscapeLead = 3, pullEscapeAhead = 0.03,
  -- the eased pull weight follows the arc length of the actual path (clearance detour around V, the exit leg, the box push included; sampled once at
  -- the freeze with V there, pullArcSamples points): the former path weight ran through the detour around V at 1.4 x the planned peak (Bratsk 9.7 ->
  -- 13.5 m/s) and through the exit leg at 1.1 x. The duration then uses the length of that path. false = the former path weight. A start inside the box
  -- (the fallback when the early pull-out cannot fire) keeps the former weight: the arc-length weight leaves the box later than it did. 256 samples: the
  -- 144 fps peak of the truck path stays at 6 m/s (128: 6.6 m/s).
  pullArcLength = true, pullArcSamples = 256,
  -- the hand-over glide of a car waits for a slowed pull-out to end (TryHandGlide); false = it composes with the pull-out tail
  handGlideAfterPull = true, handGlideAfterPullSlack = 0.3,
  -- the box push of a path that starts outside the box acts from the first frame (margin faded in over 0.1 of the weight); false = from w 0.03
  hullPushEarly = true,
  -- (sport1 'behind' mount: the walking camera rested over the rear deck at veh (-0.90,-2.18,0.76), the flat box read
  -- it 0.12 m inside for 4 frames while it is clear of the body): topY = the top of the box along its length, sorted { y, top } pairs (vehicle
  -- frame), linear between them and the end values beyond the ends (M.HullTop); top stays the highest point. A record without topY keeps the flat top.
  -- sport1: the body shell (ext01_body_01 + glass, Base-bone frame; the vehicleTPPCamera at the seat
  -- (-0.36,-0.41,0.03) = the extracted seat (-0.36,-0.41,-0.01)) is 0.92 m high over the cabin (y -1.0 .. 0.0) and falls to 0.47 - 0.58 m at the rear end
  -- and 0.46 - 0.51 m over the bonnet (surface samples of the uncropped meshes): the profile is at least 0.1 m above the shell of every 0.2 m slice
  -- (the flat box had 0.13 m over the roof). The rear wing is not part of the extracted body (the camera at 0.76 m over the deck was clear of
  -- it; the profile there is 0.70 m). hullProfile = false: the flat top of every record.
  hullProfile = true,
  hullDefault = { hx = 1.15, hyF = 2.6, hyB = 2.6, top = 1.7, bottom = -1.2 },
  hullRecords = {
    ["Vehicle.v_standard2_archer_hella"] = { hx = 1.0, hyF = 2.3, hyB = 2.3, top = 1.5 },
    ["Vehicle.v_standard25_villefort_columbus"] = { hx = 1.1, hyF = 2.7, hyB = 2.7, top = 1.4 },
    ["Vehicle.v_standard3_chevalier_emperor"] = { hx = 1.1, hyF = 2.8, hyB = 2.8, top = 1.5 },
    ["Vehicle.v_sport1_quadra_turbo_r"] = { hx = 1.05, hyF = 2.3, hyB = 2.3, top = 1.05,
      topY = { { -2.30, 0.68 }, { -2.00, 0.72 }, { -1.20, 0.88 }, { -1.15, 1.05 }, { 0.05, 1.05 }, { 0.45, 0.85 }, { 1.00, 0.64 }, { 2.30, 0.62 } } },
    ["Vehicle.v_utility4_kaukaz_bratsk"] = { hx = 1.5, hyF = 5.6, hyB = 4.5, top = 2.3 },
  },
  bikePullClearR = 0,     -- motorcycles keep their path as it was (no clearance)
  pullPitchSign = 1.0,    -- EulerAngles pitch sign (positive = up); -1 flips if the camera looks the wrong way
  -- mount jump fix (see the header): (a) path around the vehicle from the pose frozen at the pull start
  pullPath = true,
  -- (b) the camera kept in front of the vehicle camera from the first request to the handover, released with a blend
  priorityHold = true,
  holdRelease = 0.45,     -- s: blend of the handover (0 = cut)
  holdReleaseMargin = 0.10, -- s after the release blend
  -- s: activation blend from the vehicle TPP camera (still at TPPFar in the first exit frames);
  -- 0.20 -> 0.30 (the take started abruptly from a moving camera)
  earlyExitBlend = 0.30,
  jumpExitBlend = 0.20,   -- s: the same from a vehicle faster than exitFollowSpeed (V leaves the vehicle camera's view)
  -- exit settle fix: quick exits unmount (~1.05 s) before the early camera is released;
  -- it is kept up to exitHoldAfterUnmount s after the unmount, at most exitHoldMax s after the exit start, instead of being
  -- dropped at the unmount (2.0-2.3 m jumps)
  exitHoldAfterUnmount = 0.5,
  exitHoldMax = 3.0,
  -- exit match fix: the camera starts at the game camera pose (active camera = the rendered vehicle TPP camera, else
  -- the vehicleTPPCamera component) and blends to the vehicle-behind pose; poses closer than exitMatchMinDist m
  -- (horizontal) to the vehicle origin are rejected (FPP / interior)
  exitMatch = true,
  exitMatchTime = 0.70,
  exitMatchMinDist = 1.8,
  camTrace = false,       -- log the active (rendered) camera in the vehicle frame around the mount / exit (<= camTraceHz)
  camTraceHz = 20,
  camTraceAfter = 1.0,    -- s after the handover / unmount
  joDiagHz = 20,          -- 'jo diag' lines per s (jump-out push diagnostic, only with camTrace, see M.JoDiagTick)
  armWindow = 8.0,        -- s: give up if no TPP perspective after this long from the ride start
  requestInterval = 0.25, -- s between requests while waiting
  maxRequests = 16,
  settleAfterDrive = 2.0, -- s after the first Driving frame without TPP -> give up
  ownHandoverMatch = true,    -- before the handover: blend to the vehicleTPPCamera pose when it reads outside the vehicle
  ownHandoverMatchTime = 0.25,
  -- car entry glide: the vehicleTPPCamera component reads at V's seat, so the match
  -- never engaged and the engine blend zoomed out 3.5 m in 0.45 s. Cars only (positively identified: vehCar): once the
  -- pull-out is over and the vehicle TPP perspective is active, utppCamera itself glides (eased, position / yaw / pitch /
  -- FOV, in the vehicle frame) to the vehicle TPP camera's rest pose, then hands over with a short residual engine blend.
  -- Target: a pose read outside the vehicle (ownHandoverMatch) > the pose learned this session for this vehicle record and
  -- perspective (vehTppLearn) > the class default vehTppRest (measured: Archer Hella TPPFar at rest) > none
  -- (engine blend lengthened to handFallbackBlend). A/B (not persisted): SetHandGlide(false) = the previous hand-over.
  handGlide = true,
  handGlideTime = 0.9,
  handGlideSpeed = 5.0,           -- m/s: peak speed of the glide; a longer way takes longer (up to handGlideTimeMax)
  handGlideTimeMax = 1.7,
  handGlideResidualBlend = 0.20,  -- Deactivate blend after a glide to a read / learned pose
  handGlideDefaultBlend = 0.80,   -- ... after a glide to the class default / an unconfirmed learned pose (unverified for this
                                  -- vehicle: a larger residual, 0.45 -> 0.80 s: residuals of 0.25-2.5 m seen)
  handFallbackBlend = 0.90,       -- Deactivate blend of a car hand-over without a target pose
  handGlideMaxSpeed = 2.0,        -- m/s: a faster vehicle skips the glide (the rest pose is not valid while driving)
  handGlideMaxDist = 8.0,         -- m: sanity limit of target - shown pose
  handGlideMaxRadius = 15.0,      -- m: sanity limit of the target's horizontal distance
  vehTppLearn = true,
  vehTppLearnSpeed = 0.5,         -- m/s: learning needs a standing vehicle
  vehTppLearnStable = 0.2,        -- s: the active camera must stay within vehTppLearnTol this long
  vehTppLearnTol = 0.05,          -- m
  vehTppLearnAzimuth = 8,         -- deg: a camera orbited sideways by the player is not the rest pose
  vehTppLearnLookHeight = 1.05,   -- m above the vehicle origin: look-at point for an unread orientation (tweak lookAt z)
  -- (Columbus / Emperor / Turbo-R / Bratsk rides): the pitch of the
  -- class default and of every learned pose used to be computed to a look-at point 1.05 m above the vehicle origin (-8 ...
  -- -15 deg), but the settled vehicle camera reads pitch 0 (forward and transform) on every car, the Hella TPPFar included
  -- (read pose (0.02,-9.00,2.50) pitch 0). The glide therefore ended 8-15 deg off and the release turned the view in
  -- 0.2-0.45 s (the jump at the handover). vehTppRestPitch = the pitch of an unread
  -- orientation (nil = the old look-at formula, A/B).
  vehTppRestPitch = 0,
  vehTppRest = { car = { [3] = { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = 0, fov = 69,
                                 src = "measured default (Archer Hella TPPFar at rest)" } } },
  -- measured rest poses per vehicle record and perspective (settled active camera, standing vehicle; yaw 0,
  -- pitch 0 as read). The class default above is the Hella's: Columbus 0.25 m, Emperor 0.61 m, Turbo-R 1.04 m and Bratsk
  -- 2.50 m away from it, so a first ride of those glided to the wrong place. A record outside this table (and not yet
  -- learned this session) still uses the class default, with the longer handGlideDefaultBlend.
  vehTppRestRecords = {
    ["Vehicle.v_standard2_archer_hella_player"] = {
      [1] = { x = 0.0, y = -4.75, z = 1.25, yaw = 0, pitch = 0, fov = 69, src = "measured (log 11-05-04)" },
      [3] = { x = 0.0, y = -9.00, z = 2.50, yaw = 0, pitch = 0, fov = 69, src = "measured (log 07-50-12)" } },
    ["Vehicle.v_standard25_villefort_columbus"] = {
      [3] = { x = 0.0, y = -9.00, z = 2.75, yaw = 0, pitch = 0, fov = 69, src = "measured (log 15-00-36)" } },
    ["Vehicle.v_standard3_chevalier_emperor"] = {
      [3] = { x = 0.0, y = -9.50, z = 2.85, yaw = 0, pitch = 0, fov = 69, src = "measured (log 15-00-36)" } },
    ["Vehicle.v_sport1_quadra_turbo_r"] = {
      [3] = { x = 0.0, y = -8.00, z = 2.20, yaw = 0, pitch = 0, fov = 69, src = "measured (log 15-20-28)" } },
    ["Vehicle.v_utility4_kaukaz_bratsk"] = {
      [3] = { x = 0.0, y = -11.00, z = 4.00, yaw = 0, pitch = 0, fov = 69, src = "measured (log 15-20-28)" } },
  },
  -- Deactivate blend of a glide to a pose measured for this record (the residual of a verified pose is ~0)
  handGlideRecordBlend = 0.20,
  -- pull-out speed: the path around the vehicle took 0.9 s for any sweep (Columbus / Emperor / Bratsk: 7-19 m/s,
  -- 90-150 deg in about 0.5 s, V still getting in). The duration now grows with the frozen path: peak speed (1.5 x the mean
  -- of the eased path) <= pullMaxSpeed, peak turn rate <= pullMaxTurnRate, never below pullTime, at most pullTimeMax.
  -- 7.0 m/s (0.117 m per frame at 60 fps) -> 6.0 m/s (0.10 m), and the cap 2.2 -> 4.0 s: the 2.2 s cap held the 14 m pull-out around the
  -- truck at a 9.7 m/s peak (0.16 m per frame); at 6 m/s it takes about 3.7 s and the hand-over glide waits for its end (handGlideAfterPull)
  pullScale = true,
  pullMaxSpeed = 6.0,       -- m/s
  pullMaxTurnRate = 110,    -- deg/s around the vehicle
  pullTimeMax = 4.0,        -- s
  -- exit follow of a standing car: (a) the follow fired at the exit start
  -- for a seat that is 3.7 m from the origin of a long truck (the 2.0 m gate is not per vehicle) with a 14.8 m offset blended
  -- in 0.25 s along a straight line through the cab; (b) a 2.3 m offset was crossed in 0.25 s. Now the gate is relative to
  -- the distance at the exit start, a standing car's follow lasts offset / exitFollowStandSpeed (clamped) and runs around the
  -- vehicle (vehicle frame cylinder: angle, radius, height). Jump-outs, launches and bikes keep the offset-carrying blend.
  exitFollowAround = true,
  exitFollowSlack = 1.2,        -- m beyond the exit start distance before the gate fires
  exitFollowStandSpeed = 4.5,   -- m/s: mean speed of a standing car's follow
  exitFollowStandMax = 2.0,     -- s
  exitFollowAroundSeat = 2.5,   -- m: a car seat farther than this from the origin follows around the vehicle at the unmount
  exitFollowHoldMax = 4.5,      -- s after the exit start: cap of the hold while a follow around the vehicle runs
  -- a jump-out by the speed whose car is at rest at the unmount (below exitFollowRestSpeed m/s) follows around the vehicle (see UpdateExitFollow)
  exitFollowAroundAtRest = true,
  exitFollowRestSpeed = 0.5,    -- m/s
  exitFollowRestMean = 2.5,     -- m/s: mean speed of that follow (eased peak 3.75 m/s, below the 5 m/s exit pose governor of utpp_camera)
  -- car exit: the native exit forces the vehicle camera to FPP 16 ms after the exit
  -- start, so Activate(earlyExitBlend) mixed in a camera moving into the cabin (1.2 m lurch), and the 0.70 s match then
  -- moved 3.8 m (7 m/s peak). Cars: a take from the rendered camera matched in the same frame is a cut (blend
  -- exitTakeBlendMatched); an orientation that reads 0 / 0 (canonical forward, pitch not proven) keeps a short blend
  -- exitTakeBlendUnsure; the match duration scales with its distance (exitMatchSpeed m/s average, clamped).
  exitTakeBlendMatched = 0.0,
  exitTakeBlendUnsure = 0.12,
  exitMatchScale = true,
  exitMatchSpeed = 4.0,
  exitMatchTimeMin = 0.70,
  exitMatchTimeMax = 1.0,
  -- the exit keep requests of cars are no-ops (the native exit forces the vehicle camera to FPP): off; the diagnostic count
  -- of exit frames in vehicle FPP stays. Other vehicles keep exitKeepVehicleTpp (A).
  carExitKeepVehicleTpp = false,
  -- an aborted car exit (exit -> ride) asks for the entry's TPP perspective again (the native exit left the camera in FPP)
  abortExitRequest = true,
  abortExitWindow = 1.0,
  -- every entry starts in third person, passengers included: PassengerEvents.OnEnter queues an FPP request itself, the
  -- TPP request follows once the vehicle reports FPP in the passenger state (passengerRequestDelay s after its first frame)
  forceTppPassenger = true,
  -- passenger rides with the mount hold stay on utppCamera's vehicle-behind chase pose (the passenger's vehicle camera does not
  -- leave FPP: observed 10 s with active=0 after the request). Needs pullOut; A/B SetPassengerChase(false) = the request only
  passengerChase = true,
  passengerRequestDelay = 0.15,
  passengerSettle = 3.0,          -- s of the passenger state watched: a TPP reading inside it can still be followed by the FPP one
  passengerTppHold = 0.5,         -- s of a steady TPP reading that ends the request (a later FPP switch is the player's)
  ownRideFppGrace = 0.3,      -- s of a vehicle FPP perspective in the ride before the view is left to the game camera
  -- jump-out follow (exit hold): V beyond exitFollowDist m from the vehicle centre, or the unmount after an exit
  -- with the vehicle faster than exitFollowSpeed m/s -> hold target blends to the orbit around V over exitFollowBlend s
  exitFollow = true,
  exitFollowDist = 2.0,
  exitFollowSpeed = 4.0,
  -- the native jump-out launch (Game.UTPPVehicleJumpOutEpoch: the flight after an unmount, also from a standing or slow
  -- vehicle with the double F) arms the jump-out landing mode when the speed classification above did not. false = speed only.
  exitJumpNative = true,
  -- jump-out side push (setting vehicle.jumpOutSidePush, default on; false = the vanilla flight exactly as before, no
  -- event at all): the standing-car jump-out (double F) drops V straight down ~0.16 m outside the body side (car-frame
  -- x -1.15 against the body side -0.99), the capsule touches the fender on the way down and the depenetration shoves her 1.8 m
  -- sideways right before the landing. At the native launch edge V gets a small outward velocity (Game.UTPPQueuePlayerImpulse: the
  -- vanilla PSMImpulse, utpp_vehicle_exit.reds): jumpOutPushSpeed m/s along the car's side axis, toward the side she is on. Only for
  -- a positively identified car that moves slower than jumpOutPushMaxCarSpeed m/s, with V at jumpOutPushMinX..MaxX m from the car
  -- origin sideways (below the minimum the side is unclear; beyond the maximum she is clear of the body already) and within
  -- jumpOutPushMaxY m along the car (a moving car leaves her behind it, no push).
  jumpOutSidePush = true,
  jumpOutPushSpeed = 0.65,
  jumpOutPushMaxCarSpeed = 3.0,
  jumpOutPushMinX = 0.5,
  jumpOutPushMaxX = 1.8,
  jumpOutPushMaxY = 2.6,
  exitFollowBlend = 0.25, -- was 0.5 s, now 0.25 s (V left the frame during the slower blend)
  -- The hand-over went out with no readable vehicle
  -- camera pose (vehicleTPPCamera at the seat) and the rendered camera then stayed fixed in the world while the car drove
  -- off (cam veh= drifted 0 -> 7.4 m). Ride watchdog (vehicle.rideWatchdog, default on): after the hand-over, while the
  -- vehicle moves faster than rideWatchdogSpeed m/s, the active camera must move at least rideWatchdogMove m per
  -- rideWatchdogTime s; else (1) one TPPFar request is sent again, (2) utppCamera takes the view back on the chase pose
  -- behind the vehicle ("retake", keep mount -> hand-over tried again), up to rideWatchdogMax retakes, then (3) the chase
  -- pose is kept for the ride (keep "chase", exit from it as from the mount hold). Hand-over gate (ownHandoverGate, A/B
  -- SetOwnHandoverGate, default OFF: in game the component always reads at the seat, so the gate only delays): no pose ->
  -- utppCamera keeps the chase pose and the pose is read again every ownHandoverGateRetry s for ownHandoverGateMax s.
  rideWatchdog = true,
  rideWatchdogMove = 0.2,   -- m
  rideWatchdogSpeed = 2.0,  -- m/s
  rideWatchdogTime = 0.3,   -- s
  rideWatchdogMax = 2,      -- retakes per ride (one request retry before the first retake)
  ownHandoverGate = false,
  ownHandoverGateMax = 2.0, -- s
  ownHandoverGateRetry = 0.25,
  -- every driver entry starts in a third-person vehicle camera (setting
  -- vehicle.forceTppOnEnter, default on). Rides without the UTPP mount hold (passive) run the same request loop from the
  -- first Driving frame. The requested value is rideBase[bike|car] = the TPP perspective UTPP requested last in this
  -- session (not persisted), else the persistent 1-3 (4-6 mapped to 1-3), else TPPClose. The player's camera choice
  -- during the ride (FPP included) is honoured and never stored; the next entry's request replaces it. Nothing is
  -- requested in states 2/3/5/7, for passengers or in scenes (PSM HighLevel 3-6). The game's settings file is untouched.
  forceTppOnEnter = true,
  forceSceneMin = 3, forceSceneMax = 6, -- PSM HighLevel scene tiers without a forced request
  -- keep "game": the game's vehicle camera has the view while still mounted (seat states 2/3/5/7 with a TPP vehicle
  -- camera, vehicle camera FPP in the ride, or a driver ride without the hold); the exit is taken from it through
  -- utpp_camera.TakeForVehicleExit (vehicle API >= 2), else the camera re-enters on its own rules as before
  togameBlend = 0.30,   -- s: mount / chase hold handed to the game camera (HoldForVehicle(false, why, "game", blend))
  gameTakeBlend = 0.25, -- s: exit take-over from a TPP vehicle camera at a readable pose (0 = one cut from FPP)
  -- Bike exits:
  -- (1): a close bike TPP camera (TPPClose about 1.55 m) is a valid outside pose: accepted from bikeExitMatchMinDist
  -- when the vehicle manager reports TPP and the pose is behind the bike (y < bikeExitMatchBehind)
  bikeExitMatchMinDist = 1.0,
  bikeExitMatchBehind = -0.8,
  -- (3): FOV continuity at the exit take-over (the active camera FOV read at the match, blended to the profile FOV
  -- with the match weight; only when a reader exists, else "fov unread"). A/B (not persisted): SetExitMatchFov(false)
  exitMatchFov = true,
  -- jump-out / launch follow after the unmount: V's capsule more than exitFollowRise m above the vehicle origin +
  -- the seat height (bike / car), or V more than bikeExitFollowDist m from a bike; pitch while following clamped
  exitFollowRise = 0.6,
  bikeSeatHeight = 0.9, carSeatHeight = 0.6,
  bikeExitFollowDist = 1.2,
  exitFollowPitchMin = -20, exitFollowPitchMax = 5,
  bikeExitHoldAfterUnmount = 0.2,
  -- (4): caps of the follow output per frame (deg/s and m per frame)
  exitFollowMaxRate = 180,
  exitFollowMaxStep = 0.35,
  exitFollowMaxSpeed = 15, -- m/s of the framing change relative to V (0.25 m per frame at 60 fps: <= 0.8 m per 50 ms)
  -- launch follow (jump-out only: ctrl.exitJump, by the speed or the native launch): the offset between the vehicle-behind pose
  -- and the orbit around V collapses in 0.25 s, which on top of V's own climb made the camera rise at 15 - 17 m/s (V: 9.7 m/s) with
  -- +92 / -120 m/s^2. The vertical position of the follow relative to V is rate (m/s) and acceleration (m/s^2) limited and eases into
  -- the orbit (stopping distance bound); 0 = off (the previous output exactly). Other follows (standing car, bike walk-off) never use it.
  -- frame stall: time beyond blendDtCap s of one frame does not advance the pull-out / exit pull / follow blends (a 83 - 120 ms frame
  -- moved them by 10 - 15 % at once: 0.535 m and 0.602 m / 11.6 deg catch-up steps in the game logs). Every frame up to the cap - all normal
  -- ones - gives exactly the previous value. 0 = off.
  blendDtCap = 0.04,
  exitFollowMaxVz = 4.0,
  exitFollowMaxAz = 60,
  exitFollowVzTau = 0.10,
  -- launch smoothing (jump-out only: ctrl.exitJump): the acceleration of the launch follow's height went 0 -> 60 m/s^2 within one frame (a jerk
  -- of thousands of m/s^3 at the start of the climb) and the framing change relative to V ran at up to 15 m/s (0.25 m per frame at 60 fps).
  -- exitFollowMaxJz (m/s^3, 0 = off, the previous acceleration limiter exactly): the acceleration of the follower changes by at most that per second. OFF by
  -- default: in the offline launch replay (test_utpp_camera_vehicle.lua, 20 - 144 fps) 300 - 1200 m/s^3 made the camera overshoot V's climb (12 - 13 m/s) and brake at
  -- -90 .. -150 m/s^2, with a larger pitch swing and a release speed step - worse than the 0 -> 60 m/s^2 onset it replaces.
  -- exitJumpMaxSpeed (m/s, 0 = off) and exitJumpMaxRate (deg/s, 0 = off): the caps of the framing change relative to V and of the follow's yaw / pitch
  -- rate during a jump-out follow (the min of these and exitFollowMaxSpeed 15 / exitFollowMaxRate 180; both are per second, so frame time aware). Other follows
  -- are unchanged. OFF by default: 11 m/s / 120 deg/s lowered the peak framing speed relative to V (13.8 -> 12.1 m/s moving, 11.9 -> 9.3 standing at 60 fps) but
  -- raised the pitch acceleration of the standing launch from 351 to 622 deg/s^2; the 0.677 m step was a vehicle-frame maximum on a render hitch
  -- (repeated frame in the video), not a camera discontinuity, so no cap is shipped as a fix.
  exitFollowMaxJz = 0,
  exitJumpMaxSpeed = 0,
  exitJumpMaxRate = 0,
  -- bike pull-out target (closer than the car's 5.5 / 2.4) and a longer hand-over blend for bikes
  bikePullBack = 3.6,
  bikePullHeight = 1.7,
  bikeHoldRelease = 0.65,
  -- A: keep the vehicle TPP perspective during the exit (the game switches the vehicle camera to FPP at the exit
  -- start, and the TPP representation drops about 0.47 s later). Not persisted; A/B SetExitKeepVehicleTpp(false).
  exitKeepVehicleTpp = true,
  exitKeepInterval = 0.10,
  exitKeepMax = 8,
  -- W9 safe teleport exit for motorcycles (setting vehicle.safeTeleportExit, pushed to Game.UTPPSetSafeTeleportExit)
  safeTeleportExit = true,
  -- car entry approach walk (setting vehicle.approachWalk, pushed to Game.UTPPSetVehicleApproachEnabled)
  approachWalk = true,
  -- passenger-side entry: the mount from seat_front_right is
  -- followed by the vanilla switch_seat to the free driver seat (state 4 for ~3.5 s, slot seat_front_right until the
  -- Drive state). The hold is taken at the mount start when that switch is expected (c.SeatSwitchExpected); the seat is
  -- final at PSM Vehicle 1/6 (driver) or 2/3/5/7 (passenger). keepTimeout (6 s) stays the safety cap of the mount hold.
  -- Not persisted; A/B SetSeatSwitchHold(false) = the decision waits for the seat as before.
  seatSwitchHold = true,
}

local function copy(t)
  local r = {}
  for k, v in pairs(t) do r[k] = v end
  return r
end
local function smooth01(t) t = math.max(0, math.min(1, t)); return t * t * (3 - 2 * t) end
local function wrap180(d)
  d = d % 360
  if d > 180 then d = d - 360 end
  return d
end
-- vehicle-frame pose blend { x, y, z, yaw, pitch } (yaw along the shorter arc)
local function vposeLerp(a, b, w)
  return { x = a.x + (b.x - a.x) * w, y = a.y + (b.y - a.y) * w, z = a.z + (b.z - a.z) * w,
           yaw = (a.yaw or 0) + wrap180((b.yaw or 0) - (a.yaw or 0)) * w,
           pitch = (a.pitch or 0) + ((b.pitch or 0) - (a.pitch or 0)) * w }
end
M.VPoseLerp = vposeLerp

-- camera continuity measure (camTrace): per-frame change of the rendered camera (vehicle frame pose
-- { x, y, z, yaw, pitch }) so a one-frame snap is measurable from the log. Pure; fed by the wiring every frame while a
-- trace window is open. Frames more than 0.25 s apart (a window gap) are not compared.
function M.NewContinuity()
  return { frames = 0, prev = nil, lineStep = nil, lineRot = 0, max = { step = 0, rot = 0 }, hand = { step = 0, rot = 0, frames = 0 },
           box = { checked = 0, frames = 0, depth = 0 } }
end
-- the rendered camera inside the keep-out box of the vehicle (M.HullFor): st.box counts the frames deeper than warnDepth inside it. Returns
-- the depth (m, > 0 inside) of this frame or nil when no box is given.
function M.ContinuityBox(st, t, pose, hull, warnDepth)
  if hull == nil then return nil end
  local b = st.box
  local d = M.HullDepth(hull, pose.x, pose.y, pose.z)
  b.checked = b.checked + 1
  if d > (warnDepth or 0.05) then
    b.frames = b.frames + 1
    b.first, b.last = b.first or t, t
    if d > b.depth then b.depth, b.at, b.x, b.y, b.z = d, t, pose.x, pose.y, pose.z end
  end
  return d
end
function M.ContinuityStep(st, t, pose, handAt, handDur, hull, warnDepth)
  M.ContinuityBox(st, t, pose, hull, warnDepth)
  local p = st.prev
  st.prev = { t = t, x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pose.pitch or 0 }
  if p == nil or t <= p.t or t - p.t > 0.25 then return nil end
  local d = math.sqrt((pose.x - p.x) ^ 2 + (pose.y - p.y) ^ 2 + (pose.z - p.z) ^ 2)
  local rot = math.abs(wrap180((pose.yaw or 0) - p.yaw)) + math.abs((pose.pitch or 0) - p.pitch)
  st.frames = st.frames + 1
  st.lineStep, st.lineRot = math.max(st.lineStep or 0, d), math.max(st.lineRot or 0, rot)
  local dt = t - p.t
  if d > st.max.step then st.max.step, st.max.at, st.max.dt = d, t, dt end
  if rot > st.max.rot then st.max.rot, st.max.rotAt = rot, t end
  if handAt ~= nil and t >= handAt - 0.15 and t <= handAt + (handDur or 0.45) + 0.3 then
    local h = st.hand
    h.frames = h.frames + 1
    if d > h.step then h.step, h.at, h.dt = d, t, dt end
    if rot > h.rot then h.rot, h.rotAt = rot, t end
  end
  return d, rot
end
function M.ContinuitySummary(st, clock)
  local m, h = st.max, st.hand
  local function sp(x) return (x.dt and x.dt > 0) and (x.step / x.dt) or 0 end
  return string.format("[utpp_vehiclecam] t=%.2f camera continuity: %d frames, max per-frame step %.3f m (%.1f m/s) at t=%s, max "
    .. "rotation %.1f deg at t=%s; handover window (%d frames): max step %.3f m (%.1f m/s) at t=%s, max rotation %.1f deg at t=%s",
    clock, st.frames, m.step, sp(m), m.at and string.format("%.2f", m.at) or "-", m.rot, m.rotAt and string.format("%.2f", m.rotAt) or "-",
    h.frames, h.step, sp(h), h.at and string.format("%.2f", h.at) or "-", h.rot, h.rotAt and string.format("%.2f", h.rotAt) or "-")
    .. M.ContinuityBoxText(st)
end
-- the WARNING line of the wiring is due for THIS frame's depth only (> warnDepth inside), at most one per 0.5 s (w = wiring state:
-- boxWarnAt). It printed whenever the count had grown since the last line, also on a later frame that was already outside (-0.59 / -1.88 m "inside"
-- the pose 0.2 s after the last frame inside).
function M.ContinuityBoxWarnDue(w, depth, clock, warnDepth)
  if not (depth and depth > (warnDepth or 0.05)) then return false end
  if w.boxWarnAt ~= nil and clock - w.boxWarnAt < 0.5 and clock >= w.boxWarnAt then return false end
  w.boxWarnAt = clock
  return true
end
-- (append-only) the box part of the summary: empty when no box was checked
function M.ContinuityBoxText(st)
  local b = st.box
  if b == nil or (b.checked or 0) == 0 then return "" end
  if b.frames == 0 then return string.format("; camera box: clear (%d frames)", b.checked) end
  return string.format("; camera inside the vehicle box WARNING: %d of %d frames (t=%.2f-%.2f), deepest %.2f m at t=%.2f veh=(%.2f,%.2f,%.2f)",
    b.frames, b.checked, b.first or 0, b.last or 0, b.depth, b.at or 0, b.x or 0, b.y or 0, b.z or 0)
end
-- jump-out push diagnostic (test aid, only with camTrace): V's stationary jump-out is pushed ~1.7 m radially away
-- from the car at up to 6.6 m/s right after a false ground contact next to the sill. The line
--   [utpp_vehiclecam] jo diag t=.. car=(x,y,z) vcar=(vx,vy,vz) onG=0|1 carV=.. vzw=.. jt=.. [door=..] [frz=1]
-- gives V in the CAR frame (x right, y forward, z up from the car origin; the live pose of the car entity, not the frozen
-- pose of the last mounted frame), V's velocity relative to the car in that frame, the car's horizontal speed and V's world
-- vertical speed. onG is DERIVED (the game flag is not readable from Lua): 1 from the first sample with |vzw| < 0.5 after a descent
-- below -2 m/s until V falls again (a false contact that is followed by a fall re-opens the flight). The window opens when the
-- jump-out is armed and V is out of the car, closes JO_DIAG_AFTER_LAND s after the landing (or JO_DIAG_MAX s after opening).
-- Pure state machine + formatter: the wiring only supplies the samples.
M.JO_DIAG_AFTER_LAND = 1.5
M.JO_DIAG_MAX = 6.0
M.JO_DIAG_DESCENT = -2.0
M.JO_DIAG_REST = 0.5
function M.NewJoDiag()
  return { open = false, done = false, t0 = nil, air = false, landAt = nil, tickAt = nil, prev = nil, lines = 0, misses = 0 }
end
-- gate: the camera trace flag, an armed jump-out (ctrl.exitJump) and V out of the vehicle
function M.JoDiagActive(cfg, exitJump, mounted)
  return cfg ~= nil and cfg.camTrace == true and exitJump == true and mounted ~= true
end
-- true when the next sample is due (throttle: joDiagHz lines per s; a clock that went back counts as due)
function M.JoDiagDue(st, clock, hz)
  local h = math.max(1, tonumber(hz) or 20)
  return st.tickAt == nil or clock < st.tickAt or clock - st.tickAt >= 1 / h - 1e-4
end
-- V (world x, y, z) in the car frame { x right, y forward, z up } of the car pose { x, y, z, yaw (deg) }; the same rotation as the
-- camera trace (c * rx - s * ry, s * rx + c * ry)
function M.JoDiagCarFrame(car, v)
  local rad = math.pi / 180
  local c, s = math.cos(-(car.yaw or 0) * rad), math.sin(-(car.yaw or 0) * rad)
  local rx, ry = v.x - car.x, v.y - car.y
  return c * rx - s * ry, s * rx + c * ry, v.z - car.z
end
-- One tick (call when JoDiagDue). sample = { car = { x, y, z, yaw }, v = { x, y, z }, frozen = bool } or nil (unreadable);
-- door = optional text. Returns the log line or nil (window closed / not active / nothing readable).
function M.JoDiagTick(st, clock, active, sample, door)
  if not active then
    st.open, st.done, st.t0, st.air, st.landAt, st.tickAt, st.prev = false, false, nil, false, nil, nil, nil
    return nil
  end
  if st.done then return nil end
  if not st.open then st.open, st.t0, st.air, st.landAt, st.prev = true, clock, false, nil, nil end
  if clock - st.t0 > M.JO_DIAG_MAX or (st.landAt ~= nil and clock - st.landAt > M.JO_DIAG_AFTER_LAND + 1e-6) then
    st.open, st.done = false, true
    return nil
  end
  st.tickAt = clock
  if sample == nil or sample.car == nil or sample.v == nil then st.misses = st.misses + 1; return nil end
  local lx, ly, lz = M.JoDiagCarFrame(sample.car, sample.v)
  local p = st.prev
  local vx, vy, vz, vzw, carV
  if p ~= nil and clock - p.t > 1e-4 and clock - p.t < 0.5 then
    local dt = clock - p.t
    vx, vy, vz = (lx - p.lx) / dt, (ly - p.ly) / dt, (lz - p.lz) / dt
    vzw = (sample.v.z - p.wz) / dt
    carV = math.sqrt((sample.car.x - p.cx) ^ 2 + (sample.car.y - p.cy) ^ 2) / dt
  end
  st.prev = { t = clock, lx = lx, ly = ly, lz = lz, wz = sample.v.z, cx = sample.car.x, cy = sample.car.y }
  if vzw ~= nil then
    if vzw < M.JO_DIAG_DESCENT then st.air, st.landAt = true, nil
    elseif st.air and math.abs(vzw) < M.JO_DIAG_REST then if st.landAt == nil then st.landAt = clock end
    elseif st.landAt ~= nil and vzw < -1 then st.landAt = nil end -- falls again after a false contact
  end
  local function f2(x) return x ~= nil and string.format("%.2f", x) or "-" end
  st.lines = st.lines + 1
  return string.format("[utpp_vehiclecam] jo diag t=%.3f car=(%.2f,%.2f,%.2f) vcar=(%s,%s,%s) onG=%d carV=%s vzw=%s jt=%.2f%s%s",
    clock, lx, ly, lz, f2(vx), f2(vy), f2(vz), st.landAt ~= nil and 1 or 0, f2(carV), f2(vzw), clock - st.t0,
    door and (" door=" .. door) or "", sample.frozen and " frz=1" or "")
end

-- jump-out side push (cfg jumpOutSidePush): decision at the native launch edge. car = { x, y, z, yaw (deg) } (live pose of
-- the car), v = { x, y, z } (V, world), carSpeed = the car's horizontal speed (m/s), isCar = positively identified car.
-- -> { vx, vy, vz = 0, side = -1|1, x, y, speed } (world velocity change in m/s) or nil, reason. V's car-frame x / y come from
-- M.JoDiagCarFrame (the trace's own rotation); the push goes along the same axis, away from the car: world (cos yaw, sin yaw) * side.
function M.JoPushDecide(cfg, car, v, carSpeed, isCar)
  if cfg == nil or cfg.jumpOutSidePush == false then return nil, "setting off" end
  if not isCar then return nil, "not a car" end
  if car == nil or v == nil then return nil, "no vehicle pose" end
  if (tonumber(carSpeed) or 0) > (tonumber(cfg.jumpOutPushMaxCarSpeed) or 3) then return nil, "car moves" end
  local lx, ly = M.JoDiagCarFrame(car, v)
  local ax = math.abs(lx)
  if ax < (tonumber(cfg.jumpOutPushMinX) or 0.5) then return nil, "side unclear", lx, ly end
  if ax > (tonumber(cfg.jumpOutPushMaxX) or 1.8) then return nil, "already clear of the body", lx, ly end
  if math.abs(ly) > (tonumber(cfg.jumpOutPushMaxY) or 2.6) then return nil, "beside no part of the car", lx, ly end
  local speed = math.max(0, math.min(tonumber(cfg.jumpOutPushSpeed) or 0, 1.5)) -- the reds side refuses more than 2 m/s
  if speed <= 0 then return nil, "zero speed", lx, ly end
  local side = lx < 0 and -1 or 1
  local rad = (car.yaw or 0) * math.pi / 180
  return { vx = side * speed * math.cos(rad), vy = side * speed * math.sin(rad), vz = 0, side = side, x = lx, y = ly, speed = speed }
end

-- Runs the pending push of a controller (ctrl.joPush, set by the native launch edge) once: decision, one Game.UTPPQueuePlayerImpulse
-- call, one log line (applied or the gate that refused). sample = { car, v, frozen } as the wiring reads it (nil = unreadable).
-- Returns "applied" | "skipped" | "unavailable" | "failed" | nil (nothing pending). A pending push older than JO_PUSH_STALE s is dropped.
M.JO_PUSH_STALE = 0.5
function M.JoPushRun(ctrl, clock, sample, log)
  local p = ctrl.joPush
  if p == nil then return nil end
  ctrl.joPush = nil
  local function say(msg) if log then log(string.format("[utpp_vehiclecam] t=%.3f %s", clock, msg)) end end
  if clock - (p.at or clock) > M.JO_PUSH_STALE then
    say(string.format("jump-out side push dropped: %.2f s after the launch", clock - p.at))
    return "skipped"
  end
  local d, why, lx, ly = M.JoPushDecide(ctrl.cfg, sample and sample.car or nil, sample and sample.v or nil, p.speed, p.car)
  if d == nil then
    say(string.format("jump-out side push skipped (%s): car-frame x=%s y=%s, car %.1f m/s", why, lx and string.format("%.2f", lx) or "-",
      ly and string.format("%.2f", ly) or "-", tonumber(p.speed) or 0))
    return "skipped"
  end
  local fn = Game and Game.UTPPQueuePlayerImpulse
  if type(fn) ~= "function" then
    say("jump-out side push: Game.UTPPQueuePlayerImpulse unavailable (utpp_vehicle_exit.reds not installed) - no push")
    return "unavailable"
  end
  local ok, r = pcall(fn, d.vx, d.vy, d.vz)
  say(string.format("jump-out side push %s: car-frame x=%.2f y=%.2f, side %s, %.2f m/s -> world (%.2f,%.2f,%.2f), car %.1f m/s%s",
    (ok and r == true) and "applied" or "FAILED", d.x, d.y, d.side < 0 and "left" or "right", d.speed, d.vx, d.vy, d.vz,
    tonumber(p.speed) or 0, (ok and r == true) and "" or (" (" .. (ok and "Game.UTPPQueuePlayerImpulse returned " .. tostring(r) or tostring(r)) .. ")")))
  return (ok and r == true) and "applied" or "failed"
end

-- the peak speed of an engine / eased blend of the given residual over `blend` s (smoothstep: 1.5 x the mean speed), the
-- implied per-frame change at `fps`: used by the offline handover gate
function M.BlendPeak(residualM, residualDeg, blend, fps)
  local b = math.max(0.02, tonumber(blend) or 0.45)
  local f = 1 / math.max(1, tonumber(fps) or 60)
  return 1.5 * (residualM or 0) / b * f, 1.5 * (residualDeg or 0) / b * f
end
-- optional callback field for utpp_body (set at run time by that module): called as
-- pcall(M.onVehiclePerspective, prev, new) with the vehicle camera perspectives 0-6 while the game's vehicle camera has the
-- view (keep "game"). No registration function: the field itself is the interface.
M.onVehiclePerspective = nil
-- the pull-out / vehicle-behind target of motorcycles (bikePullBack / bikePullHeight), other keys from cfg
local function pullCfgFor(cfg, bike)
  if not bike then return cfg end
  return setmetatable({ pullBack = tonumber(cfg.bikePullBack) or cfg.pullBack,
                        pullHeight = tonumber(cfg.bikePullHeight) or cfg.pullHeight,
                        pullClearR = tonumber(cfg.bikePullClearR) or 0 }, { __index = cfg })
end
M.PullCfg = pullCfgFor

-- Pure controller (no game calls): fed with per-frame observations, returns the perspective to request or nil.
function M.NewController(cfg)
  local c = {
    cfg = copy(M.DEFAULTS),
    phase = "idle",       -- idle | armed | done | failed | passive
    reason = "",
    clock = 0,
    rideFromTpp = false,
    armedAt = 0, sawDrive = false, driveAt = 0, requests = 0, lastReqAt = nil,
    stats = { rides = 0, armed = 0, done = 0, failed = 0, requests = 0, lastRequest = -1, lastActive = -1, lastState = 0 },
    trace = nil,
    keep = "off", keepReady = true, keepSince = 0, keepReason = "",
    seatedActive = nil, phaseEndAt = nil, firstReqAt = nil,
    -- exit fix: exitPull nil | "hold" | "release"
    exitPull = nil, exitPullSince = nil, lastActive = nil,
    vehBlend = nil,       -- mount handover fix: vehicle camera blend time read for this ride (s), nil = unread
    vehBike = nil,        -- exit match 2: the ridden vehicle is a motorcycle (read with vehBlend), nil = unknown
    -- mount jump fix: cam.hold nil | "mount" | "exit" (utppCamera at high priority); cam.releaseUntil = end of the
    -- handover release blend (+ margin); cam.cmds = commands for utpp_camera (M.RunOwnCmds); pathStart = pull
    -- start pose in the vehicle frame { a, r, h }; pullFrame = { px, py, pz, pyaw, vx, vy, vz, vyaw } (wiring)
    cam = { hold = nil, releaseUntil = nil, early = false, cmds = {} },
    pathStart = nil, pullFrame = nil, camTraceUntil = nil,
    -- exit match fix: match = { x, y, z (vehicle frame), yaw (deg, vehicle relative), pitch, at, dur, why, src } (game
    -- camera pose utppCamera starts from); release = { at, dur } (exit release to the orbit)
    match = nil, release = nil,
    -- own = vehicle path active (wiring: utpp_camera's vehicle API present), ownHeld = utpp_camera held for this vehicle,
    -- handMatch = pose match before the handover { x, y, z, yaw, pitch (vehicle frame), at, dur, src }, ownLast = last
    -- shown world pose
    own = false, ownHeld = false, handMatch = nil, handMatchTried = false, rideFppSince = nil, ownLast = nil,
    -- rideBase[bike|car] = last TPP perspective UTPP requested (session only); ownApiMotion = utpp_camera
    -- has TakeForVehicleExit (wiring); seatPendingLogged = "seat not final yet" logged in this ride
    rideBase = {}, ownApiMotion = false, seatPendingLogged = false,
    -- vehCar = the ridden vehicle was positively identified as a car (IsA bike read succeeded and was false;
    -- vehBike alone is true or nil, so "not vehBike" also means "unknown"), vehRecKey = vehicle record, vehTppLearned =
    -- session cache of the settled vehicle TPP camera pose per record and perspective
    vehCar = nil, vehRecKey = nil, vehTppLearned = {}, handGlideTried = false, learn = nil,
  }
  for k, v in pairs(cfg or {}) do c.cfg[k] = v end

  local function log(msg)
    if c.trace then c.trace(string.format("[utpp_vehiclecam] t=%.2f %s", c.clock, msg)) end
  end
  c.Log = log

  -- mount jump fix: commands for utpp_camera, executed by M.RunOwnCmds after Step (before the requests)
  local function camCmd(op, blend, why)
    c.cam.cmds[#c.cam.cmds + 1] = { op = op, blend = blend, why = why }
    c.stats.camCmds = (c.stats.camCmds or 0) + 1
  end
  function c.TakeCamCmds()
    local l = c.cam.cmds
    c.cam.cmds = {}
    return l
  end
  function c.Releasing() return c.cam.releaseUntil ~= nil and c.clock <= c.cam.releaseUntil end
  -- exit settle fix: early exit camera kept after the unmount until the deadline
  function c.ExitHoldExtended() return c.cam.hold == "exit" and c.cam.holdUntil ~= nil and c.clock <= c.cam.holdUntil end
  function c.CamTraceWanted()
    if c.keep == "mount" and (c.pulling or c.own) then return true end
    if c.keep == "chase" then return true end
    if c.keep == "exit" or c.cam.hold ~= nil or c.Releasing() or c.exitPull ~= nil then return true end
    return c.camTraceUntil ~= nil and c.clock <= c.camTraceUntil
  end

  -- exit match fix: the game camera pose (vehicle frame) from the wiring's reader c.gameCamQuery(allowActive) ->
  -- { x, y, z, yaw (deg, relative to the vehicle yaw), pitch (deg, + = up), src } or (exit match 2) a list of such
  -- poses in priority order (active camera first, then the vehicleTPPCamera component). allowActive: the camera
  -- system's active camera may be used (only while utppCamera is not the active one). The first candidate outside
  -- the vehicle radius wins; second value = why nothing was usable, third = the rejected candidates (logged).
  -- the vehicle camera manager reports TPP (IsTPPActive) on this or the previous frame (the game switches the vehicle
  -- camera to FPP within a frame of the exit start); unread -> the active perspective of the last mounted frame
  function c.VehTppActive()
    if c.vehTppNow == true or c.vehTppPrev == true then return true end
    if c.vehTppNow == nil and c.vehTppPrev == nil then return c.lastActive ~= nil and c.lastActive ~= 0 end
    return false
  end
  function c.QueryGameCam(allowActive)
    if type(c.gameCamQuery) ~= "function" then return nil, "no game camera reader" end
    local ok, pose, why = pcall(c.gameCamQuery, allowActive)
    if not ok then return nil, "read failed: " .. tostring(pose) end
    if type(pose) ~= "table" then return nil, why or "no game camera pose" end
    local list = pose
    if pose.x ~= nil then list = { pose } end
    local rejected = {}
    -- (1): bikes with a TPP vehicle camera accept a close camera behind the bike (TPPClose ~1.55 m)
    local bikeTpp = c.vehBike and c.VehTppActive()
    for _, p in ipairs(list) do
      if type(p) == "table" and type(p.x) == "number" and type(p.y) == "number" and type(p.z) == "number" then
        local minD = c.cfg.exitMatchMinDist
        if bikeTpp and p.y < (tonumber(c.cfg.bikeExitMatchBehind) or -0.8) then
          minD = math.min(minD, tonumber(c.cfg.bikeExitMatchMinDist) or minD)
        end
        -- an exactly canonical orientation (yaw 0 / pitch 0) while the alternative reading of the same camera
        -- differs by more than 2 deg -> the alternative (logged with the source)
        if p.yaw == 0 and p.pitch == 0 and type(p.altYaw) == "number" and type(p.altPitch) == "number"
           and (math.abs(wrap180(p.altYaw)) > 2 or math.abs(p.altPitch) > 2) then
          local q = {}
          for k, v in pairs(p) do q[k] = v end
          q.yaw, q.pitch = p.altYaw, p.altPitch
          q.src = tostring(p.src) .. " (" .. tostring(p.altSrc or "alternative orientation") .. ")"
          p = q
        elseif math.abs(wrap180(tonumber(p.yaw) or 0)) < 0.5 and math.abs(tonumber(p.pitch) or 0) < 0.5 then
          -- an orientation reading 0 / 0 that no alternative reading contradicts is not proven (the canonical
          -- forward of the camera system); the car exit keeps a short take blend for it
          local q = {}
          for k, v in pairs(p) do q[k] = v end
          q.unsure = true
          p = q
        end
        if math.sqrt(p.x * p.x + p.y * p.y) < minD then
          rejected[#rejected + 1] = string.format("%s at (%.2f,%.2f,%.2f) is inside the vehicle", tostring(p.src), p.x,
            p.y, p.z)
        else
          return p, nil, (#rejected > 0) and table.concat(rejected, "; ") or nil
        end
      end
    end
    if #rejected > 0 then
      local err = table.concat(rejected, "; ")
      if why then err = err .. "; " .. tostring(why) end
      return nil, err
    end
    return nil, why or "no game camera pose"
  end
  -- the car fixes act on positively identified cars only (vehCar); bikes and unknown classes keep
  -- the previous behaviour
  local function isCar() return c.vehCar == true and c.vehBike ~= true end
  c.IsCar = isCar
  -- duration of the exit match (s): exitMatchTime (A/B SetExitMatchTime, bikes, unknown class), for cars scaled with the
  -- distance to the vehicle-behind hold pose and clamped (exitMatchSpeed m/s average)
  function c.MatchDuration(m)
    local base = math.max(0.05, tonumber(c.cfg.exitMatchTime) or 0.7)
    if not (c.cfg.exitMatchScale and isCar()) then return base end
    local dx, dy, dz = m.x, m.y + (tonumber(c.cfg.pullBack) or 5.5), m.z - (tonumber(c.cfg.pullHeight) or 2.4)
    local d = math.sqrt(dx * dx + dy * dy + dz * dz)
    local lo, hi = tonumber(c.cfg.exitMatchTimeMin) or 0.7, tonumber(c.cfg.exitMatchTimeMax) or 1.0
    return math.max(0.05, math.max(lo, math.min(hi, d / math.max(0.1, tonumber(c.cfg.exitMatchSpeed) or 4.0))))
  end
  -- utppCamera starts at the game camera pose and blends to its pose (vehicle-behind hold) over exitMatchTime
  function c.StartMatch(why, allowActive)
    if not (c.cfg.exitMatch and c.cfg.enabled and c.cfg.exitPull and c.exitPull == "hold") then return false end
    local pose, err, rejected = c.QueryGameCam(allowActive)
    if not pose then
      c.stats.exitMatchMisses = (c.stats.exitMatchMisses or 0) + 1
      log(string.format("exit match: no game camera pose at %s (%s) - %s", why, tostring(err),
        c.match and "current blend kept" or "vehicle-behind pose as is"))
      return false
    end
    c.match = { x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pose.pitch or 0, at = c.clock,
                why = why, src = tostring(pose.src), unsure = pose.unsure == true }
    c.match.dur = c.MatchDuration(c.match)
    -- (3): FOV of the matched camera (only with a reader)
    local fovNote = ""
    if c.cfg.exitMatchFov then
      local fv = tonumber(pose.fov)
      if fv and fv >= 10 and fv <= 120 then c.match.fov = fv; fovNote = string.format(" fov=%.1f", fv)
      else fovNote = " (fov unread)" end
    end
    c.stats.exitMatches = (c.stats.exitMatches or 0) + 1
    log(string.format("exit match: cam=(%.2f,%.2f,%.2f) yaw=%.0f pitch=%.0f (%s) -> target in %.2f s (%s)%s%s", pose.x,
      pose.y, pose.z, c.match.yaw, c.match.pitch, c.match.src, c.match.dur, why,
      rejected and (" [skipped: " .. rejected .. "]") or "", fovNote))
    return true
  end
  -- weight of the matched pose (1 at the match, 0 when the blend to the hold pose is over)
  function c.MatchWeight()
    if c.match == nil then return 0 end
    return 1 - smooth01((c.clock - c.match.at) / c.match.dur)
  end
  local function endMatch(why)
    if c.match == nil then return end
    log(string.format("exit match: %s (%.2f s after the match at %s)", why, c.clock - c.match.at, c.match.why))
    c.match = nil
  end
  c.EndMatch = endMatch

  local function finish(phase, reason)
    c.phase, c.reason, c.phaseEndAt = phase, reason, c.clock
    if phase == "done" then c.stats.done = c.stats.done + 1 else c.stats.failed = c.stats.failed + 1 end
    -- one summary line instead of one line per request (ride-FPP fix)
    local req = ""
    if c.requests > 0 then
      req = string.format(", %d request(s) over %.2f s", c.requests, c.clock - (c.firstReqAt or c.clock))
    end
    -- time from the arming (mount start) to the end of the request loop: the game's vehicle TPP pose swap follows the perspective
    local since = ""
    if c.armedAt ~= nil and c.armedAt > 0 then since = string.format(", %.2f s after the mount start", c.clock - c.armedAt) end
    log(phase .. " (" .. reason .. ")" .. req .. since)
  end


  -- fixed perspective > rideBase of this vehicle class (the last UTPP request in this session) >
  -- persistent TPP 1-3 > persistent DriverCombat 4-6 mapped to 1-3 > TPPClose. A persistent FPP is never chosen.
  local function rideClass() return c.vehBike and "bike" or "car" end
  c.RideClass = rideClass
  local function choose(persistent)
    local p = c.cfg.perspective
    if p == 1 or p == 2 or p == 3 then return p end
    local b = c.rideBase[rideClass()]
    if b == 1 or b == 2 or b == 3 then return b end
    if persistent == 1 or persistent == 2 or persistent == 3 then return persistent end
    if persistent == 4 or persistent == 5 or persistent == 6 then return persistent - 3 end
    return M.PERSP.TPPClose
  end
  c.Choose = choose

  -- mount handover fix: s after the vehicle TPP perspective became active before the view is handed over
  function c.HandoverDelay()
    local h = c.cfg.keepHandover
    if c.cfg.keepHandoverBlend and c.phase == "done" then
      local blend = (type(c.vehBlend) == "number" and c.vehBlend > 0) and c.vehBlend or c.cfg.vehBlendDefault
      h = math.min(math.max(h, blend + c.cfg.keepHandoverMargin), c.cfg.keepHandoverMax)
    end
    return h
  end

  -- pull-out weight (0..1, smoothstep applied by M.OwnPose)
  c.pull = 0
  function c.UpdatePull(clock, obs, dt)
    dt = M.BlendDt(c, dt, tonumber(c.cfg.blendDtCap) or 0) -- a frame stall does not jump the pull weight
    -- exit fix: hold (weight 1) during the exit hold, then release (1 -> 0 over exitPullTime)
    if c.exitPull ~= nil then
      if c.exitPull == "hold" then
        -- exit settle fix: also while the early camera is kept after the unmount (the release starts when the hold ends)
        if (c.keep == "exit" or c.ExitHoldExtended()) and c.cfg.exitPull then
          c.pull = 1
          return
        elseif c.keep == "ride" or not c.cfg.exitPull then
          log("exit pull: end (" .. (c.keep == "ride" and "exit aborted" or "switched off") .. ")")
          c.exitPull, c.pull = nil, 0
          endMatch("dropped (" .. (c.keep == "ride" and "exit aborted" or "switched off") .. ")")
          return
        end
        c.exitPull = "release"
        log(string.format("exit pull: release over %.2f s (%.2f s after the exit start%s)", c.cfg.exitPullTime,
          clock - (c.exitPullSince or clock), c.keepReason ~= "" and (", " .. c.keepReason) or ", unmounted"))
        c.release = { at = clock, dur = c.cfg.exitPullTime }
        log(string.format("exit release: blend %.2f s start (from the last shown pose)", c.cfg.exitPullTime))
        -- utpp_camera blends from its last shown pose to its orbit and ends the hold itself
        camCmd("orbit", c.cfg.exitPullTime, c.exitFollow and "exit release after the jump-out follow" or "exit release")
        -- jump-out follow: the release carries its offset with the orbit (V keeps moving)
        if c.exitFollow then c.cam.cmds[#c.cam.cmds].carry = true end
        c.ownHeld = false
        return
      end
      c.pull = math.max(0, c.pull - (dt or 0) / math.max(0.05, c.cfg.exitPullTime))
      if c.pull <= 0 or c.keep == "mount" then
        log("exit pull: end (utpp_camera's orbit)")
        if c.release then
          log(string.format("exit release: blend %.2f s end (%.2f s)", c.release.dur, clock - c.release.at))
        end
        c.exitPull, c.pull, c.release = nil, 0, nil
        endMatch("dropped with the release")
      end
      return
    end
    local want, escape = false, false
    -- held until the hold ends (mount -> ride), also after the vehicle camera became active (no snap back inside the car)
    -- also the chase pose after a ride watchdog retake (keep mount again / chase)
    if (c.cfg.pullOut or (c.rideRetakes or 0) > 0) and (c.keep == "mount" or c.keep == "chase") then
      local since = clock - (c.keepSince or clock)
      local near = obs.vehDist ~= nil and obs.vehDist < c.cfg.pullDist and since > 0.2
      -- mount pivot continuity: while utpp_camera blends its pivot from V's position before the seat-slot snap
      -- (obs.camMountBlend, utpp_camera.MountBlendActive) the near trigger waits for the blend end (at most mountBlendTime
      -- 0.38 s; pullStart still applies): the path is frozen from a camera at rest, not mid-way through the blend (the
      -- shown camera stopped from about 4 m/s in one frame); one motion after the other
      if near and not c.pulling and obs.camMountBlend == true and since < c.cfg.pullStart then
        near = false
        c.pullWaited = true
      end
      -- (pullEscape): the orbit camera is inside the keep-out box of the car (OwnPose sets camInBox) - the pull-out starts now (after the
      -- pivot blend, pullEscapeMin s after the mount) instead of letting the camera sit in the cabin until pullStart
      escape = c.camInBox ~= nil and c.cfg.pullEscape ~= false and since >= (tonumber(c.cfg.pullEscapeMin) or 0.2) and obs.camMountBlend ~= true
      -- the orbit camera about to enter the box (OwnPose sets camBoxNear) starts it without the pullEscapeMin wait
      local ahead = c.camBoxNear ~= nil and c.cfg.pullEscape ~= false and obs.camMountBlend ~= true
      escape = escape or ahead
      want = c.pulling or since >= c.cfg.pullStart or near or escape
    end
    if want then
      if not c.pulling then
        c.pulling = true
        c.pathStart = nil -- frozen by the first pose written by M.OwnPose
        c.stats.pulls = (c.stats.pulls or 0) + 1
        log(string.format("pull-out start (%.2f s after the mount start, dist %s, path %s, %.2f s%s%s%s)",
          clock - (c.keepSince or clock), tostring(obs.vehDist), c.cfg.pullPath and "around the vehicle" or "linear",
          (c.vehBike and tonumber(c.cfg.bikePullTime)) or c.cfg.pullTime, c.vehBike and ", motorcycle" or "",
          c.pullWaited and ", after the camera's mount pivot blend" or "",
          (escape and clock - (c.keepSince or clock) < c.cfg.pullStart and not near) and (c.camInBox and string.format(", orbit camera %.2f m inside the vehicle box", c.camInBox)
            or string.format(", orbit camera approaching the vehicle box (depth %.2f m, %.2f m in %d frames; < 0 = outside)", c.camBoxNear.d or 0,
              c.camBoxNear.pred or 0, tonumber(c.cfg.pullEscapeLead) or 0)) or ""))
        c.pullWaited = nil
      end
      local pt = (c.vehBike and tonumber(c.cfg.bikePullTime)) or c.cfg.pullTime
      if isCar() and c.pathStart and tonumber(c.pathStart.dur) then pt = c.pathStart.dur end -- (cars only)
      c.pull = math.min(1, c.pull + (dt or 0) / math.max(0.05, pt))
    elseif c.pulling and c.Releasing() then
      c.pull = 1 -- mount jump fix: utppCamera stays at the pull-out pose while the engine blends it out (handover)
    else
      if c.pulling then log("pull-out end (" .. tostring(c.keep) .. ")") end
      c.pulling, c.pull, c.pathStart = false, 0, nil
    end
  end

  function c.StepRide(clock, obs, vs, inVehicle)

    if not inVehicle then
      if c.phase ~= "idle" then
        c.phase, c.reason, c.rideFromTpp = "idle", "", false
      end
      return nil
    end

    if c.phase == "idle" then
      c.stats.rides = c.stats.rides + 1
      c.rideFromTpp = (c.keep == "mount") -- the utppCamera mount hold (UpdateOwn runs first)
      c.armedAt, c.sawDrive, c.driveAt, c.requests, c.lastReqAt = clock, false, 0, 0, nil
      c.firstReqAt, c.phaseEndAt, c.seatedActive = nil, nil, nil
      c.forced, c.forceDone, c.forceDriveAt, c.lateForced = false, false, nil, false
      if c.cfg.enabled and c.rideFromTpp then
        c.phase, c.reason = "armed", ""
        c.stats.armed = c.stats.armed + 1
        log("ride from UTPP third person (utppCamera held) - armed")
      else
        c.phase, c.reason = "passive", c.rideFromTpp and "disabled" or "ride from FPP"
      end
    end

    -- forceTppOnEnter: a driver ride without the UTPP hold runs the same request loop from the first
    -- Driving frame; never in seat states 2/3/5/7 (passengers: c.StepPassenger), in scenes, or once the ride is past its
    -- entry. Also a ride whose mount hold ran into armWindow ("failed (timeout)") - the
    -- request is armed again at its first Driving frame (once per ride, inside the entry window, so it never overrides a
    -- key switch made later in the ride)
    local lateEntry = c.phase == "failed" and c.reason == "timeout" and not c.sawDrive and not c.lateForced and isCar()
    if (lateEntry or (c.phase == "passive" and not c.forceDone)) and c.cfg.enabled and c.cfg.forceTppOnEnter then
      local high = obs.highLevel
      local scene = type(high) == "number" and high >= (c.cfg.forceSceneMin or 3) and high <= (c.cfg.forceSceneMax or 6)
      if vs == 1 and c.forceDriveAt == nil then c.forceDriveAt = clock end
      local why = nil
      if vs == 4 and c.forceDriveAt ~= nil then why = "exit or seat switch"
      elseif vs == 3 and c.cfg.forceTppPassenger then
        why = c.passChase and "vehicle state 3, passenger ride on the own camera" or "vehicle state 3, the passenger request follows"
      elseif vs == 2 or vs == 3 or vs == 5 or vs == 7 then why = "vehicle state " .. tostring(vs)
      elseif c.forceDriveAt ~= nil and (clock - c.forceDriveAt) > c.cfg.settleAfterDrive then why = "driver seat not confirmed"
      end
      -- passenger-side entry: state 1 is the driver state (DriveEvents); its first frame can still read the old
      -- seat_front_right slot after the vanilla switch_seat, so the slot does not veto the request there
      if why then
        c.forceDone = true
        if lateEntry then c.lateForced = true end
        log("always third person on entry: no request for this ride (" .. why .. ")")
      elseif vs == 1 and obs.mounted and not scene then
        local was = c.reason
        c.forceDone, c.forced = true, true
        c.phase, c.reason, c.armedAt = "armed", "", clock
        if lateEntry then
          c.lateForced = true
          c.requests, c.lastReqAt, c.firstReqAt = 0, nil, nil
          c.stats.lateEntryRides = (c.stats.lateEntryRides or 0) + 1
        end
        c.stats.armed = c.stats.armed + 1
        c.stats.forcedRides = (c.stats.forcedRides or 0) + 1
        log(string.format("always third person on entry: driver ride without the UTPP hold (%s) - armed",
          lateEntry and "mount hold timed out" or (was ~= "" and was or "passive")))
      end
    end

    if c.phase ~= "armed" then return nil end
    if not c.cfg.enabled then
      c.phase, c.reason = "passive", "disabled"
      return nil
    end
    if (clock - c.armedAt) > c.cfg.armWindow then finish("failed", "timeout"); return nil end
    if vs == 2 or vs == 3 or vs == 5 or vs == 7 then
      finish("failed", "vehicle state " .. tostring(vs)); return nil
    end
    -- DriverCombat (6) maps the vehicle camera to DriverCombatClose/Medium/Far (third person): done when the
    -- vehicle camera is TPP, never a request in state 6
    if vs == 6 then
      if obs.activePersp ~= nil and obs.activePersp ~= M.PERSP.FPP then
        finish("done", "vehicle " .. tostring(M.PERSP_NAME[obs.activePersp] or obs.activePersp) .. " in driver combat")
      else
        finish("failed", "vehicle state 6 (first person)")
      end
      return nil
    end
    if vs == 1 and not c.sawDrive then c.sawDrive, c.driveAt = true, clock end
    if vs == 4 and c.sawDrive then finish("failed", "exit or seat switch before TPP"); return nil end
    -- passenger-side entry: a non-driver slot before the Drive state is the seat before the vanilla switch_seat (the
    -- passenger states 2/3/5/7 ended the ride above); state 1 is the driver state whatever the slot reads in its first frame
    if obs.driver == false and vs ~= 1 then return nil end
    if obs.activePersp ~= nil and obs.activePersp ~= M.PERSP.FPP then
      finish("done", "vehicle " .. tostring(M.PERSP_NAME[obs.activePersp] or obs.activePersp)); return nil
    end
    if c.sawDrive and (clock - c.driveAt) > c.cfg.settleAfterDrive then finish("failed", "no TPP after driving"); return nil end
    if not obs.mounted then return nil end
    if c.cfg.waitDrive and vs ~= 1 then return nil end
    local maxReq = (c.keep ~= "off") and c.cfg.keepMaxRequests or c.cfg.maxRequests
    local interval = (c.keep ~= "off") and c.cfg.keepRequestInterval or c.cfg.requestInterval
    if c.requests >= maxReq then finish("failed", "max requests"); return nil end
    if c.lastReqAt ~= nil and (clock - c.lastReqAt) < interval then return nil end
    local p = choose(obs.persistentPersp)
    c.requests, c.lastReqAt = c.requests + 1, clock
    if c.requests == 1 then c.firstReqAt = clock end
    c.stats.requests = c.stats.requests + 1
    c.stats.lastRequest = p
    c.rideBase[rideClass()] = p -- the value the next entry of this class requests (never an in-ride choice)
    if c.requests == 1 then -- retries are counted and summarised by finish()
      log(string.format("request %s (state=%d active=%s persistent=%s; retry every %.2f s, max %d)", M.PERSP_NAME[p], vs,
                        tostring(obs.activePersp), tostring(obs.persistentPersp), interval, maxReq))
    end
    return p
  end

  -- ================================================================ vehicle path on utppCamera (see the header)
  -- Phases c.keep "mount" / "ride" / "exit" on utppCamera through utpp_camera's vehicle API.
  -- Commands (c.cam.cmds, executed by M.RunOwnCmds): vhold (HoldForVehicle true), hold / unhold (priority), release /
  -- drop (HandToVehicle), take (SetPoseOverride + TakeFromVehicle), orbit (ReleaseToOrbit), free (HoldForVehicle false),
  -- togame (HoldForVehicle false, "game", blend) and gametake (TakeForVehicleExit, vehicle API >= 2: exit from keep
  -- "game"). c.ownCamReady() / c.ownExitQuery() / c.ownApiMotion are set by the wiring.
  -- (2): passenger combat 2, passenger 3, turret 5 and scene 7 leave the view to the game; driver combat
  -- 6 is a driver state (the game maps the vehicle camera to DriverCombat TPP) and keeps mount / ride / chase
  local function passengerState(vs) return vs == 2 or vs == 3 or vs == 5 or vs == 7 end
  local function driverCombat(vs) return vs == 6 end
  c.PassengerState, c.DriverCombat = passengerState, driverCombat
  local function vehTpp(a) return a ~= nil and a ~= M.PERSP.FPP end
  local function ownCmd(op, blend, why)
    camCmd(op, blend, why)
    if op == "free" or op == "togame" or op == "orbit" then c.ownHeld = false end
  end
  function c.OwnEnabled() return c.own == true and c.cfg.enabled and c.cfg.keepTpp end
  function c.OwnCamReady()
    if type(c.ownCamReady) ~= "function" then return false, "utpp_camera vehicle API missing" end
    local ok, r, why = pcall(c.ownCamReady)
    if not ok then return false, "camera query failed: " .. tostring(r) end
    return r == true, why
  end
  -- start of the mount hold (from Step or the camera's delegate on its "vehicle" decision). True = held / to be held.
  -- the seat is decided once it is known. A non-driver slot during the mount transition (the
  -- mounting slot can read as another seat for seconds) keeps the decision open; only a confirmed passenger (state 3,
  -- or a non-driver slot once seated) gets the passenger rule. A hold taken late (seated) starts the pull-out at once.
  -- passenger-side entry: vanilla PassangerSwitchSeatsCondition switches V from seat_front_right to seat_front_left when
  -- that slot is free and the vehicle allows seat switches. obs.seat = mounting slot name, obs.driverSeatFree =
  -- VehicleComponent.IsSlotAvailable(seat_front_left), obs.switchSeatsDisabled = VehDataPackage DisableSwitchSeats (nil =
  -- unread). Unread values do not block (every entry starts in third person; a confirmed passenger still ends the hold
  -- in state 2/3/5/7). Back seats never switch to the driver seat.
  function c.SeatSwitchExpected(obs)
    if not c.cfg.seatSwitchHold then return false end
    local seat = obs.seat
    if type(seat) == "string" and seat ~= "" and not string.find(seat, "seat_front_right", 1, true) then return false end
    if obs.driverSeatFree == false then return false end
    if obs.switchSeatsDisabled == true then return false end
    return true
  end
  function c.TryStartOwn(clock, obs, why)
    if c.keep == "mount" then return true end
    if c.keep ~= "off" or not c.keepReady then return false end
    local vs = obs.vehicleState or 0
    if not ((vs ~= 0) or obs.mounted) then return false end
    -- a confirmed car passenger with the camera still in third person takes the hold now (late, like a seated driver)
    -- and keeps utppCamera on the chase pose (keep "chase", see seatLeave)
    local passChaseNow = vs == 3 and c.cfg.passengerChase and c.cfg.pullOut and c.cfg.forceTppOnEnter and isCar() and c.OwnEnabled()
    if vs == 3 and not passChaseNow then
      c.keepReady = false
      if c.OwnEnabled() then log("passenger seat - the camera keeps its own rules (first person)") end
      return false
    end
    if passengerState(vs) and not passChaseNow then return false end
    if not c.OwnEnabled() then c.keepReady = false; return false end
    -- passenger-side entry: PSM Vehicle 1 / 6 are the driver states (DriveEvents /
    -- DriverCombat), so they decide the seat; the mounting slot is read one frame late after the vanilla switch_seat
    -- (state 1 with the old seat_front_right slot for one frame) and is not a passenger signal there.
    if obs.driver == false and (vs == 1 or vs == 6) and not c.slotLagLogged then
      c.slotLagLogged = true
      log(string.format("seat slot %s in driver state %d - slot not updated yet, driver seat (PSM)", tostring(obs.seat), vs))
    end
    local switchNote = nil
    if obs.driver == false and vs ~= 1 and vs ~= 6 and not passChaseNow then
      local switchExpected = c.SeatSwitchExpected(obs)
      if not switchExpected then
        if not c.seatPendingLogged then
          c.seatPendingLogged = true
          log(string.format("seat not final yet (state %d, driver=%s, slot %s, driver seat free=%s, switch disabled=%s) - "
            .. "decision kept", vs, tostring(obs.driver), tostring(obs.seat), tostring(obs.driverSeatFree),
            tostring(obs.switchSeatsDisabled)))
        end
        return false
      end
      -- the vanilla SwitchSeats state (PassangerSwitchSeatsCondition) moves V from seat_front_right to the free driver
      -- seat after the enter animation: the hold covers the whole entry, the seat is decided by the PSM state at the end
      switchNote = string.format("passenger-side entry (slot %s, driver seat free=%s) - seat switch to the driver seat "
        .. "expected, hold kept until the seat is final", tostring(obs.seat), tostring(obs.driverSeatFree))
    elseif obs.driver == nil and vs ~= 1 and vs ~= 6 and not passChaseNow then
      if not c.seatPendingLogged then
        c.seatPendingLogged = true
        log(string.format("seat not final yet (state %d, driver=%s) - decision kept", vs, tostring(obs.driver)))
      end
      return false
    end
    local ok, reason = c.OwnCamReady()
    if not ok then
      c.keepReady = false -- one decision per ride
      log("mount while the camera is not held-ready (" .. tostring(reason) .. ") - camera keeps its own rules")
      return false
    end
    c.keepReady = false -- one decision per ride
    local late = (vs == 1 or vs == 6 or passChaseNow)
    c.keep, c.keepSince, c.keepReason = "mount", clock, ""
    c.handMatch, c.handMatchTried, c.rideFppSince, c.ownLast, c.unmountAt = nil, false, nil, nil, nil
    c.exitFollow, c.exitJump, c.vehSample, c.vehSpeed, c.exitJumpSpeed = nil, false, nil, nil, nil
    c.joEpoch = nil
    c.handGate, c.wd, c.wdTrips, c.rideRetakes, c.wdFinal, c.forceRequest = nil, nil, 0, 0, false, nil
    c.handAt, c.handOwnPose, c.exitKeep = nil, nil, nil -- bike exit state
    c.pullWaited = nil -- mount pivot continuity
    c.handGlideTried, c.glideTarget, c.learn, c.abortAsk, c.passChase = false, nil, nil, nil, false -- car entry glide
    c.stats.keepMounts = (c.stats.keepMounts or 0) + 1
    c.stats.ownMounts = (c.stats.ownMounts or 0) + 1
    ownCmd("vhold", nil, why or "mount start")
    c.ownHeld = true
    -- a ride already marked "from FPP" (seen before the hold) is armed again
    if c.phase == "passive" and c.reason == "ride from FPP" then
      c.phase, c.reason, c.rideFromTpp, c.armedAt = "armed", "", true, clock
      c.stats.armed = c.stats.armed + 1
      c.forceDone = true
    end
    log(string.format("keepTpp off -> mount (utppCamera kept active, %s)", tostring(why or "mount start")))
    if switchNote then
      c.seatSwitch = { since = clock }
      c.stats.seatSwitchHolds = (c.stats.seatSwitchHolds or 0) + 1
      log(switchNote)
    end
    if late and c.cfg.pullOut then
      -- seated already: no pullStart wait, the path is frozen from the current orbit pose (M.OwnPose)
      c.pulling, c.pull, c.pathStart = true, 0, nil
      c.stats.pulls = (c.stats.pulls or 0) + 1
      c.stats.lateHolds = (c.stats.lateHolds or 0) + 1
      log(string.format("pull-out start at once (hold taken while seated, state %d)", vs))
    end
    if passChaseNow then
      c.keep, c.keepReason, c.passChase = "chase", "passenger ride on the own camera", true
      c.cam.hold = "mount"
      ownCmd("hold", nil, "passenger chase")
      c.stats.passengerChases = (c.stats.passengerChases or 0) + 1
      log("passenger seat (state 3): hold taken while seated - utppCamera takes the vehicle-behind chase pose for the ride")
    end
    return true
  end
  -- outcome of the "vhold" command (the camera may refuse, e.g. it left third person in the same frame)
  function c.OnOwnHold(ok, err)
    if ok then return end
    c.ownHeld = false
    c.stats.ownRefused = (c.stats.ownRefused or 0) + 1
    if c.keep == "mount" then
      c.keep, c.keepReason = "off", "camera refused the hold"
      c.cam.hold, c.pulling, c.pull, c.pathStart = nil, false, 0, nil
      if c.phase == "armed" then c.phase, c.reason = "passive", "camera refused the hold" end
      log("keepTpp mount -> off (camera refused the hold: " .. tostring(err) .. ")")
    end
  end
  -- outcome of the "gametake" command: refused -> no exit camera, the camera keeps its own rules (as before)
  function c.OnGameTake(ok, err)
    if ok then return end
    c.ownHeld = false
    c.stats.gameTakeRefused = (c.stats.gameTakeRefused or 0) + 1
    if c.keep == "exit" and c.exitFromGame then
      c.keep, c.keepReason = "off", "camera refused the exit take-over"
      c.cam.hold, c.cam.early, c.cam.holdUntil = nil, false, nil
      c.exitPull, c.pull, c.release, c.match, c.exitFromGame = nil, 0, nil, nil, false
      log("keepTpp exit -> off (camera refused the exit take-over: " .. tostring(err) .. ")")
    end
  end
  -- lesson of the exit settle fix: nothing queued / pending from the mount may deactivate the exit camera
  local function flushMountRelease(clock)
    local kept, dropped = {}, 0
    for _, cmd in ipairs(c.cam.cmds) do
      if cmd.op == "settle" or cmd.op == "release" or cmd.op == "drop" then dropped = dropped + 1
      else kept[#kept + 1] = cmd end
    end
    c.cam.cmds = kept
    if c.cam.releaseUntil ~= nil or dropped > 0 then
      c.stats.releaseFlushes = (c.stats.releaseFlushes or 0) + 1
      log(string.format("exit start: pending mount release flushed (%s, %d queued command(s) dropped) - no settle / "
        .. "Deactivate on the exit camera", c.cam.releaseUntil and string.format("%.2f s left", c.cam.releaseUntil - clock)
        or "window over", dropped))
    end
    c.cam.releaseUntil, c.handMatch = nil, nil
  end
  -- the last shown utppCamera pose (c.ownLast, world) in the vehicle frame of c.pullFrame, or nil
  function c.ShownVehPose()
    local f, s = c.pullFrame, c.ownLast
    if not (f and s) then return nil end
    local r = math.rad(-f.vyaw)
    local dx, dy = s.x - f.vx, s.y - f.vy
    return { x = math.cos(r) * dx - math.sin(r) * dy, y = math.sin(r) * dx + math.cos(r) * dy, z = s.z - f.vz,
             yaw = wrap180(s.yaw - f.vyaw), pitch = s.pitch * (c.cfg.pullPitchSign or 1), fov = s.fov }
  end
  -- weight of the vehicle camera in the running hand-over engine blend (0 at the hand-over, 1 at its end), nil
  -- when no blend runs or the shown pose at the hand-over is unknown
  function c.HandBlend(clock)
    if c.handAt == nil or c.handOwnPose == nil then return nil end
    local dur = tonumber(c.handBlendDur) or tonumber(c.cfg.holdRelease) or 0.45
    local dt = clock - c.handAt
    if dur <= 0 or dt < 0 or dt >= dur then return nil end
    return smooth01(dt / dur)
  end
  -- car exit: the take blend for a match read from the rendered camera in this frame. nil = not applicable
  -- (not a car, no match, a match from another source): the caller keeps its blend. A pose that reads 0 / 0 keeps a short
  -- blend (its pitch is not proven: a cut would pop by the unknown difference).
  local function carTakeBlend()
    local m = c.match
    if not (isCar() and m and string.sub(tostring(m.src), 1, 13) == "active camera") then return nil end
    if m.unsure then return math.max(0, tonumber(c.cfg.exitTakeBlendUnsure) or 0.12), "orientation unproven (0 / 0)" end
    return math.max(0, tonumber(c.cfg.exitTakeBlendMatched) or 0), "matched in the same frame"
  end
  -- exit start. take = utppCamera was handed over (take the view back at the matched vehicle camera pose); false =
  -- utppCamera is still the shown camera (exit before the handover): the match is the shown pose, no activation.
  local function startOwnExit(clock, why, take)
    c.keep, c.keepSince, c.keepReason = "exit", clock, ""
    -- review: unmountAt of the previous ride is cleared here too (TryStartOwn, which also clears it, is not reached
    -- by a ride kept as 'game'); a stale value started the far-seat exit follow on the first exit frame
    c.exitFollow, c.exitJump, c.exitStartDist, c.unmountAt, c.exitJumpSpeed = nil, false, nil, nil, nil
    c.joEpoch = c.NativeJumpOutEpoch() -- baseline of the native launch counter at the exit start (nil = native unavailable)
    c.stats.keepExits = (c.stats.keepExits or 0) + 1
    c.stats.ownExits = (c.stats.ownExits or 0) + 1
    flushMountRelease(clock)
    c.exitPull, c.exitPullSince, c.pull, c.release = "hold", clock, 1, nil
    c.stats.exitPulls = (c.stats.exitPulls or 0) + 1
    c.cam.hold, c.cam.early, c.cam.holdUntil = "exit", true, nil
    c.pulling, c.pathStart = false, nil
    c.exitFromGame = (take == "game")
    c.exitKeep = nil -- A: set below for a take from the handed-over vehicle camera only
    if take == "game" then
      -- the game's vehicle camera has the view (utpp_camera in its first-person state, no hold). The match pose
      -- is the rendered vehicle TPP camera (active camera, rejected inside the vehicle radius), else the vehicle-behind
      -- pose; never a pose at the head. TakeForVehicleExit renders it at once in third person (blend only from TPP).
      local tpp = c.exitVehTpp == true
      if tpp then c.StartMatch(why, true) end
      local b = (tpp and c.match) and (tonumber(c.cfg.gameTakeBlend) or 0.25) or 0
      local cb = tpp and carTakeBlend() or nil
      if cb ~= nil then b = cb end
      ownCmd("gametake", b, why)
      c.ownHeld = true
      c.stats.ownTakes = (c.stats.ownTakes or 0) + 1
      c.stats.gameTakes = (c.stats.gameTakes or 0) + 1
      log(string.format("exit (%s): utppCamera takes the view from the game camera at the %s pose (vehicle camera %s, "
        .. "blend %.2f s), vehicle-behind pose until the unmount", why, c.match and "matched game camera" or "vehicle-behind",
        tpp and "TPP" or "FPP", b))
      return
    end
    if take then
      c.StartMatch(why, true) -- the active camera is the vehicle camera (utppCamera is handed over)
      local blend = c.cfg.earlyExitBlend
      -- an exit from a moving vehicle (jump-out) keeps the shorter take-over blend: V leaves the vehicle camera's view
      if (c.vehSpeed or 0) > (tonumber(c.cfg.exitFollowSpeed) or 4) then
        blend = math.min(blend, tonumber(c.cfg.jumpExitBlend) or 0.20)
      end
      local cb = carTakeBlend()
      if cb ~= nil then blend = cb end
      -- exit during the hand-over engine blend: the rendered picture is a mix of the pull-out pose and the vehicle
      -- camera; the match is that mix (weight of the blend), taken with blend 0 (the matched pose IS the rendered pose)
      local hb = c.HandBlend(clock)
      if hb ~= nil and c.cfg.exitMatch then
        local gp = c.match or c.handOwnPose
        local m = vposeLerp(c.handOwnPose, gp, hb)
        -- FOV of the mix: the shown (glide) FOV and the matched camera's FOV by the blend weight
        local fa, fb = c.handOwnPose.fov, c.match and c.match.fov
        m.at, m.why = clock, why
        m.fov = (fa and fb) and (fa + (fb - fa) * hb) or fb or fa
        m.dur = c.MatchDuration(m)
        m.src = string.format("hand-over blend w=%.2f (%s)", hb, c.match and c.match.src or "pull-out pose only")
        if c.match == nil then c.stats.exitMatches = (c.stats.exitMatches or 0) + 1 end
        c.match = m
        blend = 0
        c.stats.handBlendExits = (c.stats.handBlendExits or 0) + 1
        log(string.format("exit during the hand-over blend (w=%.2f, %.2f s after the hand-over): match = pull-out pose -> "
          .. "%s, take-over blend 0", hb, clock - c.handAt, gp == c.handOwnPose and "pull-out pose (no game camera pose)"
          or "game camera pose"))
      end
      c.handAt, c.handOwnPose = nil, nil
      ownCmd("take", blend, why)
      c.stats.ownTakes = (c.stats.ownTakes or 0) + 1
      log(string.format("exit (%s): utppCamera takes the view at the %s pose (blend %.2f s, high priority), "
        .. "vehicle-behind pose until the unmount", why, c.match and "matched game camera" or "vehicle-behind", blend))
      -- A: the seated TPP perspective is requested again during the exit
      -- cars only count the exit frames in vehicle FPP (passive diagnostic); the requests are no-ops there
      local sa = c.seatedActive
      if type(sa) == "number" and sa >= 1 and sa <= 6 then
        local active = c.cfg.exitKeepVehicleTpp
        if isCar() then active = c.cfg.carExitKeepVehicleTpp end
        c.exitKeep = { p = (sa > 3) and (sa - 3) or sa, sends = 0, nextAt = clock, since = clock, fppFrames = 0,
                       frames = 0, passive = not active }
      else
        c.exitKeep = nil
      end
      return
    end
    local s = c.ShownVehPose()
    if s and c.cfg.exitMatch then
      c.match = { x = s.x, y = s.y, z = s.z, yaw = s.yaw, pitch = s.pitch, at = clock,
                  why = why, src = "shown utppCamera pose", fov = c.cfg.exitMatchFov and s.fov or nil }
      c.match.dur = c.MatchDuration(c.match)
      c.stats.exitMatches = (c.stats.exitMatches or 0) + 1
    end
    log(string.format("exit (%s): utppCamera still shown - no activation, %s", why, c.match
      and string.format("blend from the shown pose (%.2f,%.2f,%.2f) to the vehicle-behind pose in %.2f s", c.match.x,
        c.match.y, c.match.z, c.match.dur) or "vehicle-behind pose"))
  end
  -- blend = the Deactivate blend (s) of a glide / match; nil = holdRelease (bikes: bikeHoldRelease), for a car without a
  -- target pose at least handFallbackBlend (a slow blend keeps the engine blend's 3.5 m zoom gentle)
  local function ownHandover(clock, why, blend)
    c.keep, c.keepReason = "ride", ""
    c.camTraceUntil = clock + c.cfg.camTraceAfter
    -- bikes hand over with the longer bikeHoldRelease blend
    local b = math.max(0, tonumber(c.vehBike and c.cfg.bikeHoldRelease or c.cfg.holdRelease) or 0)
    if blend ~= nil then
      b = math.max(0, tonumber(blend) or b)
    elseif isCar() and c.cfg.handGlide and (c.rideRetakes or 0) == 0 then
      b = math.max(b, tonumber(c.cfg.handFallbackBlend) or b)
    end
    -- the shown pull-out pose at the hand-over (vehicle frame) and its clock, for an exit during the blend
    c.handAt, c.handOwnPose, c.handBlendDur = clock, c.ShownVehPose(), b
    c.cam.hold = nil
    c.cam.releaseUntil = clock + b + c.cfg.holdReleaseMargin
    ownCmd("release", b, "handover")
    c.stats.camReleases = (c.stats.camReleases or 0) + 1
    c.stats.ownHandovers = (c.stats.ownHandovers or 0) + 1
    log(string.format("handover %.2f s after the vehicle TPP perspective (vehicle camera blend %s, %s): utppCamera "
      .. "Deactivate(%.2f) -> vehicle %s, high priority off, TPP signal stays on", clock - (c.phaseEndAt or clock),
      type(c.vehBlend) == "number" and string.format("%.3f s read", c.vehBlend) or "unread", why, b,
      tostring(M.PERSP_NAME[c.lastActive] or c.lastActive)))
  end
  -- ------------------------------------------------------------ car entry glide (see M.DEFAULTS handGlide)
  local function poseStr(p)
    return string.format("(%.2f,%.2f,%.2f) yaw=%.0f pitch=%.0f%s", p.x, p.y, p.z, p.yaw or 0, p.pitch or 0,
      p.fov and string.format(" fov=%.0f", p.fov) or "")
  end
  -- target sanity: behind the vehicle, outside it, at a plausible height and distance, not too far from the shown pose.
  -- Returns nil or the reason it was rejected.
  local function poseRejected(p, shown)
    for _, k in ipairs({ "x", "y", "z" }) do
      if type(p[k]) ~= "number" or p[k] ~= p[k] then return "not a number" end
    end
    local h = math.sqrt(p.x * p.x + p.y * p.y)
    if h < (tonumber(c.cfg.exitMatchMinDist) or 1.8) then return string.format("inside the vehicle (%.2f m)", h) end
    if h > (tonumber(c.cfg.handGlideMaxRadius) or 15) then return string.format("%.1f m from the vehicle", h) end
    if p.y >= 0 then return "not behind the vehicle" end
    if p.z < 0.3 or p.z > 6 then return string.format("height %.2f m", p.z) end
    if shown then
      local d = math.sqrt((p.x - shown.x) ^ 2 + (p.y - shown.y) ^ 2 + (p.z - shown.z) ^ 2)
      if d > (tonumber(c.cfg.handGlideMaxDist) or 8) then return string.format("%.1f m from the shown pose", d) end
    end
    return nil
  end
  -- the vehicle TPP camera's rest pose (vehicle frame) for the ridden vehicle and perspective: learned this session
  -- (record + perspective) > class default; nil + reason when neither is usable
  function c.VehTppTarget(persp, shown)
    if not isCar() or persp == nil then return nil, "not a car" end
    local key = tostring(c.vehRecKey or "car") .. "|" .. tostring(persp)
    local reasons = {}
    local learned = c.vehTppLearned[key]
    if learned then
      local why = poseRejected(learned, shown)
      if not why then return learned, "learned" end
      reasons[#reasons + 1] = "learned pose " .. why
    end
    -- a pose measured for this very record (shipped table) before the class default
    local recs = c.cfg.vehTppRestRecords
    local rec = recs and c.vehRecKey and (recs[c.vehRecKey] or recs[(string.gsub(tostring(c.vehRecKey), "_player$", ""))])
    local recPose = rec and rec[persp]
    if recPose then
      local why = poseRejected(recPose, shown)
      if not why then return recPose, "record" end
      reasons[#reasons + 1] = "record pose " .. why
    end
    local rest = c.cfg.vehTppRest and c.cfg.vehTppRest.car and c.cfg.vehTppRest.car[persp]
    if rest then
      local why = poseRejected(rest, shown)
      if not why then return rest, "default" end
      reasons[#reasons + 1] = "default pose " .. why
    end
    if #reasons == 0 then reasons[1] = "no learned or default pose for " .. tostring(M.PERSP_NAME[persp] or persp) end
    return nil, table.concat(reasons, "; ")
  end
  -- start of the glide: the request phase is done (the vehicle reports a TPP perspective) and the pull-out is over (at the
  -- latest HandoverDelay after the phase end, the old hand-over time; the glide then composes with the pull-out tail).
  -- Decided once per mount hold: handMatch is set (the glide), or the engine blend follows at HandoverDelay.
  -- the glide lasts handGlideTime, longer when the distance to the target would exceed handGlideSpeed m/s peak
  -- (the eased path peaks at 1.5 x its mean speed), at most handGlideTimeMax: Bratsk 5.5 m 0.9 s = 9 m/s -> 1.65 s
  function c.GlideDuration(target, shown)
    local base = math.max(0.05, tonumber(c.cfg.handGlideTime) or 0.9)
    local vmax = tonumber(c.cfg.handGlideSpeed) or 0
    if vmax <= 0 or not shown then return base end
    local d = math.sqrt((target.x - shown.x) ^ 2 + (target.y - shown.y) ^ 2 + (target.z - shown.z) ^ 2)
    return math.min(math.max(base, tonumber(c.cfg.handGlideTimeMax) or base), math.max(base, 1.5 * d / vmax))
  end
  function c.TryHandGlide(clock, obs)
    if c.handGlideTried or c.handMatch ~= nil then return end
    if not (c.cfg.handGlide and isCar() and not c.cfg.ownHandoverGate and (c.rideRetakes or 0) == 0) then return end
    if c.phase ~= "done" or c.phaseEndAt == nil or not vehTpp(obs.activePersp) then return end
    if not c.pulling then return end
    if (c.pull or 0) < 1 and (clock - c.phaseEndAt) < c.HandoverDelay() then return end
    -- a pull-out slowed by its path (stored duration above pullTime) is not composed with the glide: a glide started mid-way blends towards a target
    -- 5 - 10 m away from a pose that still moves (12 m/s at mid-way for the 3.7 s truck path, against 6 m/s for each alone). It waits for the pull-out
    -- end, at the latest until handGlideTimeMax + handGlideAfterPullSlack s before keepTimeout (the glide must start before the timeout cuts the hold).
    if (c.pull or 0) < 1 and c.cfg.handGlideAfterPull ~= false and isCar() and c.pathStart ~= nil
       and (tonumber(c.pathStart.dur) or 0) > (tonumber(c.cfg.pullTime) or 0.9) + 1e-6
       and clock - (c.keepSince or clock) < (tonumber(c.cfg.keepTimeout) or 6) - (tonumber(c.cfg.handGlideTimeMax) or 1.7)
         - (tonumber(c.cfg.handGlideAfterPullSlack) or 0.3) then
      return
    end
    c.handGlideTried = true
    local shown = c.ShownVehPose()
    local persp = obs.activePersp
    local target, src, note = nil, nil, nil
    if shown == nil then
      note = "shown pose unknown"
    elseif (c.vehSpeed or 0) > (tonumber(c.cfg.handGlideMaxSpeed) or 2.0) then
      note = string.format("vehicle moves %.1f m/s", c.vehSpeed or 0)
    else
      if c.cfg.ownHandoverMatch then -- utppCamera is the active camera: the vehicleTPPCamera component only (seat in game)
        local pose = c.QueryGameCam(false)
        if pose and not poseRejected(pose, shown) then
          target, src = { x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pose.pitch or 0,
                          fov = tonumber(pose.fov) }, "read " .. tostring(pose.src)
        end
      end
      if not target then
        local t, how = c.VehTppTarget(persp, shown)
        if t then target, src = t, how else note = how end
      end
    end
    if not target then
      c.stats.handGlideMisses = (c.stats.handGlideMisses or 0) + 1
      log(string.format("handover glide: no vehicle TPP pose (%s) - engine blend %.2f s", tostring(note),
        math.max(tonumber(c.cfg.holdRelease) or 0, tonumber(c.cfg.handFallbackBlend) or 0)))
      return
    end
    local fov = tonumber(target.fov)
    c.handMatch = { x = target.x, y = target.y, z = target.z, yaw = target.yaw or 0, pitch = target.pitch or 0, fov = fov,
                    at = clock, dur = c.GlideDuration(target, shown), src = src, glide = true,
                    -- a class default and a learned pose that no later ride has confirmed (a first reading with a boom
                    -- shortened by a wall would be one) keep the longer residual blend
                    blend = (src == "default" or (src == "learned" and not target.confirmed))
                      and tonumber(c.cfg.handGlideDefaultBlend) or tonumber(c.cfg.handGlideResidualBlend) }
    if src == "record" then c.handMatch.blend = tonumber(c.cfg.handGlideRecordBlend) or c.handMatch.blend end
    c.glideTarget = { x = target.x, y = target.y, z = target.z, yaw = target.yaw or 0, pitch = target.pitch or 0, src = src,
                      persp = persp }
    c.stats.handGlides = (c.stats.handGlides or 0) + 1
    log(string.format("handover glide: %s target %s from shown %s over %.2f s, then Deactivate(%.2f)", src, poseStr(target),
      poseStr(shown), c.handMatch.dur, c.handMatch.blend or 0))
  end
  -- Learning (cars): the settled vehicle camera after the release, read from the rendered camera (utppCamera is not the
  -- active camera any more). Only a standing vehicle, a stable camera outside the vehicle, behind it, not orbited sideways.
  -- A later reading replaces a stored one only when it is not closer by more than 0.3 m (a collision shortens the boom).
  function c.UpdateLearn(clock, obs, vs)
    if not (c.cfg.vehTppLearn and isCar() and c.keep == "ride" and c.phase == "done" and vs == 1 and obs.mounted
            and c.handAt ~= nil and not c.Releasing() and vehTpp(obs.activePersp) and obs.activePersp <= 3
            and (c.vehSpeed or 0) < (tonumber(c.cfg.vehTppLearnSpeed) or 0.5)) then
      if c.learn and not c.learn.done then c.learn = nil end
      return
    end
    local L = c.learn
    if L == nil or L.persp ~= obs.activePersp then
      L = { persp = obs.activePersp, samples = {}, nextAt = clock }
      c.learn = L
    end
    if L.done or clock < L.nextAt then return end
    L.nextAt = clock + 0.05
    local pose = c.QueryGameCam(true)
    if not (pose and string.sub(tostring(pose.src), 1, 13) == "active camera") then L.samples = {}; return end
    L.samples[#L.samples + 1] = { t = clock, x = pose.x, y = pose.y, z = pose.z }
    local win = tonumber(c.cfg.vehTppLearnStable) or 0.2
    while #L.samples > 0 and clock - L.samples[1].t > win + 0.06 do table.remove(L.samples, 1) end
    local first = L.samples[1]
    if #L.samples < 3 or clock - first.t < win - 1e-6 then return end
    local tol = tonumber(c.cfg.vehTppLearnTol) or 0.05
    for _, s in ipairs(L.samples) do
      if math.sqrt((s.x - pose.x) ^ 2 + (s.y - pose.y) ^ 2 + (s.z - pose.z) ^ 2) > tol then return end
    end
    L.done = true
    local why = poseRejected(pose, nil)
    local az = math.deg(math.atan2(math.abs(pose.x), math.max(0.01, -pose.y)))
    if not why and az > (tonumber(c.cfg.vehTppLearnAzimuth) or 8) then why = string.format("orbited %.0f deg sideways", az) end
    local fv = tonumber(pose.fov)
    if fv and (fv < 10 or fv > 120) then fv = nil end
    local pitch, how = pose.pitch or 0, tostring(pose.src)
    if pose.unsure then -- 0 / 0: not proven
      local rp = tonumber(c.cfg.vehTppRestPitch)
      if rp ~= nil then
        -- the settled vehicle camera reads pitch 0 on every car (the rendered release converges to it): the read
        -- value is the rest pitch, not a look-at estimate (that one ended the glide 8-15 deg off)
        pitch, how = rp, "orientation reads 0 / 0, rest pitch"
      else
        local h = math.sqrt(pose.x * pose.x + pose.y * pose.y)
        pitch = math.deg(math.atan2((tonumber(c.cfg.vehTppLearnLookHeight) or 1.05) - pose.z, h))
        how = "orientation unread, pitch to the look-at point"
      end
    elseif pose.altSrc then how = tostring(pose.src) .. " (" .. tostring(pose.altSrc) .. ")" end
    local key = tostring(c.vehRecKey or "car") .. "|" .. tostring(obs.activePersp)
    local new = { x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pitch, fov = fv, src = "learned" }
    local old = c.vehTppLearned[key]
    if not why and old then
      local hOld, hNew = math.sqrt(old.x ^ 2 + old.y ^ 2), math.sqrt(new.x ^ 2 + new.y ^ 2)
      if hNew < hOld - 0.3 then why = string.format("%.2f m closer than the stored pose (collision?)", hOld - hNew) end
    end
    if why then
      c.stats.vehTppLearnRejects = (c.stats.vehTppLearnRejects or 0) + 1
      log(string.format("vehicle TPP pose not learned: %s %s %s - %s", key, tostring(M.PERSP_NAME[obs.activePersp]),
        poseStr(new), why))
      return
    end
    -- confirmed = an earlier ride's reading of the same record and perspective lies within 0.3 m of this one
    new.confirmed = old ~= nil and math.sqrt((new.x - old.x) ^ 2 + (new.y - old.y) ^ 2 + (new.z - old.z) ^ 2) <= 0.3
    c.vehTppLearned[key] = new
    c.stats.vehTppLearns = (c.stats.vehTppLearns or 0) + 1
    local errNote = ""
    local g = c.glideTarget
    if g and g.persp == obs.activePersp then
      -- the pitch error is against the pitch READ from the settled camera (pose.pitch: it used to compare two
      -- values from the same look-at formula and always read 0)
      errNote = string.format(", error vs the %s glide target %.2f m / %.0f deg yaw / %.1f deg pitch (read pitch %.1f)", g.src,
        math.sqrt((new.x - g.x) ^ 2 + (new.y - g.y) ^ 2 + (new.z - g.z) ^ 2), math.abs(wrap180(new.yaw - g.yaw)),
        math.abs((tonumber(pose.pitch) or 0) - g.pitch), tonumber(pose.pitch) or 0)
    end
    log(string.format("vehicle TPP pose learned: %s %s %s (%s)%s%s", key, tostring(M.PERSP_NAME[obs.activePersp]), poseStr(new),
      how, errNote, new.confirmed and " - confirmed by an earlier reading" or ""))
  end
  -- utppCamera takes the view back from a vehicle camera that does not follow the vehicle: chase pose behind
  -- the vehicle at once (pull weight 1), keep "mount" (hand-over tried again after HandoverDelay) or, final = true,
  -- keep "chase" (kept for the ride; the exit starts from the shown camera as after "exit before the hand-over")
  function c.RetakeChase(clock, why, final)
    local prev = c.keep
    c.keep, c.keepSince, c.keepReason = final and "chase" or "mount", clock, ""
    log(string.format("keepTpp %s -> %s (%s)", prev, c.keep, why))
    c.cam.hold, c.cam.releaseUntil = "mount", nil
    c.handMatch, c.handMatchTried, c.handGate = nil, false, nil
    c.handGlideTried = true -- a retake hands over by the engine blend as before (no glide while the camera is not following)
    c.phaseEndAt = clock
    c.pulling, c.pull, c.pathStart = true, 1, nil
    c.stats.camHolds = (c.stats.camHolds or 0) + 1
    if not final then c.stats.rideRetakes = (c.stats.rideRetakes or 0) + 1 end
    ownCmd("take", c.cfg.earlyExitBlend, why)
    c.stats.ownTakes = (c.stats.ownTakes or 0) + 1
  end
  -- ride watchdog (see M.DEFAULTS): obs.camX/Y/Z = active camera world position, c.vehSpeed from UpdateExitFollow
  function c.RideWatchdog(clock, obs)
    if not c.cfg.rideWatchdog or c.keep ~= "ride" or c.Releasing() or c.phase ~= "done" then c.wd = nil; return end
    if obs.camX == nil or obs.camY == nil or obs.camZ == nil then return end
    local spd = c.vehSpeed or 0
    if spd <= (tonumber(c.cfg.rideWatchdogSpeed) or 2.0) then c.wd = nil; return end
    local w = c.wd
    if w == nil then c.wd = { x = obs.camX, y = obs.camY, z = obs.camZ, t = clock }; return end
    local win = tonumber(c.cfg.rideWatchdogTime) or 0.3
    if clock - w.t < win then return end
    local d = math.sqrt((obs.camX - w.x) ^ 2 + (obs.camY - w.y) ^ 2 + (obs.camZ - w.z) ^ 2)
    c.wd = { x = obs.camX, y = obs.camY, z = obs.camZ, t = clock }
    if d >= (tonumber(c.cfg.rideWatchdogMove) or 0.2) then return end
    c.wd = nil
    c.wdTrips = (c.wdTrips or 0) + 1
    c.stats.rideWatchdogTrips = (c.stats.rideWatchdogTrips or 0) + 1
    local head = string.format("ride watchdog: active camera moved %.2f m in %.2f s at (%.2f,%.2f,%.2f) while the vehicle drives "
      .. "%.1f m/s - the vehicle camera is not following", d, clock - w.t, obs.camX, obs.camY, obs.camZ, spd)
    local maxR = tonumber(c.cfg.rideWatchdogMax) or 2
    if c.wdTrips == 1 then
      c.forceRequest = choose(obs.persistentPersp)
      c.stats.rideWatchdogRequests = (c.stats.rideWatchdogRequests or 0) + 1
      log(head .. string.format(": %s requested again (trip 1)", tostring(M.PERSP_NAME[c.forceRequest] or c.forceRequest)))
    elseif (c.rideRetakes or 0) < maxR then
      c.rideRetakes = (c.rideRetakes or 0) + 1
      log(head .. string.format(": utppCamera takes the view back (retake %d/%d, chase pose behind the vehicle, blend %.2f s), "
        .. "hand-over tried again after %.2f s", c.rideRetakes, maxR, c.cfg.earlyExitBlend, c.HandoverDelay()))
      c.RetakeChase(clock, string.format("ride watchdog retake %d/%d", c.rideRetakes, maxR), false)
    elseif not c.wdFinal then
      c.wdFinal = true
      c.stats.rideChases = (c.stats.rideChases or 0) + 1
      log(head .. string.format(": retakes exhausted (%d) - utppCamera keeps the chase pose for this ride", maxR))
      c.RetakeChase(clock, "ride watchdog: chase for the ride", true)
    end
  end
  -- mount / chase hold in a seat state: the game camera keeps the view ("game", HoldForVehicle(false, why, "game",
  -- togameBlend)) unless the seat is a confirmed passenger or the vehicle camera is FPP ("off": the camera's own rules)
  local function seatLeave(vs, obs)
    -- the passenger's vehicle camera takes the TPP request into the persistent perspective
    -- but stays FPP (active=0 for the whole ride), so a car passenger whose hold was taken keeps utppCamera on the vehicle-
    -- behind chase pose (keep "chase": exit from the shown pose, release to the orbit like the driver's exit)
    if vs == 3 and c.cfg.passengerChase and c.cfg.pullOut and isCar() and c.ownHeld and c.cfg.forceTppOnEnter then
      c.keep, c.keepReason, c.passChase = "chase", "passenger ride on the own camera", true
      c.cam.hold = "mount"
      ownCmd("hold", nil, "passenger chase")
      c.stats.passengerChases = (c.stats.passengerChases or 0) + 1
      log("passenger seat (state 3): utppCamera keeps the vehicle-behind chase pose for the ride (the passenger's vehicle camera "
        .. "does not go third person)")
      return nil
    end
    if vs == 3 or obs.driver == false or obs.activePersp == M.PERSP.FPP then
      c.keep, c.keepReason = "off", (vs == 3 or obs.driver == false) and "passenger" or ("vehicle state " .. vs)
      return nil
    end
    c.keep, c.keepReason = "game", "vehicle state " .. vs
    return tonumber(c.cfg.togameBlend) or 0.30
  end
  function c.UpdateOwn(clock, obs, vs, inVehicle)
    local prev = c.keep
    local gameBlend = nil
    if vs == 1 and obs.activePersp ~= nil then c.seatedActive = obs.activePersp end
    local sw = c.seatSwitch
    if sw and not sw.final and inVehicle and (vs == 1 or driverCombat(vs) or passengerState(vs)) then
      sw.final = true
      if passengerState(vs) then
        c.stats.seatSwitchPassengers = (c.stats.seatSwitchPassengers or 0) + 1
        log(string.format("seat final: passenger state %d %.2f s after the mount start (no switch to the driver seat), "
          .. "keep %s", vs, clock - sw.since, tostring(c.keep)))
      else
        c.stats.seatSwitchDrivers = (c.stats.seatSwitchDrivers or 0) + 1
        log(string.format("seat final: driver state %d %.2f s after the mount start (slot %s), keep %s - TPP held "
          .. "through the seat switch", vs, clock - sw.since, tostring(obs.seat), tostring(c.keep)))
      end
    end
    if not c.OwnEnabled() then
      if c.keep ~= "off" then c.keep, c.keepReason = "off", "switched off" end
    elseif not inVehicle then
      if prev == "exit" then
        c.unmountAt = clock
        camCmd("foot", 1, "unmount") -- RunOwnCmds -> utpp_camera.VehicleOnFoot(true): MM pose write allowed during the exit hold
        -- bikes hold shorter after the unmount. After a jump-out the hold is the follow pose around V
        -- (UpdateExitFollow starts it at this unmount frame at the latest), never the vehicle-behind pose
        local after = tonumber(c.vehBike and c.cfg.bikeExitHoldAfterUnmount or c.cfg.exitHoldAfterUnmount) or 0
        if c.cam.hold == "exit" then
          c.cam.holdUntil = math.min(clock + after, (c.keepSince or clock) + (tonumber(c.cfg.exitHoldMax) or 3.0))
          c.ExtendFollowHold()
          if c.cam.holdUntil > clock then
            log(string.format("exit: unmounted %.2f s after the exit start - vehicle-behind pose held %.2f s "
              .. "more, then %.2f s to the orbit", clock - (c.keepSince or clock), c.cam.holdUntil - clock, c.cfg.exitPullTime))
          else c.cam.holdUntil = nil end
        end
        local k = c.exitKeep
        if k then -- A result (the head slot losses are utpp_body's log)
          -- the persistent perspective seen during the exit (persistent=3 TPPFar the whole
          -- exit, active forced to FPP 16 ms after the exit start; the request only sets the persistent value, which is
          -- already TPPFar, so the native exit camera keeps FPP)
          log(string.format("exit %s: active stayed TPP=%s (%d of %d exit frames with the vehicle camera in FPP), "
            .. "%d request(s), persistent %s during the exit", k.passive and "vehicle camera (no requests, diagnostic)"
            or "keep result", tostring(k.fppFrames == 0), k.fppFrames, k.frames, k.sends,
            tostring(M.PERSP_NAME[k.persistent] or k.persistent)))
          c.exitKeep = nil
        end
      end
      if prev == "game" then c.keepReason = "" end -- left the vehicle (nothing held)
      c.keep = "off"
      c.keepReady = true
      c.seatPendingLogged, c.gameExitLogged, c.exitFromGame = false, false, false
      c.slotLagLogged, c.seatSwitch, c.passChase = false, nil, false
    elseif c.keep == "off" then
      if c.keepReady then
        c.TryStartOwn(clock, obs, "mount start")
        prev = c.keep -- logged by TryStartOwn
      end
      -- (1): a driver ride whose hold was never taken (passive): the game's vehicle camera has the view, the exit
      -- is taken from it
      if c.keep == "off" and not c.keepReady and vs == 1 then -- state 1 = driver (the slot may lag one frame)
        c.keep, c.keepReason = "game", "driver ride without the hold"
      end
      -- a passenger ride with the third-person request made by c.StepPassenger: the game's vehicle camera has the
      -- view (exit take-over from it, utpp_body's head callback), like a driver ride without the hold
      if c.keep == "off" and not c.keepReady and vs == 3 and c.passengerForced and vehTpp(obs.activePersp) then
        c.keep, c.keepReason = "game", "passenger ride in the game's third-person vehicle camera"
        -- a passenger ride starts in the vanilla FPP camera: the FPP -> TPP change is reported to utpp_body even though
        -- the game keep begins with the TPP reading
        c.gamePersp = M.PERSP.FPP
      end
    elseif c.keep == "mount" then
      if (vs == 1 or driverCombat(vs)) and c.phase ~= "armed" and c.phaseEndAt ~= nil then c.TryHandGlide(clock, obs) end
      if passengerState(vs) then gameBlend = seatLeave(vs, obs)
      elseif (clock - c.keepSince) > c.cfg.keepTimeout and not (c.handMatch ~= nil and c.handMatch.glide) then
        -- (an entry glide in progress is not cut by the cap: it ends within handGlideTime with the hand-over)
        c.keep, c.keepReason = "off", "timeout"
      elseif vs == 4 and c.sawDrive then startOwnExit(clock, "exit before the handover", false)
      elseif (vs == 1 or driverCombat(vs)) and c.phase ~= "armed" and c.phaseEndAt ~= nil
             and (clock - c.phaseEndAt) >= c.HandoverDelay()
             -- a hand-over without a glide waits for a slowed pull-out to end (pull -> 1 at the hand-over would jump)
             and not (c.handMatch == nil and c.pulling and (c.pull or 0) < 1 and isCar()
                      and c.pathStart ~= nil and (tonumber(c.pathStart.dur) or 0) > (tonumber(c.cfg.pullTime) or 0.9) + 1e-6) then
        -- never before the vehicle reports a TPP perspective and its camera blend is over (HandoverDelay)
        if c.phase ~= "done" or obs.activePersp == nil or obs.activePersp == M.PERSP.FPP then
          c.keep, c.keepReason = "off", "no vehicle TPP perspective (" .. tostring(c.reason) .. ")"
        elseif c.handMatch ~= nil then
          if clock - c.handMatch.at >= c.handMatch.dur then
            ownHandover(clock, c.handMatch.glide and ("glide matched: " .. tostring(c.handMatch.src)) or "pose matched first",
              c.handMatch.blend)
          end
        elseif c.cfg.ownHandoverMatch and (not c.handMatchTried or c.handGate ~= nil) then
          -- hand-over gate (ownHandoverGate): no readable pose -> wait on the chase pose, read again
          if c.handGate ~= nil and clock < c.handGate.nextAt then
            -- gated: waiting for the next read
          else
            c.handMatchTried = true
            local pose, err = c.QueryGameCam(false) -- utppCamera is the active camera: the vehicleTPPCamera component only
            if pose and isCar() then -- cars: the same sanity as the glide target (bikes keep the previous read)
              local rej = poseRejected(pose, c.ShownVehPose())
              if rej then pose, err = nil, string.format("%s at (%.2f,%.2f,%.2f) rejected: %s", tostring(pose.src), pose.x, pose.y, pose.z, rej) end
            end
            if pose then
              c.handMatch = { x = pose.x, y = pose.y, z = pose.z, yaw = pose.yaw or 0, pitch = pose.pitch or 0, at = clock,
                              dur = math.max(0.05, tonumber(c.cfg.ownHandoverMatchTime) or 0.25), src = tostring(pose.src) }
              c.stats.ownHandMatches = (c.stats.ownHandMatches or 0) + 1
              if c.handGate ~= nil then
                log(string.format("handover gate: vehicle camera pose readable after %.2f s (%d reads)", clock - c.handGate.since,
                  c.handGate.reads))
                c.handGate = nil
              end
              log(string.format("handover match: utppCamera -> %s (%.2f,%.2f,%.2f) yaw=%.0f pitch=%.0f in %.2f s, then the "
                .. "hand-over", c.handMatch.src, pose.x, pose.y, pose.z, c.handMatch.yaw, c.handMatch.pitch, c.handMatch.dur))
            elseif c.cfg.ownHandoverGate and (c.handGate == nil
                   or clock - c.handGate.since < (tonumber(c.cfg.ownHandoverGateMax) or 2.0)) then
              if c.handGate == nil then
                c.handGate = { since = clock, reads = 0 }
                c.stats.handGates = (c.stats.handGates or 0) + 1
                log(string.format("handover gated: no vehicle camera pose (%s) - utppCamera keeps the chase pose behind the "
                  .. "vehicle, pose read again every %.2f s for up to %.1f s", tostring(err),
                  tonumber(c.cfg.ownHandoverGateRetry) or 0.25, tonumber(c.cfg.ownHandoverGateMax) or 2.0))
              end
              c.handGate.reads = c.handGate.reads + 1
              c.handGate.nextAt = clock + (tonumber(c.cfg.ownHandoverGateRetry) or 0.25)
            else
              local gated = ""
              if c.handGate ~= nil then
                gated = string.format(" (gate timed out after %.2f s, %d reads)", clock - c.handGate.since, c.handGate.reads)
                c.handGate = nil
              end
              log(string.format("handover match: no vehicle camera pose (%s) - engine blend from the pull-out pose%s",
                tostring(err), gated))
              ownHandover(clock, "engine blend")
            end
          end
        else
          ownHandover(clock, "engine blend")
        end
      end
    elseif c.keep == "ride" then
      if obs.activePersp == M.PERSP.FPP then c.rideFppSince = c.rideFppSince or clock else c.rideFppSince = nil end
      local fpp = c.rideFppSince ~= nil and (clock - c.rideFppSince) >= (tonumber(c.cfg.ownRideFppGrace) or 0.3)
      if c.abortAsk ~= nil and clock <= c.abortAsk.untilAt then fpp = false end -- the aborted exit's request is still running
      -- (1): still mounted -> keep "game" (the exit is taken from the game camera), not "off"
      if passengerState(vs) then c.keep, c.keepReason = "game", "vehicle state " .. vs
      elseif fpp then c.keep, c.keepReason = "game", "vehicle camera FPP"
      elseif vs == 4 then
        local okW, wanted, whyNot = true, true, nil
        if type(c.ownExitQuery) == "function" then okW, wanted, whyNot = pcall(c.ownExitQuery) end
        if okW and wanted == false then c.keep, c.keepReason = "off", "exit in first person (" .. tostring(whyNot) .. ")"
        else startOwnExit(clock, "exit start", true) end
      end
    elseif c.keep == "exit" then
      if passengerState(vs) then c.keep, c.keepReason = "off", "vehicle state " .. vs
      elseif vs == 1 or driverCombat(vs) then -- an exit 4 -> 6 -> 4 is an aborted exit as 4 -> 1
        -- exit aborted / seat switch: the view goes back to the vehicle camera
        c.keep, c.keepReason = "ride", ""
        c.cam.hold, c.cam.early, c.cam.holdUntil = nil, false, nil
        c.exitPull, c.pull, c.release = nil, 0, nil
        endMatch("dropped (exit aborted)")
        ownCmd("drop", c.cfg.earlyExitBlend, "exit aborted - back to the vehicle camera")
        -- cars: the native exit forced the vehicle camera to FPP and the exit keep requests are off, so the ride
        -- would fall to keep "game" in FPP after ownRideFppGrace: ask for the entry perspective again (c.StepAbortAsk)
        if isCar() and c.cfg.abortExitRequest then
          c.abortAsk = { untilAt = clock + (tonumber(c.cfg.abortExitWindow) or 1.0), nextAt = clock, n = 0 }
          c.stats.abortAsks = (c.stats.abortAsks or 0) + 1
        end
      elseif (clock - c.keepSince) > c.cfg.keepExitTimeout then c.keep, c.keepReason = "off", "exit timeout"
      end
    elseif c.keep == "chase" then -- utppCamera on the chase pose for the ride (ride watchdog)
      if passengerState(vs) and not (c.passChase and vs == 3) then gameBlend = seatLeave(vs, obs)
      elseif vs == 4 then startOwnExit(clock, "exit start (chase)", false) end
    elseif c.keep == "game" then
      -- the game's vehicle camera has the view while mounted; vs 4 = the exit (game is entered only seated)
      if vs == 3 and not c.passengerForced then c.keep, c.keepReason = "off", "passenger"
      elseif vs == 4 then
        if not c.ownApiMotion then
          if not c.gameExitLogged then
            c.gameExitLogged = true
            log("exit from the game camera: utpp_camera has no TakeForVehicleExit (vehicle API < 2) - the camera "
              .. "re-enters on its own rules")
          end
        else
          local okW, wanted, whyNot = true, true, nil
          if type(c.ownExitQuery) == "function" then okW, wanted, whyNot = pcall(c.ownExitQuery) end
          if okW and wanted == false then c.keep, c.keepReason = "off", "exit in first person (" .. tostring(whyNot) .. ")"
          else
            -- the vehicle camera switches to FPP within a frame of the exit start: the last mounted frame decides
            c.exitVehTpp = vehTpp(c.lastActive) or vehTpp(obs.activePersp)
            startOwnExit(clock, "exit start (from the game camera)", "game")
          end
        end
      end
    end
    if c.keep ~= prev then
      log(string.format("keepTpp %s -> %s%s", prev, c.keep, (c.keepReason ~= "" and (" (" .. c.keepReason .. ")") or "")))
      if c.keep == "game" then
        local why = c.keepReason ~= "" and c.keepReason or "game camera"
        c.gameExitLogged = false
        if prev == "mount" or prev == "chase" then
          -- (3): the game's vehicle camera keeps the view (no FPP camera activation)
          c.cam.hold, c.pulling, c.pull, c.pathStart, c.handMatch = nil, false, 0, nil, nil
          if c.ownHeld then
            ownCmd("togame", gameBlend, "mount hold ended (" .. why .. ")")
            c.stats.keepFallbacks = (c.stats.keepFallbacks or 0) + 1
          end
        elseif prev == "ride" then
          if c.ownHeld then ownCmd("togame", nil, "ride ended (" .. why .. ")") end
          c.stats.keepFallbacks = (c.stats.keepFallbacks or 0) + 1
        end
      elseif c.keep == "off" then
        local why = c.keepReason ~= "" and c.keepReason or "left the vehicle"
        if c.keepReason ~= "" then c.stats.keepFallbacks = (c.stats.keepFallbacks or 0) + 1 end
        if prev == "mount" or prev == "chase" then
          -- the camera returns to its own rules (the vehicle decision cuts to first person, as before)
          c.cam.hold, c.pulling, c.pull, c.pathStart, c.handMatch = nil, false, 0, nil, nil
          if c.ownHeld then ownCmd("free", nil, "mount hold ended (" .. why .. ")") end
        elseif prev == "ride" then
          -- handed over: the game's vehicle camera keeps the view (on foot: the camera takes it back)
          if c.ownHeld then ownCmd(inVehicle and "togame" or "free", nil, "ride ended (" .. why .. ")") end
        elseif prev == "exit" and c.keepReason ~= "" and c.keepReason ~= "exit timeout" then
          c.cam.hold, c.cam.early, c.cam.holdUntil = nil, false, nil
          c.exitPull, c.pull, c.release = nil, 0, nil
          endMatch("dropped (" .. why .. ")")
          if c.ownHeld then ownCmd("free", nil, "exit hold ended (" .. why .. ")") end
        elseif prev == "exit" and c.keepReason == "exit timeout" then
          c.cam.hold, c.cam.early, c.cam.holdUntil = nil, false, nil -- the release to the orbit follows (UpdatePull)
        end
      end
    end
  end
  -- jump-out follow: vehicle speed from its position samples (only while mounted), V's distance to the vehicle centre
  -- (obs.vehDist while mounted, else the last pull frame: the vehicle pose is kept after the unmount); the follow starts
  -- once per exit, only while the exit hold (vehicle-behind pose) is up
  -- the exit hold of a follow around the vehicle lasts until the follow ends (the release to the orbit is a
  -- straight blend and would cross the body), at most exitFollowHoldMax s after the exit start
  function c.ExtendFollowHold()
    local ef = c.exitFollow
    if not (ef and ef.around and c.cam.hold == "exit" and c.unmountAt ~= nil) then return end
    local cap = (c.keepSince or 0) + (tonumber(c.cfg.exitFollowHoldMax) or 4.5)
    local want = math.min(ef.at + ef.dur + 0.05, cap)
    if want > (c.cam.holdUntil or 0) then c.cam.holdUntil = want end
  end
  -- native jump-out launch counter (Game.UTPPVehicleJumpOutEpoch -> number, nil when the DLL is older or the call fails; logged once)
  function c.NativeJumpOutEpoch()
    if c.cfg.exitJumpNative == false then return nil end
    local fn = Game and Game.UTPPVehicleJumpOutEpoch
    if type(fn) ~= "function" then
      if not c.joNativeLogged then c.joNativeLogged = true; log("Game.UTPPVehicleJumpOutEpoch unavailable (older UTPP.dll): the jump-out is classified by the vehicle speed only") end
      return nil
    end
    local ok, r = pcall(fn)
    if ok and type(r) == "number" then return r end
    if not c.joNativeLogged then c.joNativeLogged = true; log("Game.UTPPVehicleJumpOutEpoch failed (" .. tostring(r) .. "): the jump-out is classified by the vehicle speed only") end
    return nil
  end
  function c.UpdateExitFollow(clock, obs)
    if obs.vehX ~= nil and obs.vehY ~= nil then
      local sm = c.vehSample
      if sm and clock - sm.t > 1e-4 then
        c.vehSpeed = math.sqrt((obs.vehX - sm.x) ^ 2 + (obs.vehY - sm.y) ^ 2) / (clock - sm.t)
      end
      c.vehSample = { x = obs.vehX, y = obs.vehY, t = clock }
    end
    if c.keep == "exit" and not c.exitJump and (c.vehSpeed or 0) > (tonumber(c.cfg.exitFollowSpeed) or 4) then
      c.exitJump, c.exitJumpSpeed = true, true -- (exitJumpSpeed: classified by the vehicle speed, not by the native launch)
      camCmd("jumpout", 1, "jump-out") -- RunOwnCmds -> utpp_camera.VehicleJumpOut(true): the pivot's jump-out landing mode (a normal exit never sends it)
      log(string.format("exit while the vehicle moves %.1f m/s (> %.1f): jump-out - utppCamera follows V at the unmount or "
        .. "beyond %.1f m", c.vehSpeed, tonumber(c.cfg.exitFollowSpeed) or 4, tonumber(c.cfg.exitFollowDist) or 2))
    end
    -- the exit of a standing or slow vehicle can still be a jump-out (double F): the native launch edge arms the same mode. Polled once per
    -- frame while an exit is in progress (the exit hold, also after the unmount) and no jump-out is known yet.
    if not c.exitJump and (c.keep == "exit" or (c.unmountAt ~= nil and c.cam.hold == "exit")) then
      local e = c.NativeJumpOutEpoch()
      if e ~= nil then
        if c.joEpoch == nil then c.joEpoch = e
        elseif e ~= c.joEpoch then
          c.joEpoch = e
          c.exitJump = true
          camCmd("jumpout", 1, "jump-out (native launch)")
          log(string.format("jump-out launch reported by the native (epoch %d, vehicle %.1f m/s): jump-out landing mode - utppCamera follows V", e, c.vehSpeed or 0))
          -- the side push of this launch (the wiring runs it once with the live car / V poses: M.JoPushRun); off = nothing is queued
          if c.cfg.jumpOutSidePush ~= false then c.joPush = { epoch = e, at = clock, speed = c.vehSpeed or 0, car = isCar() } end
        end
      end
    end
    if not c.cfg.exitFollow or c.exitFollow ~= nil or c.exitPull ~= "hold" then return end
    local dist = obs.vehDist
    local f = c.pullFrame
    if dist == nil and f then dist = math.sqrt((f.px - f.vx) ^ 2 + (f.py - f.vy) ^ 2) end
    local why = nil
    local rise = (f and c.unmountAt ~= nil) and (f.pz - f.vz - (tonumber(c.vehBike and c.cfg.bikeSeatHeight
      or c.cfg.carSeatHeight) or 0.6)) or nil
    -- the gate is relative to V's distance at the exit start for a car (the seat of a long truck is 3.7 m from
    -- the origin: the fixed 2.0 m fired at the exit start)
    if c.exitStartDist == nil and dist ~= nil and c.keep == "exit" then c.exitStartDist = dist end
    local gate = tonumber(c.cfg.exitFollowDist) or 2
    if isCar() and c.exitStartDist ~= nil and not c.exitJump and (c.vehSpeed or 0) <= (tonumber(c.cfg.exitFollowSpeed) or 4) then
      gate = math.max(gate, c.exitStartDist + (tonumber(c.cfg.exitFollowSlack) or 1.2)) -- standing car only (jump-outs keep 2.0 m)
    end
    local farSeat = isCar() and c.cfg.exitFollowAround and (c.exitStartDist or 0) > (tonumber(c.cfg.exitFollowAroundSeat) or 2.5)
    if dist ~= nil and dist > gate then
      why = string.format("V %.2f m from the vehicle centre", dist)
    elseif farSeat and c.unmountAt ~= nil and not c.exitJump then
      why = string.format("unmount of a car seat %.2f m from the vehicle centre", c.exitStartDist or 0)
    elseif c.unmountAt ~= nil and c.exitJump then
      why = string.format("unmount after a jump-out, vehicle %.1f m/s", c.vehSpeed or 0)
    elseif rise ~= nil and rise > (tonumber(c.cfg.exitFollowRise) or 0.6) then
      -- (a): V launched upward after the unmount (capsule above the vehicle origin + seat height)
      why = string.format("V %.2f m above the seat height after the unmount", rise)
    elseif c.vehBike and c.unmountAt ~= nil and dist ~= nil and dist > (tonumber(c.cfg.bikeExitFollowDist) or 1.2) then
      -- (b): bikes follow from a shorter distance after the unmount
      why = string.format("V %.2f m from the motorcycle after the unmount", dist)
    end
    if not why then return end
    c.exitFollow = { at = clock, dur = math.max(0.05, tonumber(c.cfg.exitFollowBlend) or 0.5), why = why }
    -- a standing car (no jump-out, no launch): the follow runs around the vehicle, its duration is set from the path in OwnPose
    -- (standard3 'parallel' exit): a rolling exit (4.2 m/s at the exit start: jump-out by the speed) whose car stood
    -- still at the unmount (0.0 m/s) took the straight 0.25 s follow; the exit pose governor (5 m/s beyond V) had not finished it when the hold ended and the
    -- release ran straight from behind the car to the orbit through the rear corner of the box (7 frames, 0.14 m). exitFollowAroundAtRest: such an exit
    -- (classified by the speed only, the car below exitFollowRestSpeed m/s at the unmount, V not above the seat height) follows around the vehicle as a
    -- standing car does; a car still moving at the unmount, a native launch (double F) and a launched V keep the straight follow. false = off.
    local restJump = c.exitJump and c.exitJumpSpeed == true and c.cfg.exitFollowAroundAtRest ~= false and c.unmountAt ~= nil
      and (c.vehSpeed or 0) <= (tonumber(c.cfg.exitFollowRestSpeed) or 0.5)
    if isCar() and c.cfg.exitFollowAround and (not c.exitJump or restJump) and not (rise ~= nil and rise > (tonumber(c.cfg.exitFollowRise) or 0.6))
       and (c.vehSpeed or 0) <= (tonumber(c.cfg.exitFollowSpeed) or 4) then
      c.exitFollow.around = true
      if restJump then why = why .. ", the vehicle at rest: around the vehicle"; c.exitFollow.why, c.exitFollow.rest = why, true end
      c.ExtendFollowHold()
    end
    c.stats.exitFollows = (c.stats.exitFollows or 0) + 1
    log(string.format("exit follow (%s, %.2f s after the exit start): vehicle-behind pose -> orbit around V "
      .. "(pitch clamped to the framing range) "
      .. "over %.2f s", why, clock - (c.keepSince or clock), c.exitFollow.dur))
  end
  -- passengers (every vehicle entry starts in third person). The vanilla PassengerEvents.OnEnter
  -- queues an FPP vehicle camera request itself (after an optional TPPClose one), so the third-person request follows once
  -- the vehicle reports FPP in the passenger state (PSM Vehicle 3), passengerRequestDelay s after its first frame; retries
  -- every keepRequestInterval, at most keepMaxRequests. Never in the seat-switch window of a passenger-side entry (state 4
  -- until the driver state), in seat states 2 / 5 / 7 or in scenes. The ride then runs as keep "game" (UpdateOwn): the
  -- exit is taken from the game's vehicle camera, utpp_body gets the vehicle camera callback.
  function c.StepPassenger(clock, obs, vs, inVehicle)
    if not inVehicle then
      c.passSince, c.passDone, c.passengerForced, c.passReqs, c.passLastReq, c.passTppSince = nil, false, false, 0, nil, nil
      return nil
    end
    if c.passDone or not (c.cfg.enabled and c.cfg.forceTppOnEnter and c.cfg.forceTppPassenger) then return nil end
    if not isCar() then return nil end -- cars only (positively identified): bikes and unreadable vehicles are unchanged
    if c.passChase then return nil end -- the own camera has the ride (seatLeave): no vehicle camera request
    if vs == 1 or vs == 6 or vs == 2 or vs == 5 or vs == 7 or (vs == 4 and c.passSince ~= nil) then
      c.passDone = true
      if c.passSince ~= nil then
        log(string.format("always third person on entry: passenger ride ends the request (state %d)", vs))
      end
      return nil
    end
    if vs ~= 3 or not obs.mounted then return nil end
    local high = obs.highLevel
    if type(high) == "number" and high >= (c.cfg.forceSceneMin or 3) and high <= (c.cfg.forceSceneMax or 6) then
      c.passDone = true
      log("always third person on entry: no passenger request (scene)")
      return nil
    end
    c.passSince = c.passSince or clock
    local since = clock - c.passSince
    if since < (tonumber(c.cfg.passengerRequestDelay) or 0.15) then return nil end
    local a = obs.activePersp
    if a == nil then return nil end
    -- the vanilla OnEnter can queue TPPClose and then FPP (its timing is not in any log), so a TPP reading is not final: the
    -- request follows whenever the camera reads FPP inside the window (passengerSettle s). A TPP that holds for
    -- passengerTppHold s ends the request, so a deliberate FPP switch after that is respected; so does the window's end.
    if since > (tonumber(c.cfg.passengerSettle) or 3.0) then
      c.passDone = true
      -- (the request line is logged once per ride by the wiring; this is the summary of the retries)
      if (c.passReqs or 0) > 0 and a == M.PERSP.FPP then
        log(string.format("always third person on entry: passenger request ended after %d request(s) in %.1f s - the vehicle "
          .. "camera stayed %s (persistent %s)", c.passReqs, since, tostring(M.PERSP_NAME[a] or a),
          tostring(M.PERSP_NAME[obs.persistentPersp] or obs.persistentPersp)))
      end
      return nil
    end
    if a ~= M.PERSP.FPP then
      if not c.passengerForced then
        log(string.format("always third person on entry: passenger ride, the vehicle camera reads %s %.2f s after the passenger "
          .. "state began (watched until it holds %.1f s, at most %.1f s: the vanilla FPP request may follow)",
          tostring(M.PERSP_NAME[a] or a), since, tonumber(c.cfg.passengerTppHold) or 0.5, tonumber(c.cfg.passengerSettle) or 3.0))
      end
      c.passengerForced = true
      c.passTppSince = c.passTppSince or clock
      if clock - c.passTppSince >= (tonumber(c.cfg.passengerTppHold) or 0.5) then c.passDone = true end
      return nil
    end
    c.passTppSince = nil
    c.passengerForced = true
    if (c.passReqs or 0) >= (tonumber(c.cfg.keepMaxRequests) or 12) then
      c.passDone = true
      log("always third person on entry: passenger request given up (max requests)")
      return nil
    end
    if c.passLastReq ~= nil and (clock - c.passLastReq) < (tonumber(c.cfg.keepRequestInterval) or 0.25) then return nil end
    local p = choose(obs.persistentPersp)
    c.passReqs, c.passLastReq = (c.passReqs or 0) + 1, clock
    c.stats.passengerRequests = (c.stats.passengerRequests or 0) + 1
    c.stats.lastRequest = p
    c.rideBase[rideClass()] = p
    if c.passReqs == 1 then
      log(string.format("always third person on entry: passenger ride - request %s (state 3, vehicle camera FPP %.2f s after "
        .. "the passenger state began; retry every %.2f s, max %d)", M.PERSP_NAME[p], clock - c.passSince,
        tonumber(c.cfg.keepRequestInterval) or 0.25, tonumber(c.cfg.keepMaxRequests) or 12))
    end
    return p
  end
  -- aborted car exit (exit -> ride): the perspective of the entry (choose: rideBase, never an in-ride toggle) is
  -- requested every keepRequestInterval inside abortExitWindow s until the vehicle camera reads TPP; nothing is stored
  function c.StepAbortAsk(clock, obs, vs)
    local a = c.abortAsk
    if a == nil then return nil end
    if c.keep ~= "ride" or clock > a.untilAt or (vs ~= 1 and not driverCombat(vs)) then c.abortAsk = nil; return nil end
    if vehTpp(obs.activePersp) then
      log(string.format("aborted exit: the vehicle camera is %s again (%d request(s))", tostring(M.PERSP_NAME[obs.activePersp]), a.n))
      c.abortAsk = nil
      return nil
    end
    if clock < a.nextAt or a.n >= (tonumber(c.cfg.keepMaxRequests) or 12) then return nil end
    a.n, a.nextAt = a.n + 1, clock + (tonumber(c.cfg.keepRequestInterval) or 0.25)
    local p = choose(obs.persistentPersp)
    c.stats.abortRequests = (c.stats.abortRequests or 0) + 1
    c.stats.lastRequest = p
    if a.n == 1 then
      log(string.format("aborted exit: the vehicle camera reads %s - request %s again (every %.2f s for up to %.1f s)",
        tostring(M.PERSP_NAME[obs.activePersp] or obs.activePersp), M.PERSP_NAME[p], tonumber(c.cfg.keepRequestInterval) or 0.25,
        tonumber(c.cfg.abortExitWindow) or 1.0))
    end
    return p
  end
  -- One controller step: returns the perspective to request or nil
  function c.Step(clock, obs)
    c.clock = clock
    local vs = obs.vehicleState or 0
    c.stats.lastState = vs
    if obs.activePersp ~= nil then c.stats.lastActive = obs.activePersp end
    local inVehicle = (vs ~= 0) or (obs.mounted == true)
    local dtStep = (c.lastStepClock ~= nil) and math.max(0, clock - c.lastStepClock) or 0
    c.lastStepClock = clock
    -- (1): IsTPPActive of this and the previous frame (nil = unread / not mounted)
    c.vehTppPrev, c.vehTppNow = c.vehTppNow, (obs.mounted and type(obs.vehTppActive) == "boolean") and obs.vehTppActive or nil
    c.UpdateOwn(clock, obs, vs, inVehicle)
    c.UpdateExitFollow(clock, obs)
    c.RideWatchdog(clock, obs)
    c.UpdateLearn(clock, obs, vs)
    c.lastReqKind = nil
    local result = c.StepRide(clock, obs, vs, inVehicle)
    if result ~= nil then c.lastReqKind = "ride" end
    if result == nil then -- passengers
      result = c.StepPassenger(clock, obs, vs, inVehicle)
      if result ~= nil then c.lastReqKind = "passenger" end
    end
    if result == nil and c.abortAsk ~= nil then
      result = c.StepAbortAsk(clock, obs, vs)
      if result ~= nil then c.lastReqKind = "exitAbort" end
    end
    if c.forceRequest ~= nil then -- ride watchdog: one perspective request again
      if result == nil then result, c.lastReqKind = c.forceRequest, "watchdog" end
      c.forceRequest = nil
    end
    -- A: during the exit (state 4, still mounted) the seated TPP perspective is requested on the first exit frame,
    -- then every exitKeepInterval s, at most exitKeepMax times; never after the unmount
    local k = c.exitKeep
    if k and c.keep ~= "exit" then c.exitKeep, k = nil, nil end
    if k and vs == 4 and obs.mounted then
      k.frames = k.frames + 1
      k.persistent = obs.persistentPersp
      if obs.activePersp == M.PERSP.FPP then k.fppFrames = k.fppFrames + 1 end
      if not k.passive and result == nil and k.sends < (tonumber(c.cfg.exitKeepMax) or 8) and clock >= k.nextAt - 1e-9 then
        k.sends, k.nextAt = k.sends + 1, clock + (tonumber(c.cfg.exitKeepInterval) or 0.10)
        result, c.lastReqKind = k.p, "exitKeep"
        c.stats.exitKeepRequests = (c.stats.exitKeepRequests or 0) + 1
        if k.sends == 1 then
          log(string.format("exit keep vehicle TPP: request %s, active=%s (every %.2f s while exiting, max %d)",
            tostring(M.PERSP_NAME[k.p] or k.p), tostring(obs.activePersp), tonumber(c.cfg.exitKeepInterval) or 0.10,
            tonumber(c.cfg.exitKeepMax) or 8))
        end
      end
    end
    -- vehicle camera perspective changes while the game's vehicle camera has the view -> utpp_body's callback
    if c.keep == "game" and obs.mounted then
      local a = obs.activePersp
      if type(a) == "number" then
        if c.gamePersp ~= nil and a ~= c.gamePersp then
          local prevP = c.gamePersp
          c.stats.gamePerspChanges = (c.stats.gamePerspChanges or 0) + 1
          log(string.format("game vehicle camera: %s -> %s", tostring(M.PERSP_NAME[prevP] or prevP),
            tostring(M.PERSP_NAME[a] or a)))
          local cb = M.onVehiclePerspective
          if type(cb) == "function" then
            local okH, errH = pcall(cb, prevP, a)
            if not okH then log("onVehiclePerspective error: " .. tostring(errH)) end
          end
        end
        c.gamePersp = a
      end
    else
      c.gamePersp = nil
    end
    -- high priority before the first request: the vehicle camera is activated when the vehicle takes the perspective
    -- and would be rendered over utppCamera (its own FPP -> TPP transition) until the handover
    if result ~= nil and c.requests == 1 and c.keep == "mount" and c.cfg.priorityHold and c.cam.hold == nil and c.ownHeld then
      c.cam.hold = "mount"
      c.stats.camHolds = (c.stats.camHolds or 0) + 1
      ownCmd("hold", nil, "first request")
      log("camera hold: utppCamera at high priority until the view is handed over (vehicle camera transition hidden)")
    end
    if c.cam.hold == "mount" and c.keep ~= "mount" then c.cam.hold = nil end
    if c.cam.hold == "exit" and c.keep ~= "exit" and not c.ExitHoldExtended() then
      c.cam.hold, c.cam.early, c.cam.holdUntil = nil, false, nil
      log(string.format("exit hold ended (%.2f s after the exit start%s) - release to the orbit", clock - (c.keepSince or clock),
        c.unmountAt and string.format(", %.2f s after the unmount", clock - c.unmountAt) or ""))
    end
    if c.cam.releaseUntil ~= nil and clock > c.cam.releaseUntil then
      c.cam.releaseUntil, c.handMatch = nil, nil
      log("camera release done (utppCamera handed over to the vehicle camera)")
    end
    c.UpdatePull(clock, obs, dtStep)
    if c.match ~= nil then
      if c.exitPull == nil or not c.cfg.exitMatch then endMatch("dropped (no exit pull / switched off)")
      elseif clock - c.match.at >= c.match.dur then
        endMatch(string.format("done - utppCamera at the %s pose", c.exitPull == "hold" and "vehicle-behind" or "release"))
      end
    end
    if obs.mounted then c.lastActive = obs.activePersp end
    -- safety net: a hold never outlives its vehicle phases
    if c.ownHeld and c.keep == "off" and c.exitPull == nil then ownCmd("free", nil, "hold without a vehicle phase") end
    return result, nil
  end

  return c
end

-- mount jump fix (a): player-local camera position -> vehicle frame cylinder { a (rad, 0 = vehicle right, +90 deg =
-- forward), r (m, horizontal from the vehicle origin), h (m above it) }. f = ctrl.pullFrame (yaws deg CCW).
function M.VehCyl(lx, ly, lz, f)
  local rad = math.pi / 180
  local c, s = math.cos(f.pyaw * rad), math.sin(f.pyaw * rad)
  local wx, wy, wz = f.px + c * lx - s * ly, f.py + s * lx + c * ly, f.pz + lz
  local rx, ry = wx - f.vx, wy - f.vy
  local cv, sv = math.cos(-f.vyaw * rad), math.sin(-f.vyaw * rad)
  local ux, uy = cv * rx - sv * ry, sv * rx + cv * ry
  return { a = math.atan2(uy, ux), r = math.sqrt(ux * ux + uy * uy), h = wz - f.vz }
end
-- Pose on the path from start (VehCyl) to the pull-out target (pullBack behind, pullHeight above) at weight w: angle
-- along the shorter arc, radius and height linear - the camera goes around the vehicle, never across it. Returns the
-- player-local position and the local yaw / pitch (deg) looking at the pullLookHeight point; w = 1 equals M.PullPose.
-- keep-out box of the vehicle (see pullHull in DEFAULTS): vehicle frame, x right, y forward, z up from the origin.
-- HullFor: the box of a vehicle record (the "_player" suffix is ignored), missing keys from hullDefault.
function M.HullFor(cfg, key)
  local d = cfg.hullDefault or {}
  local recs, h = cfg.hullRecords, nil
  if type(recs) == "table" and key ~= nil then
    local k = tostring(key)
    h = recs[k] or recs[(string.gsub(k, "_player$", ""))]
  end
  h = h or d
  -- the top profile of this record only (never inherited from hullDefault), off with cfg.hullProfile = false; a malformed one is ignored (flat top)
  local prof = nil
  if cfg.hullProfile ~= false and type(h.topY) == "table" and #h.topY >= 2 then
    prof = h.topY
    for i = 1, #prof do
      local q = prof[i]
      if type(q) ~= "table" or tonumber(q[1]) == nil or tonumber(q[2]) == nil or (i > 1 and q[1] <= prof[i - 1][1]) then prof = nil; break end
    end
  end
  return { hx = h.hx or d.hx or 1.15, hyF = h.hyF or d.hyF or 2.6, hyB = h.hyB or d.hyB or 2.6, top = h.top or d.top or 1.7,
           bottom = h.bottom or d.bottom or -1.2, topY = prof }
end
-- the top of the box at y (vehicle frame): the profile topY (linear between its knots, the end values beyond them, never above top), else top
function M.HullTop(h, y)
  local p = h.topY
  if p == nil then return h.top end
  local n, v = #p, nil
  if y <= p[1][1] then v = p[1][2]
  elseif y >= p[n][1] then v = p[n][2]
  else
    for i = 2, n do
      if y <= p[i][1] then
        local a, b = p[i - 1], p[i]
        v = a[2] + (b[2] - a[2]) * (y - a[1]) / (b[1] - a[1])
        break
      end
    end
  end
  return math.min(h.top, v or h.top)
end
-- Depth of a vehicle-frame point inside the box: > 0 inside (distance to the nearest face), <= 0 outside (minus the largest
-- overshoot of a face). grow > 0 enlarges the box on every side. the top face is the profile at the point's y (M.HullTop; the vertical
-- distance to it).
function M.HullDepth(h, x, y, z, grow)
  grow = grow or 0
  local dx = (h.hx + grow) - math.abs(x)
  local dy = (y >= 0) and (h.hyF + grow - y) or (h.hyB + grow + y)
  local dz = math.min(M.HullTop(h, y) + grow - z, z - (h.bottom - grow))
  return math.min(dx, dy, dz)
end
-- radius at which the ray of angle a (rad, vehicle frame) leaves the box seen from the origin
local function hullPolar(h, a)
  local ca, sa = math.cos(a), math.sin(a)
  local t = math.huge
  if math.abs(ca) > 1e-9 then t = math.min(t, h.hx / math.abs(ca)) end
  if math.abs(sa) > 1e-9 then t = math.min(t, ((sa >= 0) and h.hyF or h.hyB) / math.abs(sa)) end
  return t
end
-- 1 below the top of the box, smoothly 0 `ramp` m above it (the path may run over the roof); the top at the point's y (M.HullTop)
local function hullZf(h, y, z, ramp)
  local top = M.HullTop(h, y)
  if z <= top then return 1 end
  if ramp <= 1e-6 or z >= top + ramp then return 0 end
  local k = 1 - (z - top) / ramp
  return k * k * (3 - 2 * k)
end
-- Plain path (no clearance, no clamp) in the vehicle frame at weight w: x, y, height and the angle (rad) that the look-at yaw follows. start.route nil =
-- the cylinder path (angle along the shorter arc, radius and height linear), else the exit leg (straight, to the point beside the car) and the arc from there.
local function pullPathVeh(start, w, cfg)
  local aT, rT, hT = -math.pi / 2, cfg.pullBack, cfg.pullHeight
  local rt = start.route
  if rt ~= nil and w <= rt.wE then
    local u = w / rt.wE
    return start.lx + (rt.ex - start.lx) * u, start.ly + (rt.ey - start.ly) * u, start.h + (rt.eh - start.h) * u, rt.aE, false
  elseif rt ~= nil then
    local u = (w - rt.wE) / (1 - rt.wE)
    local a, r = rt.aE + rt.da * u, rt.rE + (rT - rt.rE) * u
    return r * math.cos(a), r * math.sin(a), rt.eh + (hT - rt.eh) * u, a, true
  end
  local da = aT - start.a
  while da > math.pi do da = da - 2 * math.pi end
  while da <= -math.pi do da = da + 2 * math.pi end
  local a, r = start.a + da * w, start.r + (rT - start.r) * w
  return r * math.cos(a), r * math.sin(a), start.h + (hT - start.h) * w, a, true
end
-- first weight from which the plain path stays outside the box (0 = it never is inside)
local function pullExitWeight(start, cfg, hull)
  local last = 0
  for i = 0, 60 do
    local x, y, z = pullPathVeh(start, i / 60, cfg)
    if M.HullDepth(hull, x, y, z) > 0.02 then last = i + 1 end
  end
  return math.min(1, last / 60)
end
-- Pull-out plan of a car (called once at the freeze of the start; start = VehCyl result + lx, ly, lz in the vehicle frame; (vx, vy) = V in the
-- vehicle frame). start.hull = the box. A start inside the box keeps the cylinder path when that leaves the box first (the truck at its cab wall: about
-- 0.2 s) and takes an exit leg (start.route) through V's side to a point beside the car when that is faster (a start in the cabin); start.clampFrom = the
-- weight from which the radius clamp may act (the cylinder path of an inside start: where it has left the box). A start outside keeps the cylinder path.
function M.PullPlan(start, cfg, hull, vx, vy)
  start.hull, start.route, start.clampFrom = hull, nil, 0
  if hull == nil or start.lx == nil then return start end
  if M.HullDepth(hull, start.lx, start.ly, start.lz or start.h) <= (tonumber(cfg.hullPlanDepth) or 0.05) then return start end
  local wCyl = pullExitWeight(start, cfg, hull)
  local side = (math.abs(vx or 0) > 0.2) and ((vx < 0) and -1 or 1) or ((start.lx < 0) and -1 or 1)
  local ex = side * (hull.hx + (tonumber(cfg.hullClampMargin) or 0.15) + (tonumber(cfg.pullExitSide) or 0.4))
  local ey = math.min(start.ly, (vy or start.ly) - (tonumber(cfg.pullExitBehind) or 1.1)) -- out through the side at once; not within the clearance of V
  local hT = cfg.pullHeight
  local eh = start.h + 0.2 * (hT - start.h)
  local aE = math.atan2(ey, ex)
  local da = -math.pi / 2 - aE
  while da > math.pi do da = da - 2 * math.pi end
  while da <= -math.pi do da = da + 2 * math.pi end
  start.route = { ex = ex, ey = ey, eh = eh, aE = aE, rE = math.sqrt(ex * ex + ey * ey), da = da,
                  wE = math.max(0.05, math.min(0.8, tonumber(cfg.pullExitFrac) or 0.2)) }
  local wRoute = pullExitWeight(start, cfg, hull)
  start.exitCyl, start.exitRoute = wCyl, wRoute
  if wRoute <= wCyl then return start end
  start.route, start.clampFrom = nil, wCyl
  return start
end
-- unwrapped end angle of the path (rad)
local function pullEndAngle(start)
  local rt = start.route
  if rt then return rt.aE + rt.da end
  local da = -math.pi / 2 - start.a
  while da > math.pi do da = da - 2 * math.pi end
  while da <= -math.pi do da = da + 2 * math.pi end
  return start.a + da
end
-- World look-at yaw (deg, unwrapped) at the end of the path: the continuous look-at yaw of the path ends here (+ k 360)
function M.PullEndYaw(start, f)
  return math.deg(pullEndAngle(start)) + f.vyaw + 90
end
-- Last return value: the look-at yaw of the path in degrees, UNWRAPPED (continuous in w: the angle of the path around the vehicle + 90 + the vehicle
-- yaw), the earlier yaw return is that value wrapped to +-180 and relative to the player as before.
local function pullPathRaw(start, w, f, cfg)
  local rad = math.pi / 180
  local rT = cfg.pullBack
  local rt = start.route
  local ux, uy, h, a, polar = pullPathVeh(start, w, cfg)
  local r = math.sqrt(ux * ux + uy * uy)
  local hl = start.hull
  if polar and hl ~= nil then
    -- the radius never goes below the box (cylinder legs only): faded in over 0.1 of the weight (from the weight where an inside start has left the box,
    -- clampFrom), no margin at w = 0 (the frozen pose stays as it is), never above the target radius
    local m = (tonumber(cfg.hullClampMargin) or 0.15) * smooth01(w / 0.15)
    local rmin = math.min(hullPolar(hl, a) + m, rT)
    if r < rmin then
      r = r + (rmin - r) * hullZf(hl, uy, h, tonumber(cfg.hullRamp) or 0.1) * smooth01((w - (start.clampFrom or 0)) / 0.1)
      ux, uy = r * math.cos(a), r * math.sin(a)
    end
  end
  local cv, sv = math.cos(f.vyaw * rad), math.sin(f.vyaw * rad)
  local cx, cy, cz = f.vx + cv * ux - sv * uy, f.vy + sv * ux + cv * uy, f.vz + h
  local dx, dy, dz = f.vx - cx, f.vy - cy, f.vz + cfg.pullLookHeight - cz
  local yawU = math.deg(a) + f.vyaw + 90
  local camYaw
  if polar then
    camYaw = math.atan2(-dx, dy) / rad
  else
    camYaw = math.atan2(math.sin(yawU * rad), math.cos(yawU * rad)) / rad
    -- the exit leg looks along its reference angle: the horizontal distance of the pitch is that of the leg's end point
    dx, dy = -rt.rE * math.cos(rt.aE + rad * f.vyaw), -rt.rE * math.sin(rt.aE + rad * f.vyaw)
  end
  local camPitch = math.atan2(dz, math.sqrt(dx * dx + dy * dy)) / rad * cfg.pullPitchSign
  local c, s = math.cos(-f.pyaw * rad), math.sin(-f.pyaw * rad)
  local wx, wy = cx - f.px, cy - f.py
  return { x = c * wx - s * wy, y = s * wx + c * wy, z = cz - f.pz }, camYaw - f.pyaw, camPitch, c, s, yawU
end
-- height factor of the clearance, 1 inside [z0, z1] and a smoothstep ramp of `ramp` m outside each edge (a path crossing the edge while
-- close to V's axis used to pop sideways 0.45 - 0.62 m in one frame). ramp <= 0: the hard band.
local function pullClearZf(z, z0, z1, ramp)
  if z >= z0 and z <= z1 then return 1 end
  if ramp <= 1e-6 then return 0 end
  local k = (z < z0) and (z - (z0 - ramp)) / ramp or 1 - (z - z1) / ramp
  if k <= 0 then return 0 end
  if k >= 1 then return 1 end
  return k * k * (3 - 2 * k)
end
-- ONE side per pull for the clearance push. The earlier push was radial from V's axis, so a raw path that passes near the axis swung the pushed
-- point around V at the clearance radius (and flipped side within one path step on an exact pass: 1.57 m per 1/400 of the path against 0.02 m raw). The
-- side n is the normal of the chord of the raw path's part that lies inside the clearance, on the side where that part lies (its mean seen from V's axis);
-- a path through the axis itself (no side to speak of) takes the normal that points away from the vehicle centre. Depends on the start and the end of the
-- path only (not on w).
local function pullClearSide(start, f, cfg, clr, z0, z1, ramp, c, s)
  local sx, sy, n, first, last = 0, 0, 0, nil, nil
  for i = 0, 32 do
    local p = pullPathRaw(start, i / 32, f, cfg)
    if math.sqrt(p.x * p.x + p.y * p.y) < clr * pullClearZf(p.z, z0, z1, ramp) then
      sx, sy, n = sx + p.x, sy + p.y, n + 1
      first = first or p
      last = p
    end
  end
  local mx, my = 0, 0
  if n > 0 then mx, my = sx / n, sy / n end
  -- away from the vehicle centre (player-local)
  local vx, vy = c * (f.vx - f.px) - s * (f.vy - f.py), s * (f.vx - f.px) + c * (f.vy - f.py)
  local vl = math.sqrt(vx * vx + vy * vy)
  local ax, ay = 0, -1
  if vl > 1e-3 then ax, ay = -vx / vl, -vy / vl end
  if first ~= nil and last ~= nil then
    local tx, ty = last.x - first.x, last.y - first.y
    local tl = math.sqrt(tx * tx + ty * ty)
    if tl > 1e-3 then
      local nx, ny = -ty / tl, tx / tl
      local side = nx * mx + ny * my
      if math.abs(side) < 0.02 then side = nx * ax + ny * ay end
      if side < 0 then nx, ny = -nx, -ny end
      return nx, ny
    end
    local ml = math.sqrt(mx * mx + my * my)
    if ml > 0.03 then return mx / ml, my / ml end
  end
  return ax, ay
end
function M.PullPathPose(start, w, f, cfg)
  local pos, yaw, pitch, c, s, yawU = pullPathRaw(start, w, f, cfg)
  -- V's capsule is no part of the path (player-local frame: V's axis is the origin); the look direction above stays that of the unpushed path.
  -- A start that is already inside the clearance (a boom that was shorter before the hold) fades the clearance in from its own distance over the first
  -- pullClearFade of the path weight: w = 0 stays the frozen orbit pose, bit for bit.
  -- the push keeps ONE side for the whole pull (pullClearSide) and moves the point along it onto the clearance circle, so it is continuous in w for
  -- every start; the height band has a pullClearRamp m fade outside both edges (pullClearRamp = 0: the hard band).
  local clr = tonumber(cfg.pullClearR) or 0
  local z0, z1 = tonumber(cfg.pullClearZ0) or -0.3, tonumber(cfg.pullClearZ1) or 2.1
  local ramp = tonumber(cfg.pullClearRamp) or 0.25
  local zf = clr > 0 and pullClearZf(pos.z, z0, z1, ramp) or 0
  if zf > 0 then
    local d = math.sqrt(pos.x * pos.x + pos.y * pos.y)
    local eff = clr
    if w < 1 then
      local p0 = pullPathRaw(start, 0, f, cfg)
      local d0 = math.sqrt(p0.x * p0.x + p0.y * p0.y)
      if cfg.pullClearFreeze ~= false then -- the distance at the first frame of the pull, not the live one (V moves toward the frozen start)
        if start.d0f == nil then start.d0f = d0 end
        d0 = start.d0f
      end
      if d0 < clr * pullClearZf(p0.z, z0, z1, ramp) then
        local fade = math.max(tonumber(cfg.pullClearFade) or 0.3, 1e-3)
        local k = math.min(1, w / fade)
        eff = d0 + (clr - d0) * k * k * (3 - 2 * k)
      end
    end
    eff = eff * zf
    if d < eff then
      if tonumber(cfg.pullClearSide) == 0 then
        -- pullClearSide = 0: the radial push (the A/B tests; swings around V when the raw path passes near her axis)
        local ux, uy
        if d > 1e-3 then ux, uy = pos.x / d, pos.y / d
        else
          local vx, vy = c * (f.vx - f.px) - s * (f.vy - f.py), s * (f.vx - f.px) + c * (f.vy - f.py)
          local vl = math.sqrt(vx * vx + vy * vy)
          if vl > 1e-3 then ux, uy = -vx / vl, -vy / vl else ux, uy = 0, -1 end
        end
        pos.x, pos.y = ux * eff, uy * eff
      else
        local nx, ny = pullClearSide(start, f, cfg, clr, z0, z1, ramp, c, s)
        -- the side is kept for the pull: a recomputed side (V's slide into the seat moves the raw path) is taken while it stays on the same half plane
        local key = clr .. "/" .. z0 .. "/" .. z1 .. "/" .. ramp
        local L = start.clearSide
        if L ~= nil and L.key == key and L.x * nx + L.y * ny < 0 then nx, ny = L.x, L.y end
        start.clearSide = { key = key, x = nx, y = ny }
        -- Q = p + n (eff - d) equals p on the circle d = eff (continuous there) and is bounded away from 0 for a side that holds the clipped part of the path
        local qx, qy = pos.x + nx * (eff - d), pos.y + ny * (eff - d)
        local ql = math.sqrt(qx * qx + qy * qy)
        if ql > 1e-6 then pos.x, pos.y = qx / ql * eff, qy / ql * eff else pos.x, pos.y = nx * eff, ny * eff end
      end
    end
  end
  -- the clearance push (and nothing else) can move a cylinder-leg point into the box: the nearest face takes it out again (not during the exit leg
  -- of a start inside the box or before its cylinder path has left the box, and not at the end of the path)
  local rt = start.route
  local pm = tonumber(cfg.hullPushMargin) or 0.05
  if start.hull ~= nil and rt == nil and (start.clampFrom or 0) <= 0 and cfg.hullPushEarly ~= false then
    -- a start outside the box (or less than hullPlanDepth inside: the early pull-out freezes it at the face) is kept out from the first frame on;
    -- the margin grows with the path weight (a start at the face moves only as far as it is inside)
    if w > 0 and w < 1 - 1e-9 then pos = M.PullHullPush(start.hull, pos, f, pm * smooth01(w / 0.1)) end
  elseif start.hull ~= nil and w < 1 - 1e-9 and w > (rt and rt.wE or (start.clampFrom or 0) + 0.03) then
    pos = M.PullHullPush(start.hull, pos, f, pm)
  end
  return pos, yaw, pitch, yawU
end
-- a player-local point inside the box grown by `margin` (vehicle frame, see M.HullFor) moves to the nearest face of the grown box (continuous at its
-- surface: a point that is only just inside moves only just); returns the point (the same table when outside)
function M.PullHullPush(hl, pos, f, margin)
  local rad = math.pi / 180
  local c, s = math.cos(f.pyaw * rad), math.sin(f.pyaw * rad)
  local wx, wy, wz = f.px + c * pos.x - s * pos.y, f.py + s * pos.x + c * pos.y, f.pz + pos.z
  local cv, sv = math.cos(-f.vyaw * rad), math.sin(-f.vyaw * rad)
  local rx, ry = wx - f.vx, wy - f.vy
  local x, y, z = cv * rx - sv * ry, sv * rx + cv * ry, wz - f.vz
  if M.HullDepth(hl, x, y, z, margin) <= 0 then return pos end
  local hx, hyF, hyB, top = hl.hx + margin, hl.hyF + margin, hl.hyB + margin, M.HullTop(hl, y) + margin -- (the top at the point's y)
  local best, kind = hx - math.abs(x), "side"
  if hyF - y < best then best, kind = hyF - y, "front" end
  if hyB + y < best then best, kind = hyB + y, "rear" end
  if top - z < best then best, kind = top - z, "top" end
  if kind == "side" then x = ((x >= 0) and 1 or -1) * hx
  elseif kind == "front" then y = hyF
  elseif kind == "rear" then y = -hyB
  else z = top end
  local cw, sw = math.cos(f.vyaw * rad), math.sin(f.vyaw * rad)
  local ox, oy = f.vx + cw * x - sw * y - f.px, f.vy + sw * x + cw * y - f.py
  local cl, sl = math.cos(-f.pyaw * rad), math.sin(-f.pyaw * rad)
  return { x = cl * ox - sl * oy, y = sl * ox + cl * oy, z = f.vz + z - f.pz }
end
-- pull-out duration for the frozen path: path length (m) and sweep (deg) -> seconds. The eased path peaks at 1.5 x
-- its mean speed / turn rate; the duration is the longest of pullTime (the old fixed value), the time that keeps the peak
-- speed <= pullMaxSpeed and the peak turn rate <= pullMaxTurnRate, capped by pullTimeMax. Returns seconds, length, sweep.
function M.PullPathStats(start, cfg)
  local aT, rT, hT = -math.pi / 2, cfg.pullBack, cfg.pullHeight
  local rt = start.route
  if rt ~= nil then -- exit leg (straight) + the arc from its end
    local l1 = math.sqrt((rt.ex - start.lx) ^ 2 + (rt.ey - start.ly) ^ 2 + (rt.eh - start.h) ^ 2)
    local arc = math.abs(rt.da) * (rt.rE + rT) / 2
    local l2 = math.sqrt(arc * arc + (rT - rt.rE) ^ 2 + (hT - rt.eh) ^ 2)
    return l1 + l2, math.abs(math.deg(rt.da))
  end
  local da = aT - start.a
  while da > math.pi do da = da - 2 * math.pi end
  while da <= -math.pi do da = da + 2 * math.pi end
  local rm = (start.r + rT) / 2
  local arc = math.abs(da) * rm
  local len = math.sqrt(arc * arc + (rT - start.r) ^ 2 + (hT - start.h) ^ 2)
  return len, math.abs(math.deg(da))
end
function M.PullDuration(cfg, start, base)
  base = tonumber(base) or tonumber(cfg.pullTime) or 0.9
  if cfg.pullScale == false or start == nil then return base, 0, 0 end
  local len, sweep = M.PullPathStats(start, cfg)
  if tonumber(start.arcLen) then len = start.arcLen end -- the sampled length of the actual path (M.PullArcTable)
  local t = base
  local vmax, wmax = tonumber(cfg.pullMaxSpeed) or 0, tonumber(cfg.pullMaxTurnRate) or 0
  if vmax > 0 then t = math.max(t, 1.5 * len / vmax) end
  if wmax > 0 then t = math.max(t, 1.5 * sweep / wmax) end
  t = math.min(t, math.max(base, tonumber(cfg.pullTimeMax) or base))
  return t, len, sweep
end
-- exit match fix: frames. f = ctrl.pullFrame { px, py, pz, pyaw, vx, vy, vz, vyaw } (yaws deg CCW). Vehicle frame: x right,
-- y forward, z up from the vehicle origin; player local: the frame of the player entity.
local function rotZ(x, y, deg)
  local r = deg * math.pi / 180
  local c, s = math.cos(r), math.sin(r)
  return c * x - s * y, s * x + c * y
end
function M.WorldToLocal(wx, wy, wz, f)
  local lx, ly = rotZ(wx - f.px, wy - f.py, -f.pyaw)
  return lx, ly, wz - f.pz
end
function M.LocalToWorld(lx, ly, lz, f)
  local wx, wy = rotZ(lx, ly, f.pyaw)
  return f.px + wx, f.py + wy, f.pz + lz
end

-- Target pose of the pull-out in the player's local frame (positions m, yaws deg CCW, forward = (-sin, cos)).
-- Returns pos {x,y,z} and the camera yaw / pitch in degrees (local). Pure math (offline tested).
function M.PullPose(px, py, pz, pyaw, vx, vy, vz, vyaw, cfg)
  local rad = math.pi / 180
  local fx, fy = -math.sin(vyaw * rad), math.cos(vyaw * rad)
  local cx, cy, cz = vx - fx * cfg.pullBack, vy - fy * cfg.pullBack, vz + cfg.pullHeight
  local lx, ly, lz = vx, vy, vz + cfg.pullLookHeight
  local dx, dy, dz = lx - cx, ly - cy, lz - cz
  local camYaw = math.atan2(-dx, dy) / rad
  local camPitch = math.atan2(dz, math.sqrt(dx * dx + dy * dy)) / rad * cfg.pullPitchSign
  local c, s = math.cos(-pyaw * rad), math.sin(-pyaw * rad)
  local wx, wy = cx - px, cy - py
  return { x = c * wx - s * wy, y = s * wx + c * wy, z = cz - pz }, camYaw - pyaw, camPitch
end
-- Vehicle path: world pose for utppCamera this frame (utpp_camera.SetPoseOverride) - exit hold (vehicle-behind
-- pose for the current player pose, matched game camera pose on top with ctrl.MatchWeight()), mount pull-out (path
-- around the vehicle from the orbit pose frozen at the pull start, orientation from the orbit yaw / pitch frozen at the
-- same frame to the look-at, w = smoothstep(pull); handover match on top) - or nil (the camera's own orbit / release
-- blend; nothing while handed over).
-- f = ctrl.pullFrame, orbit = utpp_camera.OrbitPose() { x, y, z, yaw, pitch } (world). Returns { pos, yaw, pitch }.
local function lerpAngle(a, b, w)
  local d = (b - a) % 360
  if d > 180 then d = d - 360 end
  return a + d * w
end
local function vehToWorld(x, y, z, f)
  local wx, wy = rotZ(x, y, f.vyaw)
  return f.vx + wx, f.vy + wy, f.vz + z
end
local function worldToVeh(x, y, z, f)
  local lx, ly = rotZ(x - f.vx, y - f.vy, -f.vyaw)
  return lx, ly, z - f.vz
end
local function poseLerp(a, bx, by, bz, byaw, bpitch, w)
  return { x = a.x + (bx - a.x) * w, y = a.y + (by - a.y) * w, z = a.z + (bz - a.z) * w, yaw = lerpAngle(a.yaw, byaw, w),
           pitch = a.pitch + (bpitch - a.pitch) * w }
end
-- frame stall detector (also in utpp_camera): the time a frame counts for the blends is min(dt, max(base, 2 x the running mean frame time)); a steady
-- slow game is not slowed down, a spike is cut to about two normal frames. h.dtEma = mean of the counted times (seeded with the first). base <= 0: dt as is.
function M.BlendDt(h, dt, base)
  dt = dt or 0
  if not (base and base > 0) or dt <= 0 then return dt end
  local ema = h.dtEma
  local cap = math.max(base, 2 * (ema or 0))
  local eff = dt > cap and cap or dt
  h.dtEma = ema and (ema + (eff - ema) * 0.1) or eff
  return eff
end
-- arc length of the actual pull-out path (M.PullPathPose: clearance, exit leg, box push) in the vehicle frame for the frame f (V where she is
-- at the freeze), n + 1 samples of the path weight. Returns { cum = cumulative lengths (cum[1] = 0), n } and the total length. The sampling leaves the
-- per-pull state of the path (frozen clearance distance and side) as it found it.
function M.PullArcTable(start, f, cfg, n)
  n = math.max(4, math.floor(tonumber(n) or 64))
  local d0f, side = start.d0f, start.clearSide
  local cum, px, py, pz = { 0 }, nil, nil, nil
  for i = 0, n do
    local p = M.PullPathPose(start, i / n, f, cfg)
    local wx, wy, wz = M.LocalToWorld(p.x, p.y, p.z, f)
    local x, y, z = worldToVeh(wx, wy, wz, f)
    if i > 0 then cum[i + 1] = cum[i] + math.sqrt((x - px) ^ 2 + (y - py) ^ 2 + (z - pz) ^ 2) end
    px, py, pz = x, y, z
  end
  start.d0f, start.clearSide = d0f, side
  return { cum = cum, n = n }, cum[n + 1]
end
-- the path weight at which the fraction s (0..1) of the arc length is reached (linear between the samples; s outside 0..1 is clamped)
function M.PullArcW(arc, s)
  local cum, n = arc.cum, arc.n
  local L = cum[n + 1]
  if not (L and L > 1e-6) then return s end
  if s <= 0 then return 0 end
  if s >= 1 then return 1 end
  local target = s * L
  local lo, hi = 1, n + 1
  while hi - lo > 1 do
    local mid = math.floor((lo + hi) / 2)
    if cum[mid] < target then lo = mid else hi = mid end
  end
  local seg = cum[hi] - cum[lo]
  local u = seg > 1e-9 and (target - cum[lo]) / seg or 0
  return (lo - 1 + u) / n
end
function M.OwnPose(ctrl, f, orbit)
  if f == nil then return nil end
  local cfg = ctrl.cfg
  local pcfg = pullCfgFor(cfg, ctrl.vehBike) -- the bike pull-out / vehicle-behind target
  local sign = cfg.pullPitchSign or 1
  local out = nil
  local fovOut = nil
  if ctrl.exitPull == "hold" then
    local lp, lyaw, lpitch = M.PullPose(f.px, f.py, f.pz, f.pyaw, f.vx, f.vy, f.vz, f.vyaw, pcfg)
    local wx, wy, wz = M.LocalToWorld(lp.x, lp.y, lp.z, f)
    out = { x = wx, y = wy, z = wz, yaw = lyaw + f.pyaw, pitch = lpitch }
    local m = ctrl.match
    local mw = (m and ctrl.MatchWeight) and ctrl.MatchWeight() or 0
    if m and mw > 0 then
      local mx, my, mz = vehToWorld(m.x, m.y, m.z, f)
      out = poseLerp(out, mx, my, mz, f.vyaw + m.yaw, m.pitch * sign, mw)
      -- (3): the matched camera's FOV blended to the profile FOV with the match weight
      if m.fov and orbit and tonumber(orbit.fov) then fovOut = orbit.fov + (m.fov - orbit.fov) * mw end
    end
    -- jump-out follow: hold target -> orbit pose around V (orbit yaw, pitch of the hold kept) over exitFollow.dur. The
    -- offset of the hold pose from that orbit pose is frozen at the start and shrinks to 0: the camera moves with V at
    -- once (V keeps the vehicle's momentum) instead of waiting at the vehicle while the gap grows.
    local ef = ctrl.exitFollow
    if ef then ef.fol = nil end -- a frame where the block below does not run (no orbit) must not trace the previous frame's branch (the trace also prints only a value stamped with this frame's clock, ef.folAt)
    if ef and orbit then
      -- the follow keeps V framed: hold pitch clamped to [exitFollowPitchMin, exitFollowPitchMax]
      if ef.pitch == nil then
        ef.pitch = math.max(tonumber(cfg.exitFollowPitchMin) or -20, math.min(tonumber(cfg.exitFollowPitchMax) or 5, out.pitch))
      end
      local tgt = nil
      if type(ctrl.orbitAt) == "function" then
        local okT, pT = pcall(ctrl.orbitAt, orbit.yaw, ef.pitch)
        if okT and type(pT) == "table" then tgt = pT end
      end
      tgt = tgt or { x = orbit.x, y = orbit.y, z = orbit.z, yaw = orbit.yaw }
      if ef.off == nil then
        local dy = (out.yaw - tgt.yaw) % 360
        if dy > 180 then dy = dy - 360 end
        ef.off = { x = out.x - tgt.x, y = out.y - tgt.y, z = out.z - tgt.z, yaw = dy }
        if ctrl.Log then
          ctrl.Log(string.format("exit follow: start offset %.2f m, %.0f deg from the orbit around V", math.sqrt(ef.off.x ^ 2
            + ef.off.y ^ 2 + ef.off.z ^ 2), dy))
        end
      end
      -- standing car: the start (the hold pose) and the target (the orbit around V) as vehicle-frame cylinders
      -- (angle, radius, height); the duration follows the path, the camera goes around the body (never across the cab)
      if ef.around and ef.cyl == nil then
        local sx, sy, sz = worldToVeh(out.x, out.y, out.z, f)
        local tx, ty, tz = worldToVeh(tgt.x, tgt.y, tgt.z, f)
        local s0 = { a = math.atan2(sy, sx), r = math.sqrt(sx * sx + sy * sy), h = sz }
        local t0 = { a = math.atan2(ty, tx), r = math.sqrt(tx * tx + ty * ty), h = tz }
        local da = t0.a - s0.a
        while da > math.pi do da = da - 2 * math.pi end
        while da <= -math.pi do da = da + 2 * math.pi end
        ef.cyl = { s = s0, dir = da >= 0 and 1 or -1 }
        local arc = math.abs(da) * (s0.r + t0.r) / 2
        local len = math.sqrt(arc * arc + (t0.r - s0.r) ^ 2 + (t0.h - s0.h) ^ 2)
        local spd = math.max(0.5, tonumber(cfg.exitFollowStandSpeed) or 5)
        -- the follow of a jump-out whose car is at rest runs under utpp_camera's exit pose governor (5 m/s, armed at the exit take):
        -- the eased path peaks at 1.5 x its mean; at 4.5 m/s the governed pose still lagged 1 - 2 m behind it when the hold ended (the release then ran
        -- straight through the rear corner of the box again, 3.0 m/s: 2.5 m/s of carried lag); exitFollowRestMean m/s keeps the peak below the governor
        if ef.rest then spd = math.min(spd, math.max(0.5, tonumber(cfg.exitFollowRestMean) or 2.5)) end
        local lo, hi = tonumber(cfg.exitFollowBlend) or 0.25, tonumber(cfg.exitFollowStandMax) or 1.6
        ef.dur = math.max(lo, math.min(hi, len / spd))
        ef.len = len
        if ctrl.ExtendFollowHold then ctrl.ExtendFollowHold() end
        if ctrl.Log then
          ctrl.Log(string.format("exit follow: around the vehicle, path %.1f m (%.0f deg) in %.2f s (mean %.1f m/s)", len,
            math.abs(math.deg(da)), ef.dur, len / ef.dur))
        end
      end
      -- a stalled frame (longer than blendDtCap) is not counted in the follow blend (ef.lost)
      local capDt, fdt = tonumber(cfg.blendDtCap) or 0, (ctrl.clock or 0) - (ctrl.ownLastAt or (ctrl.clock or 0))
      local capEff = math.max(capDt, 2 * (ctrl.dtEma or 0))
      -- a last shown pose older than exitFollowStaleS is not the previous frame (left over from before the exit): no stall accounting
      local staleS = tonumber(cfg.exitFollowStaleS) or 0.25
      if fdt > staleS then fdt = 0 end
      if capDt > 0 and fdt > capEff and ef.lostAt ~= ctrl.clock then ef.lost, ef.lostAt = (ef.lost or 0) + fdt - capEff, ctrl.clock end
      local k = 1 - smooth01(((ctrl.clock or 0) - ef.at - (ef.lost or 0)) / ef.dur)
      local o = ef.off
      out = { x = tgt.x + o.x * k, y = tgt.y + o.y * k, z = tgt.z + o.z * k, yaw = tgt.yaw + o.yaw * k, pitch = ef.pitch }
      if ef.around and ef.cyl then
        local tx, ty, tz = worldToVeh(tgt.x, tgt.y, tgt.z, f)
        local ta, tr = math.atan2(ty, tx), math.sqrt(tx * tx + ty * ty)
        local s0, w = ef.cyl.s, 1 - k
        local da = ta - s0.a
        while da > math.pi do da = da - 2 * math.pi end
        while da <= -math.pi do da = da + 2 * math.pi end
        -- keep the arc on the side chosen at the start (V's orbit target moves slowly; a flip at +-180 deg would cross the body)
        if ef.cyl.dir > 0 and da < -math.pi * 0.9 then da = da + 2 * math.pi
        elseif ef.cyl.dir < 0 and da > math.pi * 0.9 then da = da - 2 * math.pi end
        -- (a shrinking radius is delayed (w^2): the corner of the body is passed at the start radius, not closer)
        local wr = (tr < s0.r) and w * w or w
        local a, r, h = s0.a + da * w, s0.r + (tr - s0.r) * wr, s0.h + (tz - s0.h) * w
        out.x, out.y, out.z = vehToWorld(r * math.cos(a), r * math.sin(a), h, f)
      end
      -- (4): caps of the follow output against the last shown pose (exitFollowMaxRate deg/s, exitFollowMaxStep m
      -- per frame). The position step is measured relative to V (the camera keeps up with a flying V; only the change
      -- of the framing is capped).
      local last, dt = ctrl.ownLast, (ctrl.clock or 0) - (ctrl.ownLastAt or -1)
      local lp = ctrl.ownLastP
      if last and lp and dt > 0 and (dt < 0.1 or (capDt > 0 and dt <= staleS)) then
        local dtc = math.min(dt, 0.1) -- (a stall of 0.1 s and more used to skip the caps below: the follow stepped to the wanted pose at once)
        local ox, oy, oz = last.x - lp.x, last.y - lp.y, last.z - lp.z
        local dx, dy, dz = (out.x - f.px) - ox, (out.y - f.py) - oy, (out.z - f.pz) - oz
        local d = math.sqrt(dx * dx + dy * dy + dz * dz)
        local capSpd, capRate = tonumber(cfg.exitFollowMaxSpeed) or 15, tonumber(cfg.exitFollowMaxRate) or 180
        if ctrl.exitJump then -- a jump-out follow has its own, lower caps (0 = the common ones)
          local js, jr = tonumber(cfg.exitJumpMaxSpeed) or 0, tonumber(cfg.exitJumpMaxRate) or 0
          if js > 0 then capSpd = math.min(capSpd, js) end
          if jr > 0 then capRate = math.min(capRate, jr) end
        end
        local maxS = math.min(tonumber(cfg.exitFollowMaxStep) or 0.35, capSpd * dtc)
        if d > maxS then
          local s = maxS / d
          out.x, out.y, out.z = f.px + ox + dx * s, f.py + oy + dy * s, f.pz + oz + dz * s
          ef.capped = (ef.capped or 0) + 1
        end
        local maxA = capRate * dtc
        local dyaw = wrap180(out.yaw - last.yaw)
        if math.abs(dyaw) > maxA then out.yaw = last.yaw + (dyaw > 0 and maxA or -maxA); ef.capped = (ef.capped or 0) + 1 end
        local dp = out.pitch - last.pitch
        if math.abs(dp) > maxA then out.pitch = last.pitch + (dp > 0 and maxA or -maxA); ef.capped = (ef.capped or 0) + 1 end
      end
      -- launch follow only - the shown height is an acceleration limited follower of the wanted one (the orbit around V
      -- plus the collapsing offset): speed command = the speed of the orbit around V + the gap / exitFollowVzTau, the gap closing
      -- limited to exitFollowMaxVz and to the speed amax still stops (no overshoot), acceleration limited to exitFollowMaxAz
      local vmax, amax = tonumber(cfg.exitFollowMaxVz) or 0, tonumber(cfg.exitFollowMaxAz) or 0
      ef.fol = nil
      if vmax > 0 and amax > 0 and last and dt > 0 and dt < 0.1 and not ctrl.exitJump then
        ef.az = nil -- (the acceleration limiter of a later arming starts from rest)
        ef.vz, ef.tz = (out.z - last.z) / dt, tgt.z -- not (yet) a jump-out: the shown height stays as it is; the speed is what a later arming continues from
        ef.folAt = ctrl.clock; ef.fol = string.format("raw/%.1f/%.0fms", ef.vz, dt * 1000) -- trace: which branch shaped the height of this frame (raw = not armed), the speed, the frame time
      elseif vmax > 0 and amax > 0 and ctrl.exitJump then
        if last and dt > 0 and (dt < 0.1 or (capDt > 0 and dt <= staleS)) then
          local z1, z0 = out.z, last.z
          -- speed of the orbit around V over the real frame time (the offset collapse is closed with the limited gap speed)
          local vT = ef.tz and (tgt.z - ef.tz) / dt or 0
          local frameDt = dt -- the real frame time (the fol= trace prints this one, not the clamped value below)
          local dt = math.min(dt, 0.1) -- a stalled frame (0.1 s and more) is integrated as 0.1 s instead of dropping the follower's speed
          local gap = z1 - z0
          local close = math.max(-vmax, math.min(vmax, gap / math.max(tonumber(cfg.exitFollowVzTau) or 0.1, 1e-3)))
          local vstop = math.sqrt(0.5 * amax * math.abs(gap)) -- brakes with amax / 4 (the wanted height decelerates as well)
          close = math.max(-vstop, math.min(vstop, close))
          local v0 = ef.vz or 0
          local dv = vT + close - v0
          local v1
          local jz = tonumber(cfg.exitFollowMaxJz) or 0
          if jz > 0 then
            -- the acceleration itself changes at the jerk limit (it starts from the acceleration of the last frame, 0 at the first one)
            local a0 = ef.az or 0
            local a1 = math.max(a0 - jz * dt, math.min(a0 + jz * dt, dv / dt))
            a1 = math.max(-amax, math.min(amax, a1))
            v1, ef.az = v0 + a1 * dt, a1
          else
            v1 = v0 + math.max(-amax * dt, math.min(amax * dt, dv))
          end
          local z = z0 + v1 * dt
          if (z - z1) * gap > 0 then z = z1 end -- never beyond the wanted height
          out.z, ef.vz = z, v1
          ef.tz = tgt.z
          ef.folAt = ctrl.clock; ef.fol = string.format("lim/%.1f/%.0fms", v1, frameDt * 1000) -- (the real frame time, not min(dt, 0.1))
        else
          ef.vz, ef.tz, ef.az = nil, nil, nil
          ef.folAt = ctrl.clock; ef.fol = string.format("free/%.0fms", dt * 1000) -- trace: the limiter was skipped (no last pose, or a frame of 0.1 s and more)
        end
      end
    end
  elseif ctrl.exitPull == nil and ctrl.pulling and (ctrl.pull or 0) > 0
         and (ctrl.keep == "mount" or ctrl.keep == "chase" or (ctrl.Releasing and ctrl.Releasing())) then -- chase
    local w = smooth01(ctrl.pull)
    -- the start of the pull-out is frozen once - the position (vehicle
    -- frame cylinder) and the orbit yaw / pitch (yaw relative to the vehicle). The orientation used to lerp from the LIVE
    -- orbit yaw, which follows V's body while the override is on; the mount slide spins V +67 deg and back, so the shown
    -- camera whipped with it (camTrace out -8.4 -> 12.1 -> 20.2 -> -10.8). Frozen in pathStart: every reset of the path
    -- (pathStart = nil) also drops the frozen orientation. At w = 0 the output is the orbit pose of the freeze frame
    -- (no jump at the pull start); the live orbit is not read again until the next freeze.
    if ctrl.pathStart == nil then
      if orbit == nil then return nil end
      local lx, ly, lz = M.WorldToLocal(orbit.x, orbit.y, orbit.z, f)
      ctrl.pathStart = M.VehCyl(lx, ly, lz, f)
      ctrl.pathStart.lx, ctrl.pathStart.ly, ctrl.pathStart.lz = worldToVeh(orbit.x, orbit.y, orbit.z, f)
      ctrl.pathStart.oyawV = wrap180((tonumber(orbit.yaw) or 0) - f.vyaw)
      ctrl.pathStart.opitch = tonumber(orbit.pitch) or 0
      if cfg.pullHull ~= false and ctrl.IsCar and ctrl.IsCar() then -- the keep-out box of the car (a start inside it leaves through V's side)
        local hull = M.HullFor(cfg, ctrl.vehRecKey)
        local vx0, vy0 = worldToVeh(f.px, f.py, f.pz, f)
        M.PullPlan(ctrl.pathStart, pcfg, hull, vx0, vy0)
        if ctrl.Log then
          -- (inside / outside from the depth, not from the plan: a start inside whose cylinder path leaves the box first read "outside")
          local d0 = M.HullDepth(hull, ctrl.pathStart.lx, ctrl.pathStart.ly, ctrl.pathStart.lz)
          ctrl.Log(string.format("pull-out path: box hx %.2f hy %.2f/%.2f top %.2f (%s), start %s the box (depth %.2f m, < 0 = outside)%s%s", hull.hx, hull.hyF,
            hull.hyB, hull.top, tostring(ctrl.vehRecKey or "default"), d0 > (tonumber(pcfg.hullPlanDepth) or 0.05) and "inside" or "outside", d0,
            ctrl.pathStart.route and string.format(", exit leg to (%.2f,%.2f,%.2f) over %.0f%% of the path (outside at w %.2f, the cylinder path at %.2f)",
              ctrl.pathStart.route.ex, ctrl.pathStart.route.ey, ctrl.pathStart.route.eh, ctrl.pathStart.route.wE * 100,
              ctrl.pathStart.exitRoute or 0, ctrl.pathStart.exitCyl or 0)
              or (ctrl.pathStart.exitCyl and string.format(", cylinder path (outside at w %.2f)", ctrl.pathStart.clampFrom or 0) or ""),
            hull.topY and string.format(", top profile %d knots (%.2f at the start)", #hull.topY, M.HullTop(hull, ctrl.pathStart.ly)) or ""))
        end
      end
      if ctrl.Log then
        ctrl.Log(string.format("pull-out path: start frozen at %.0f deg, %.2f m from the vehicle, %.2f m up (utppCamera orbit)"
          .. ", orbit yaw %.1f pitch %.1f frozen", math.deg(ctrl.pathStart.a), ctrl.pathStart.r, ctrl.pathStart.h,
          tonumber(orbit.yaw) or 0, ctrl.pathStart.opitch))
      end
      if ctrl.IsCar and ctrl.IsCar() then -- the duration follows the frozen path (positively identified cars only)
        if cfg.pullPath and cfg.pullArcLength ~= false and ctrl.pathStart.route == nil and (ctrl.pathStart.clampFrom or 0) <= 0 then -- the eased weight runs along the arc length of the actual path
          local arc, L = M.PullArcTable(ctrl.pathStart, f, pcfg, cfg.pullArcSamples)
          if L > 1e-3 then ctrl.pathStart.arc, ctrl.pathStart.arcLen = arc, L end
        end
        local T, len, sweep = M.PullDuration(pcfg, ctrl.pathStart, cfg.pullTime)
        ctrl.pathStart.dur, ctrl.pathStart.len, ctrl.pathStart.sweep = T, len, sweep
        if ctrl.Log then
          ctrl.Log(string.format("pull-out path: %.1f m, %.0f deg around the vehicle -> %.2f s (peak %.1f m/s, %.0f deg/s)", len,
            sweep, T, 1.5 * len / math.max(T, 0.05), 1.5 * sweep / math.max(T, 0.05)))
        end
      end
    end
    local ps = ctrl.pathStart
    if ps.arc ~= nil then w = M.PullArcW(ps.arc, w) end -- eased along the arc length (the yaw / pitch blend below uses the same weight)
    local tp, tyaw, tpitch, tyU
    if cfg.pullPath then
      tp, tyaw, tpitch, tyU = M.PullPathPose(ps, w, f, pcfg)
    else
      local pp, pyaw, ppitch = M.PullPose(f.px, f.py, f.pz, f.pyaw, f.vx, f.vy, f.vz, f.vyaw, pcfg)
      local sx, sy, sz = vehToWorld(ps.lx, ps.ly, ps.lz, f)
      local ox, oy, oz = M.WorldToLocal(sx, sy, sz, f)
      tp, tyaw, tpitch = { x = ox + (pp.x - ox) * w, y = oy + (pp.y - oy) * w, z = oz + (pp.z - oz) * w }, pyaw, ppitch
    end
    local wx, wy, wz = M.LocalToWorld(tp.x, tp.y, tp.z, f)
    local ty = tyaw + f.pyaw
    local oyaw, opitch = f.vyaw + ps.oyawV, ps.opitch
    local yawOut
    if tyU ~= nil then
      -- the look-at yaw is continuous along the path (tyU) and its branch is the one that ends within 180 deg of the frozen yaw: the
      -- shortest arc is taken once (at the end), not every frame - a per-frame shortest arc flipped by 360 deg x w when the look-at yaw passed 180 deg
      -- from the frozen yaw (168 deg at w = 0.5 in a 'behind' mount of the truck)
      local k = math.floor((oyaw - M.PullEndYaw(ps, f)) / 360 + 0.5)
      yawOut = oyaw + (tyU + 360 * k - oyaw) * w
    else
      yawOut = lerpAngle(oyaw, ty, w)
    end
    out = { x = wx, y = wy, z = wz, yaw = yawOut, pitch = opitch + (tpitch - opitch) * w }
    -- utpp_camera: the hidden orbit keeps its world yaw under this override during the mount (no follow of the body)
    out.keepYaw = ctrl.keep == "mount" or nil
    local hm = ctrl.handMatch
    if hm then
      local hw = smooth01(((ctrl.clock or 0) - hm.at) / math.max(hm.dur, 0.05))
      local hx, hy, hz = vehToWorld(hm.x, hm.y, hm.z, f)
      out = poseLerp(out, hx, hy, hz, f.vyaw + hm.yaw, hm.pitch * sign, hw)
      out.keepYaw = ctrl.keep == "mount" or nil
      -- car entry glide: the target camera's FOV by the same weight (the pull-out shows the profile FOV)
      if hm.fov and orbit and tonumber(orbit.fov) then fovOut = orbit.fov + (hm.fov - orbit.fov) * hw end
    end
  end
  -- the orbit camera of a mount inside the keep-out box (it follows V into the cabin) asks UpdatePull for an early pull-out (pullEscape)
  if ctrl.exitPull == nil and not ctrl.pulling and ctrl.keep == "mount" and orbit and cfg.pullHull ~= false and cfg.pullEscape ~= false
     and ctrl.IsCar and ctrl.IsCar() then
    local ox, oy, oz = worldToVeh(orbit.x, orbit.y, orbit.z, f)
    local d = M.HullDepth(M.HullFor(cfg, ctrl.vehRecKey), ox, oy, oz)
    ctrl.camInBox = d > (tonumber(cfg.pullEscapeDepth) or 0.1) and d or nil
    -- the approach of the orbit camera per frame (previous call of this mount hold, at most 0.1 s ago) extrapolated pullEscapeLead frames (the flag
    -- is read by the next UpdatePull, the frozen pose is the orbit of that frame and the override shows from then on)
    local lead, prev, now = tonumber(cfg.pullEscapeLead) or 0, ctrl.camBoxPrev, ctrl.clock or 0
    if prev == nil or now ~= prev.at then -- (a second call in the same frame keeps the result of the first)
      ctrl.camBoxNear = nil
      if lead > 0 and prev ~= nil and now > prev.at and now - prev.at <= 0.1 then
        local step = d - prev.d
        local pred = d + lead * step
        if step > 1e-4 and pred > -(tonumber(cfg.pullEscapeAhead) or 0.03) then ctrl.camBoxNear = { d = d, pred = pred } end
      end
      ctrl.camBoxPrev = { d = d, at = now }
    end
  else
    ctrl.camInBox, ctrl.camBoxNear, ctrl.camBoxPrev = nil, nil, nil
  end
  if out then
    out.fov = fovOut -- the shown FOV, for an exit before the handover / during the hand-over blend (ShownVehPose)
    ctrl.ownLast, ctrl.ownLastAt, ctrl.ownLastP = out, ctrl.clock, { x = f.px, y = f.py, z = f.pz }
  elseif orbit and ctrl.keep == "mount" then
    ctrl.ownLast, ctrl.ownLastAt = { x = orbit.x, y = orbit.y, z = orbit.z, yaw = orbit.yaw, pitch = orbit.pitch }, ctrl.clock
  end
  if out == nil then return nil end
  return { pos = { x = out.x, y = out.y, z = out.z }, yaw = out.yaw, pitch = out.pitch, fov = fovOut, keepYaw = out.keepYaw }
end

-- Vehicle path: executes the controller's commands through utpp_camera's vehicle API (cam = GetMod
-- ("utpp_camera")); pose = M.OwnPose of this frame (needed by "take"). Each outcome is logged. Returns the count.
function M.RunOwnCmds(ctrl, cam, pose)
  local cmds = ctrl.TakeCamCmds()
  local log = ctrl.Log or function() end
  local function call(name, ...)
    if type(cam) ~= "table" then return false, "utpp_camera not found" end
    local fn = cam[name]
    if type(fn) ~= "function" then return false, name .. " unavailable" end
    local ok, r, e = pcall(fn, ...)
    if not ok then return false, tostring(r) end
    if r == false then return false, tostring(e or "refused") end
    return true
  end
  for _, cmd in ipairs(cmds) do
    local res = {}
    local function r(label, ok, err) res[#res + 1] = label .. "=" .. (ok and "ok" or ("FAILED " .. tostring(err))) end
    local why = "utpp_vehiclecam: " .. tostring(cmd.why)
    if cmd.op == "vhold" then
      local ok, err = call("HoldForVehicle", true, why)
      r("HoldForVehicle(true)", ok, err)
      if ctrl.OnOwnHold then ctrl.OnOwnHold(ok, err) end
    elseif cmd.op == "hold" then
      r("SetVehiclePriority(true)", call("SetVehiclePriority", true, why))
    elseif cmd.op == "unhold" then
      r("SetVehiclePriority(false)", call("SetVehiclePriority", false, why))
    elseif cmd.op == "release" or cmd.op == "drop" then
      r(string.format("HandToVehicle(%.2f)", cmd.blend or 0), call("HandToVehicle", cmd.blend or 0, why))
    elseif cmd.op == "take" then
      if pose then
        r("SetPoseOverride", call("SetPoseOverride", pose.pos, pose.yaw, pose.pitch, why, pose.fov, pose.keepYaw))
        res[#res] = res[#res] .. (ctrl.match and " (matched " .. tostring(ctrl.match.src) .. ")" or " (vehicle-behind)")
      else
        res[#res + 1] = "pose=none"
      end
      r(string.format("TakeFromVehicle(%.2f)", cmd.blend or 0), call("TakeFromVehicle", cmd.blend or 0, why))
    elseif cmd.op == "orbit" then
      r(string.format("ReleaseToOrbit(%.2f%s)", cmd.blend or 0, cmd.carry and ", carry" or ""),
        call("ReleaseToOrbit", cmd.blend or 0, why, cmd.carry == true))
    elseif cmd.op == "jumpout" then
      -- optional API (not in OWN_API): an older utpp_camera reports "VehicleJumpOut unavailable"
      r("VehicleJumpOut(true)", call("VehicleJumpOut", true, why))
    elseif cmd.op == "foot" then
      -- optional API (not in OWN_API): an older utpp_camera reports "VehicleOnFoot unavailable"
      r("VehicleOnFoot(true)", call("VehicleOnFoot", true, why))
    elseif cmd.op == "free" then
      r("HoldForVehicle(false)", call("HoldForVehicle", false, why))
    elseif cmd.op == "togame" then
      -- the blend is passed only to a camera with vehicle API >= 2 (older cameras: the same call as before)
      local b = (ctrl.ownApiMotion and cmd.blend ~= nil) and cmd.blend or nil
      r(b and string.format("HoldForVehicle(false, game, %.2f)", b) or "HoldForVehicle(false, game)",
        call("HoldForVehicle", false, why, "game", b))
    elseif cmd.op == "gametake" then
      -- exit take-over from the game camera (utpp_camera vehicle API >= 2)
      local ok, err = false, "pose=none"
      if pose then ok, err = call("TakeForVehicleExit", pose.pos, pose.yaw, pose.pitch, cmd.blend or 0, why, pose.fov) end
      r(string.format("TakeForVehicleExit(%.2f)", cmd.blend or 0), ok, err)
      if pose then res[#res] = res[#res] .. (ctrl.match and " (matched " .. tostring(ctrl.match.src) .. ")" or " (vehicle-behind)") end
      if ctrl.OnGameTake then ctrl.OnGameTake(ok, err) end
    else
      res[#res + 1] = "unknown command"
    end
    ctrl.stats.camCmdsRun = (ctrl.stats.camCmdsRun or 0) + 1
    log(string.format("utppCamera %s (%s): %s", cmd.op, tostring(cmd.why), table.concat(res, ", ")))
  end
  return #cmds
end

local ctrl = M.NewController()
M.ctrl = ctrl
local wiring = { patched = false, err = nil, readErrors = 0, requestErrors = 0 }
-- CET may return the active camera Transform as either return value, or write an out parameter.
local function activeCameraTransform(cs)
  local ok, a, b = pcall(function() return cs:GetActiveCameraWorldTransform() end)
  local function valid(tr)
    if tr == nil or type(tr) == "boolean" then return false end
    local okP, pos = pcall(function() return tr.position or (tr.GetPosition and tr:GetPosition()) end)
    return okP and pos ~= nil
  end
  if ok then
    if valid(b) then return b end
    if valid(a) then return a end
  end
  local okArg, tr = pcall(function()
    local out = Transform.new()
    local accepted = cs:GetActiveCameraWorldTransform(out)
    if accepted == false then return nil end
    return out
  end)
  if okArg and valid(tr) then return tr end
  return nil
end
M.wiring = wiring

function M.SetEnabled(on) ctrl.cfg.enabled = on and true or false; return ctrl.cfg.enabled end
function M.SetPerspective(p) ctrl.cfg.perspective = tonumber(p) or -1; return ctrl.cfg.perspective end
function M.SetBlendTime(s) ctrl.cfg.blendTime = tonumber(s); return ctrl.cfg.blendTime end -- nil = do not touch
function M.SetWaitDrive(on) ctrl.cfg.waitDrive = on and true or false; return ctrl.cfg.waitDrive end
function M.SetTrace(on) ctrl.trace = on and print or nil; return on and true or false end
-- exit fix A/B switch (not persisted): camera behind the vehicle during the exit
function M.SetExitPull(on) ctrl.cfg.exitPull = on and true or false; return ctrl.cfg.exitPull end
-- mount handover fix A/B (not persisted): false = the old fixed keepHandover (0.25 s); SetHandover(s) = that minimum
function M.SetHandoverBlend(on) ctrl.cfg.keepHandoverBlend = on and true or false; return ctrl.cfg.keepHandoverBlend end
function M.SetHandover(s) ctrl.cfg.keepHandover = tonumber(s) or M.DEFAULTS.keepHandover; return ctrl.cfg.keepHandover end
-- mount jump fix A/B (not persisted): false = the old behaviour of each part
function M.SetPullPath(on) ctrl.cfg.pullPath = on and true or false; return ctrl.cfg.pullPath end
function M.SetPriorityHold(on) ctrl.cfg.priorityHold = on and true or false; return ctrl.cfg.priorityHold end
function M.SetHoldRelease(s) ctrl.cfg.holdRelease = tonumber(s) or M.DEFAULTS.holdRelease; return ctrl.cfg.holdRelease end
function M.SetCamTrace(on) ctrl.cfg.camTrace = on and true or false; return ctrl.cfg.camTrace end
-- exit match fix A/B (not persisted): false = the fixed vehicle-behind pose
function M.SetExitMatch(on) ctrl.cfg.exitMatch = on and true or false; return ctrl.cfg.exitMatch end
function M.SetExitMatchTime(s)
  ctrl.cfg.exitMatchTime = tonumber(s) or M.DEFAULTS.exitMatchTime
  ctrl.cfg.exitMatchScale = tonumber(s) == nil -- a given duration replaces the distance scaling of cars (A/B)
  return ctrl.cfg.exitMatchTime
end
-- car fixes A/B (not persisted): the entry glide, the matched exit take blend, the distance scaled exit match
function M.SetHandGlide(on) ctrl.cfg.handGlide = on and true or false; return ctrl.cfg.handGlide end
function M.SetExitTakeBlendMatched(s) ctrl.cfg.exitTakeBlendMatched = tonumber(s) or M.DEFAULTS.exitTakeBlendMatched; return ctrl.cfg.exitTakeBlendMatched end
function M.SetExitMatchScale(on) ctrl.cfg.exitMatchScale = on and true or false; return ctrl.cfg.exitMatchScale end
function M.SetPassengerChase(on) ctrl.cfg.passengerChase = on and true or false; return ctrl.cfg.passengerChase end
function M.SetForceTppPassenger(on) ctrl.cfg.forceTppPassenger = on and true or false; return ctrl.cfg.forceTppPassenger end
function M.SetOwnHandoverMatch(on) ctrl.cfg.ownHandoverMatch = on and true or false; return ctrl.cfg.ownHandoverMatch end
-- jump-out follow A/B (not persisted): false = the vehicle-behind hold until the unmount + exitHoldAfterUnmount (before)
function M.SetExitFollow(on) ctrl.cfg.exitFollow = on and true or false; return ctrl.cfg.exitFollow end
-- ride watchdog (setting vehicle.rideWatchdog, default on) and the hand-over gate (A/B, default off)
function M.SetRideWatchdog(on) ctrl.cfg.rideWatchdog = on and true or false; return ctrl.cfg.rideWatchdog end
-- jump-out side push (setting vehicle.jumpOutSidePush, default on): takes effect at the next jump-out launch
function M.SetJumpOutSidePush(on) ctrl.cfg.jumpOutSidePush = on and true or false; return ctrl.cfg.jumpOutSidePush end
function M.SetOwnHandoverGate(on) ctrl.cfg.ownHandoverGate = on and true or false; return ctrl.cfg.ownHandoverGate end
-- A/B switches (not persisted): exit keep of the vehicle TPP perspective, FOV match
function M.SetExitKeepVehicleTpp(on) -- bikes / unknown class (A) and cars (default off there)
  ctrl.cfg.exitKeepVehicleTpp = on and true or false
  ctrl.cfg.carExitKeepVehicleTpp = ctrl.cfg.exitKeepVehicleTpp
  return ctrl.cfg.exitKeepVehicleTpp
end
function M.SetExitMatchFov(on) ctrl.cfg.exitMatchFov = on and true or false; return ctrl.cfg.exitMatchFov end
-- passenger-side entry hold (not persisted): false = the mount decision waits for the final seat as before
function M.SetSeatSwitchHold(on) ctrl.cfg.seatSwitchHold = on and true or false; return ctrl.cfg.seatSwitchHold end
-- W9 safe teleport exit (setting vehicle.safeTeleportExit, default on): pushed to Game.UTPPSetSafeTeleportExit by the
-- wiring once a game session exists (the redscript system answers false before), again for every new player object
M.safeExit = { pending = true, playerKey = nil, tries = 0, nextAt = 0, state = "pending", logged = nil }
function M.SetSafeTeleportExit(on)
  ctrl.cfg.safeTeleportExit = on and true or false
  M.safeExit.pending, M.safeExit.tries, M.safeExit.nextAt, M.safeExit.state = true, 0, 0, "pending"
  return ctrl.cfg.safeTeleportExit
end
-- player identity for the push: the entity ID hash (CET may hand out a fresh wrapper per Game.GetPlayer call, so the
-- handles themselves are not compared); a table without GetEntityID (offline) is keyed by itself, an unreadable game
-- handle by one constant key (never a per-wrapper address)
local function safeExitKey(player)
  local okK, key = pcall(function() return tostring(player:GetEntityID().hash) end)
  if okK and key ~= nil and key ~= "nil" then return "id:" .. key end
  if type(player) == "table" then return player end
  return "player"
end
-- the wiring reports frames without a player (main menu / loading): the next player is a new session and gets the push
function M.SafeTeleportExitPlayerGone()
  M.safeExit.playerKey = nil
end
-- one push attempt (wiring, every frame while pending; also callable offline with a fake Game table). The value is sent
-- again only for a new player (key change or after a frame without a player) or a setting change; each outcome is logged
-- once per value.
function M.PushSafeTeleportExit(now, player, log)
  local se = M.safeExit
  if player ~= nil then
    local key = safeExitKey(player)
    if key ~= se.playerKey then se.playerKey, se.pending, se.tries, se.nextAt = key, true, 0, 0 end
  end
  if not se.pending or (now or 0) < se.nextAt then return se.state end
  local fn = Game and Game.UTPPSetSafeTeleportExit
  if type(fn) ~= "function" then
    se.pending, se.state = false, "unavailable"
    if log then log("[utpp_vehiclecam] safe teleport exit: Game.UTPPSetSafeTeleportExit unavailable (utpp_vehicle_exit.reds "
      .. "not installed) - setting has no effect") end
    return se.state
  end
  local ok, r = pcall(fn, ctrl.cfg.safeTeleportExit)
  se.tries = se.tries + 1
  if ok and r ~= false then
    se.pending, se.state = false, "sent"
    if log and se.logged ~= ctrl.cfg.safeTeleportExit then
      se.logged = ctrl.cfg.safeTeleportExit
      log(string.format("[utpp_vehiclecam] safe teleport exit -> %s (Game.UTPPSetSafeTeleportExit ok)",
        tostring(ctrl.cfg.safeTeleportExit)))
    end
  elseif se.tries >= 30 then
    se.pending, se.state = false, "failed"
    if log then log("[utpp_vehiclecam] safe teleport exit: Game.UTPPSetSafeTeleportExit " .. (ok and "returned false"
      or ("failed: " .. tostring(r))) .. " 30 times - given up") end
  else
    se.nextAt = (now or 0) + 1.0
  end
  return se.state
end

-- Car entry approach walk (setting vehicle.approachWalk, default on): V walks to the entry clip's start spot before the mount
-- (utpp_vehicle_approach.reds). Pushed to Game.UTPPSetVehicleApproachEnabled the same way as the safe teleport exit: the
-- redscript system answers false before a game session exists, so the wiring retries until it takes it, and again for a new
-- player object or a setting change.
M.approach = { pending = true, playerKey = nil, tries = 0, nextAt = 0, state = "pending", logged = nil }
function M.SetApproachWalk(on)
  ctrl.cfg.approachWalk = on and true or false
  M.approach.pending, M.approach.tries, M.approach.nextAt, M.approach.state = true, 0, 0, "pending"
  return ctrl.cfg.approachWalk
end
function M.ApproachWalkPlayerGone()
  M.approach.playerKey = nil
end
function M.PushApproachWalk(now, player, log)
  local ap = M.approach
  if player ~= nil then
    local key = safeExitKey(player)
    if key ~= ap.playerKey then ap.playerKey, ap.pending, ap.tries, ap.nextAt = key, true, 0, 0 end
  end
  if not ap.pending or (now or 0) < ap.nextAt then return ap.state end
  local fn = Game and Game.UTPPSetVehicleApproachEnabled
  if type(fn) ~= "function" then
    ap.pending, ap.state = false, "unavailable"
    if log then log("[utpp_vehiclecam] approach walk: Game.UTPPSetVehicleApproachEnabled unavailable (utpp_vehicle_approach.reds "
      .. "not installed) - setting has no effect") end
    return ap.state
  end
  local ok, r = pcall(fn, ctrl.cfg.approachWalk)
  ap.tries = ap.tries + 1
  if ok and r ~= false then
    ap.pending, ap.state = false, "sent"
    if log and ap.logged ~= ctrl.cfg.approachWalk then
      ap.logged = ctrl.cfg.approachWalk
      log(string.format("[utpp_vehiclecam] approach walk -> %s (Game.UTPPSetVehicleApproachEnabled ok)", tostring(ctrl.cfg.approachWalk)))
    end
  elseif ap.tries >= 30 then
    ap.pending, ap.state = false, "failed"
    if log then log("[utpp_vehiclecam] approach walk: Game.UTPPSetVehicleApproachEnabled " .. (ok and "returned false"
      or ("failed: " .. tostring(r))) .. " 30 times - given up") end
  else
    ap.nextAt = (now or 0) + 1.0
  end
  return ap.state
end

-- ---------------------------------------------------------------- UTPP settings (persisted, default on)
-- Stored in utpp_settings.txt inside this mod folder (CET sandbox: mod-relative io). Format: one "key=value" per line.
M.SETTINGS_FILE = "utpp_settings.txt"
function M.LoadSettings(path)
  local f = io.open(path or M.SETTINGS_FILE, "r")
  if not f then return false end
  for line in f:lines() do
    local k, v = string.match(line, "^%s*([%w_]+)%s*=%s*(%S+)")
    if k == "keepTpp" then ctrl.cfg.keepTpp = (v == "true") end
    if k == "pullOut" then ctrl.cfg.pullOut = (v == "true") end
    if k == "forceTppOnEnter" then ctrl.cfg.forceTppOnEnter = (v == "true") end
  end
  f:close()
  return true
end
-- UTPP's own file in this mod folder (fallback without utpp_core); the game's settings are never written
function M.SaveSettings(path)
  local f = io.open(path or M.SETTINGS_FILE, "w")
  if not f then return false end
  f:write("keepTpp=" .. tostring(ctrl.cfg.keepTpp) .. "\n")
  f:write("pullOut=" .. tostring(ctrl.cfg.pullOut) .. "\n")
  f:write("forceTppOnEnter=" .. tostring(ctrl.cfg.forceTppOnEnter) .. "\n")
  f:close()
  return true
end
-- every driver entry starts in a third-person vehicle camera (setting vehicle.forceTppOnEnter, default
-- on). viaCore = the value comes from utpp_core (persisted there).
function M.SetForceTppOnEnter(on, viaCore)
  ctrl.cfg.forceTppOnEnter = on and true or false
  if not viaCore then pcall(M.SaveSettings) end
  return ctrl.cfg.forceTppOnEnter
end
-- Console / Native Settings: keep the third-person camera while entering vehicles (default true).
function M.SetKeepTpp(on)
  ctrl.cfg.keepTpp = on and true or false
  if not ctrl.cfg.keepTpp then ctrl.keep = "off" end
  pcall(M.SaveSettings)
  return ctrl.cfg.keepTpp
end

-- camera pull-out while sitting into the car (part of keep-TPP, default on)
function M.SetPullOut(on)
  ctrl.cfg.pullOut = on and true or false
  pcall(M.SaveSettings)
  return ctrl.cfg.pullOut
end

function M.Status()
  local s = ctrl.stats
  return string.format("utpp_vehiclecam err=%s enabled=%s persp=%d phase=%s reason=%s "
    .. "rides=%d armed=%d done=%d failed=%d requests=%d lastRequest=%d lastActive=%d lastState=%d "
    .. "readErrors=%d requestErrors=%d "
    .. "keepTpp=%s keep=%s keepReason=%s keepMounts=%d keepFallbacks=%d pullOut=%s pull=%.2f pulls=%d "
    .. "keepExits=%d exitPull=%s/%s exitPulls=%d "
    .. "pullPath=%s priorityHold=%s holdRelease=%s camHold=%s camHolds=%d camReleases=%d "
    .. "camCmdsRun=%d camTrace=%s exitMatch=%s exitMatchTime=%s exitMatches=%d exitMatchMisses=%d "
    .. "own=%s ownHeld=%s ownMounts=%d ownHandovers=%d ownHandMatches=%d ownExits=%d "
    .. "ownTakes=%d ownRefused=%d releaseFlushes=%d exitFollow=%s exitFollows=%d"
    .. " rideWatchdog=%s wdTrips=%d wdRequests=%d rideRetakes=%d rideChases=%d handoverGate=%s handGates=%d"
    .. " forceTppOnEnter=%s forcedRides=%d rideBase=%s/%s gameTakes=%d apiMotion=%s"
    .. " exitKeep=%s exitKeepRequests=%d handBlendExits=%d gamePerspChanges=%d safeTeleportExit=%s/%s"
    .. " handGlide=%s handGlides=%d handGlideMisses=%d vehTppLearns=%d passengerRequests=%d lateEntryRides=%d",
    tostring(wiring.err), tostring(ctrl.cfg.enabled),
    ctrl.cfg.perspective, ctrl.phase, ctrl.reason ~= "" and ctrl.reason or "-",
    s.rides, s.armed, s.done, s.failed, s.requests, s.lastRequest, s.lastActive, s.lastState,
    wiring.readErrors, wiring.requestErrors,
    tostring(ctrl.cfg.keepTpp), tostring(ctrl.keep), ctrl.keepReason ~= "" and ctrl.keepReason or "-",
    s.keepMounts or 0, s.keepFallbacks or 0, tostring(ctrl.cfg.pullOut), ctrl.pull or 0, s.pulls or 0,
    s.keepExits or 0, tostring(ctrl.cfg.exitPull), tostring(ctrl.exitPull), s.exitPulls or 0,
    tostring(ctrl.cfg.pullPath), tostring(ctrl.cfg.priorityHold), tostring(ctrl.cfg.holdRelease),
    tostring(ctrl.cam.hold), s.camHolds or 0, s.camReleases or 0,
    s.camCmdsRun or 0, tostring(ctrl.cfg.camTrace), tostring(ctrl.cfg.exitMatch), tostring(ctrl.cfg.exitMatchTime),
    s.exitMatches or 0, s.exitMatchMisses or 0,
    tostring(ctrl.own), tostring(ctrl.ownHeld), s.ownMounts or 0, s.ownHandovers or 0,
    s.ownHandMatches or 0, s.ownExits or 0, s.ownTakes or 0, s.ownRefused or 0, s.releaseFlushes or 0,
    tostring(ctrl.cfg.exitFollow), s.exitFollows or 0,
    tostring(ctrl.cfg.rideWatchdog), s.rideWatchdogTrips or 0, s.rideWatchdogRequests or 0, s.rideRetakes or 0,
    s.rideChases or 0, tostring(ctrl.cfg.ownHandoverGate), s.handGates or 0,
    tostring(ctrl.cfg.forceTppOnEnter), s.forcedRides or 0, tostring(M.PERSP_NAME[ctrl.rideBase.car] or "-"),
    tostring(M.PERSP_NAME[ctrl.rideBase.bike] or "-"), s.gameTakes or 0, tostring(ctrl.ownApiMotion),
    tostring(ctrl.cfg.exitKeepVehicleTpp), s.exitKeepRequests or 0, s.handBlendExits or 0, s.gamePerspChanges or 0,
    tostring(ctrl.cfg.safeTeleportExit), tostring(M.safeExit and M.safeExit.state),
    tostring(ctrl.cfg.handGlide), s.handGlides or 0, s.handGlideMisses or 0, s.vehTppLearns or 0, s.passengerRequests or 0,
    s.lateEntryRides or 0)
end

-- ---------------------------------------------------------------- in-game wiring (skipped by the offline test)
-- fix: the old guard also required type(GetMod) == "function" at FILE LOAD, which was false in CET - no events were
-- registered and the handoff never ran in game (scripting.log: only "Mod utpp_vehiclecam loaded!"). Events are registered
-- whenever registerForEvent exists; GetMod is resolved lazily; the outcome is printed once.
if type(registerForEvent) == "function" then
  local tries, done, clock = 0, false, 0
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
  -- CET prints do not reach a file in game; every status / phase line also goes to the UTPP red4ext log.
  local function sink(msg)
    print(msg)
    if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(msg)) end) end
  end
  if ctrl.trace == nil then ctrl.trace = sink end -- controller phase / request lines on by default in game
  local lastObsKey = nil
  pcall(M.LoadSettings) -- persisted UTPP settings (missing file = defaults)

  -- Native Settings UI (Mods > UTPP): "Keep TPP camera when entering vehicles" (persisted, default on).
  local function buildNativeSettings()
    -- shared UTPP settings (utpp_core: one UTPP tab, one utpp_settings.txt, reset). The values already loaded
    -- from this mod's own file are the initial values when utpp_core has none stored; own UI below = fallback.
    local core = GetMod and GetMod("utpp_core") or nil
    if type(core) == "table" and type(core.Register) == "function" then
      local okC, errC = pcall(function()
        core.RegisterSection("vehicle", "Vehicles", { owner = "utpp_vehiclecam" })
        core.Register({ key = "vehicle.keepTpp", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.keepTpp, label = "Keep TPP camera when entering vehicles",
          desc = "Keeps the third-person camera and full body during the enter animation, then hands over to the vehicle TPP camera.",
          apply = function(v) if v ~= ctrl.cfg.keepTpp then M.SetKeepTpp(v) end end })
        core.Register({ key = "vehicle.pullOut", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.pullOut, label = "Camera pull-out while sitting down",
          desc = "With the option above: during the last part of the enter animation the camera moves behind and above the "
          .. "vehicle instead of following the character into the car.",
          apply = function(v) if v ~= ctrl.cfg.pullOut then M.SetPullOut(v) end end })
        core.Register({ key = "vehicle.rideWatchdog", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.rideWatchdog, label = "Ride watchdog: camera follows vehicle",
          desc = "After the camera is handed to the vehicle camera: if the camera stays still while the vehicle drives, request "
          .. "the vehicle camera again, then take the view back behind the vehicle (chase camera). Off = the camera may stay "
          .. "behind in the world (seen once after a hand-over without a readable vehicle camera pose).",
          apply = function(v) if v ~= ctrl.cfg.rideWatchdog then M.SetRideWatchdog(v) end end })
        core.Register({ key = "vehicle.safeTeleportExit", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.safeTeleportExit, label = "Safe exit position for motorcycles",
          desc = "When a quick motorcycle exit places the character in the air or inside the bike, she is moved to a free "
          .. "spot on the ground beside or behind it (third person only).",
          apply = function(v) if v ~= ctrl.cfg.safeTeleportExit or M.safeExit.state == "pending" then
            M.SetSafeTeleportExit(v) end end })
        core.Register({ key = "vehicle.forceTppOnEnter", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.forceTppOnEnter, label = "Start vehicle rides in third person",
          desc = "Every vehicle entry starts in a third-person vehicle camera. The vehicle camera key still switches it "
          .. "during the ride, and that choice is not kept after leaving the vehicle.",
          apply = function(v) if v ~= ctrl.cfg.forceTppOnEnter then M.SetForceTppOnEnter(v, true) end end })
        core.Register({ key = "vehicle.approachWalk", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.approachWalk, label = "Walk to the car door before getting in",
          desc = "When you enter a car, the character first walks to the spot where the enter animation starts, instead of "
          .. "being pulled there in a fraction of a second. Pressing the interact key again, moving the stick against the "
          .. "walk, taking damage or a moving car cancels it. Off = the game's own quick slide.",
          apply = function(v) if v ~= ctrl.cfg.approachWalk or M.approach.state == "pending" then
            M.SetApproachWalk(v) end end })
        core.Register({ key = "vehicle.jumpOutSidePush", section = "vehicle", type = "bool", default = true,
          initial = ctrl.cfg.jumpOutSidePush, label = "Clear the car when jumping out",
          desc = "When you jump out of a standing car (double press of the exit key), the character gets a small push away "
          .. "from the car body, so she does not touch the fender on the way down. Off = the game's own jump-out.",
          apply = function(v) if v ~= ctrl.cfg.jumpOutSidePush then M.SetJumpOutSidePush(v) end end })
      end)
      sink("[utpp_vehiclecam] settings via utpp_core: " .. (okC and "ok" or ("FAILED " .. tostring(errC))))
      if okC then return end
    end
    local ns = GetMod and GetMod("nativeSettings") or nil
    if type(ns) ~= "table" or type(ns.addTab) ~= "function" then
      sink("[utpp_vehiclecam] Native Settings not found - console: GetMod(\"utpp_vehiclecam\").SetKeepTpp(true|false)")
      return
    end
    local okUi, err = pcall(function()
      if not (ns.pathExists and ns.pathExists("/utpp")) then ns.addTab("/utpp", "UTPP") end
      ns.addSubcategory("/utpp/vehicle", "Vehicles")
      ns.addSwitch("/utpp/vehicle", "Keep TPP camera when entering vehicles",
        "Keeps the third-person camera and full body during the enter animation, then hands over to the vehicle TPP camera.",
        ctrl.cfg.keepTpp, true, function(v) M.SetKeepTpp(v) end)
      ns.addSwitch("/utpp/vehicle", "Camera pull-out while sitting down",
        "With the option above: during the last part of the enter animation the camera moves behind and above the vehicle "
        .. "instead of following the character into the car.", ctrl.cfg.pullOut, true, function(v) M.SetPullOut(v) end)
      ns.addSwitch("/utpp/vehicle", "Start vehicle rides in third person",
        "Every vehicle entry starts in a third-person vehicle camera. The vehicle camera key still switches it during the "
        .. "ride, and that choice is not kept after leaving the vehicle.", ctrl.cfg.forceTppOnEnter, true,
        function(v) M.SetForceTppOnEnter(v) end)
    end)
    sink("[utpp_vehiclecam] Native Settings tab UTPP: " .. (okUi and "ok" or ("FAILED " .. tostring(err))))
  end

  local function enumInt(v)
    if type(v) == "number" then return v end
    local ok, n = pcall(EnumInt, v)
    if ok and type(n) == "number" then return n end
    return nil
  end

  local function nameString(n)
    if type(n) == "string" then return n end
    local ok, s = pcall(function() return n.value end)
    if ok and type(s) == "string" then return s end
    ok, s = pcall(Game.NameToString, n)
    if ok and type(s) == "string" then return s end
    return nil
  end

  local function observe(player)
    local obs = { vehicleState = 0, mounted = false }
    local ok = pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if bb then
        obs.vehicleState = bb:GetInt(defs.Vehicle) or 0
        pcall(function() obs.highLevel = bb:GetInt(defs.HighLevel) end) -- scene tiers: no forced TPP request
      end
    end)
    if not ok then wiring.readErrors = wiring.readErrors + 1 end
    local okV, veh = pcall(Game.GetMountedVehicle, player)
    if not okV or veh == nil then return obs, nil end
    obs.mounted = true
    -- active/persistent were always nil in older versions. Several conversions are tried and the raw value / method is
    -- logged once per ride, so one vehicle entry shows which read works.
    local okM, mgr = pcall(function() return veh:GetCameraManager() end)
    local function persp(getter)
      local okR, raw = pcall(function() return getter(mgr) end)
      if not okR then return nil, "err:" .. tostring(raw), "?" end
      local n = enumInt(raw)
      if n ~= nil then return n, "EnumInt", raw end
      local okS, s = pcall(function() return Game.EnumValueToString("vehicleCameraPerspective", raw) end)
      local text = (okS and type(s) == "string" and s ~= "") and s or tostring(raw)
      local best, bestLen = nil, 0
      for i, name in pairs(M.PERSP_NAME) do
        if string.find(text, name, 1, true) and #name > bestLen then best, bestLen = i, #name end
      end
      if best ~= nil then return best, "name", raw end
      local num = tonumber(text)
      if num ~= nil then return num, "tonumber", raw end
      return nil, "unconverted", raw
    end
    if okM and mgr ~= nil then
      local a, how, raw = persp(function(m) return m:GetActivePerspective() end)
      local p = persp(function(m) return m:GetPersistentPerspective() end)
      obs.activePersp, obs.persistentPersp = a, p
      -- (1): the manager's own TPP flag (the close bike camera is accepted only with it)
      local okT, tpp = pcall(function() return mgr:IsTPPActive() end)
      if okT and type(tpp) == "boolean" then obs.vehTppActive = tpp end
      if not wiring.camDiagDone then
        wiring.camDiagDone = true
        -- vehicle camera blend (VehicleCameraManager_Record.cameraBlendTime via Vehicle_Record.CameraManagerParams)
        local blendInfo = "?"
        local okB, isBike = pcall(function() return veh:IsA("vehicleBikeBaseObject") end) -- exit match 2
        ctrl.vehBike = (okB and isBike == true) or nil
        -- positive car identification (the IsA read succeeded and said no); vehBike alone is nil for both a
        -- car and an unread class (a motorcycle whose first camera read failed)
        ctrl.vehCar = (okB and isBike == false) or nil
        pcall(function() -- vehicle record: the key of the learned vehicle TPP camera poses
          local key = TDBID.ToStringDEBUG(veh:GetRecordID())
          if type(key) == "string" and key ~= "" then ctrl.vehRecKey = key end
        end)
        pcall(function()
          local rec = veh:GetRecord():CameraManagerParams()
          local id = rec:GetID()
          local idStr = TDBID.ToStringDEBUG(id)
          local cur = TweakDB:GetFlat(idStr .. ".cameraBlendTime")
          blendInfo = string.format("%s.cameraBlendTime=%s", tostring(idStr), tostring(cur))
          if type(cur) == "number" then ctrl.vehBlend = cur end -- mount handover fix
          if string.find(tostring(idStr), "_2w_", 1, true) then ctrl.vehBike = true; ctrl.vehCar = nil end -- exit match 2 (fallback)
          if ctrl.cfg.blendTime ~= nil and type(cur) == "number" and cur ~= ctrl.cfg.blendTime then
            local okSet = TweakDB:SetFlat(idStr .. ".cameraBlendTime", ctrl.cfg.blendTime)
            if okSet ~= false then ctrl.vehBlend = ctrl.cfg.blendTime end
            blendInfo = blendInfo .. string.format(" -> set %.2f (%s)", ctrl.cfg.blendTime, tostring(okSet))
          end
        end)
        wiring.camDiag = string.format("veh=%s mgr=%s activeRaw=%s:%s -> %s (%s) blend %s%s", type(veh), type(mgr),
          type(raw), tostring(raw), tostring(a), tostring(how), blendInfo, ctrl.vehBike and " motorcycle" or "")
      end
    elseif not wiring.camDiagDone then
      wiring.camDiagDone = true
      wiring.camDiag = string.format("veh=%s GetCameraManager failed: %s", type(veh), tostring(mgr))
    end
    pcall(function()
      local info = Game.GetMountingFacility():GetMountingInfoSingleWithObjects(player)
      local slot = info and nameString(info.slotId.id) or nil
      if slot ~= nil and slot ~= "" and slot ~= "None" then
        obs.driver = string.find(slot, "seat_front_left", 1, true) ~= nil
        obs.seat = slot
      end
    end)
    -- passenger-side entry: the inputs of vanilla PassangerSwitchSeatsCondition (free driver slot, switch not disabled by
    -- the vehicle data package), read only while a non-driver slot is mounted outside the driver states (the mount decision
    -- runs on the first mounted frame, PSM state 0 or 4)
    if obs.driver == false and obs.vehicleState ~= 1 and obs.vehicleState ~= 6 then
      pcall(function()
        local free = VehicleComponent.IsSlotAvailable(veh:GetGame(), veh, CName.new("seat_front_left"))
        if type(free) == "boolean" then obs.driverSeatFree = free end
      end)
      pcall(function()
        local dis = veh:GetRecord():VehDataPackage():DisableSwitchSeats()
        if type(dis) == "boolean" then obs.switchSeatsDisabled = dis end
      end)
    end
    -- pull-out: player-vehicle distance and the target camera pose (player local frame) while the hold is active
    pcall(function()
      local vp = veh:GetWorldPosition()
      wiring.vehPose = { x = vp.x, y = vp.y, z = vp.z, yaw = veh:GetWorldYaw() }
      wiring.lastVehPose = wiring.vehPose -- camera trace after the unmount
      local pp = player:GetWorldPosition()
      obs.vehDist = math.sqrt((pp.x - vp.x) ^ 2 + (pp.y - vp.y) ^ 2)
      obs.vehX, obs.vehY = vp.x, vp.y -- jump-out follow: vehicle speed
    end)
    -- utpp_camera's mount pivot blend is running (optional API; older cameras: nil = no wait)
    if wiring.cam and type(wiring.cam.MountBlendActive) == "function" then
      local okB, b = pcall(wiring.cam.MountBlendActive)
      obs.camMountBlend = okB and b == true
    end
    -- ride watchdog: the active (rendered) camera's world position while mounted
    if ctrl.cfg.rideWatchdog then pcall(function()
      local cs = Game.GetCameraSystem()
      local tr = activeCameraTransform(cs)
      if tr == nil then return end
      local pos = tr.position
      if pos == nil and tr.GetPosition then pos = tr:GetPosition() end
      if pos ~= nil then obs.camX, obs.camY, obs.camZ = pos.x, pos.y, pos.z end
    end) end
    return obs, veh
  end

  -- target pose of the pull-out (mount hold) and of the exit pull (exit hold + release after the unmount: the last
  -- vehicle pose is kept, the car does not move while V gets out)

  -- exit match fix: world pose of a camera component (GetLocalToWorld: translation + forward axis)
  local function componentWorld(comp)
    local m = comp:GetLocalToWorld()
    local okT, pos = pcall(function() return Matrix.GetTranslation(m) end)
    if not okT or pos == nil then pos = m.W end
    local okF, fwd = pcall(function() return Matrix.GetDirectionVector(m) end)
    if not okF or fwd == nil then fwd = m.Y end
    return pos, fwd
  end
  local function findComponent(player, name)
    local ok, comp = pcall(function() return player:FindComponentByName(name) end)
    if not ok or comp == nil then ok, comp = pcall(function() return player:FindComponentByName(CName.new(name)) end) end
    return (ok and comp) or nil
  end
  -- world position / forward -> vehicle frame pose { x, y, z, yaw (deg, relative to the vehicle), pitch (deg, + = up) }
  local function vehFramePose(pos, fwd, src)
    local v = wiring.vehPose or wiring.lastVehPose
    if v == nil or pos == nil or fwd == nil then return nil end
    local rad = math.pi / 180
    local c, s = math.cos(-v.yaw * rad), math.sin(-v.yaw * rad)
    local rx, ry = pos.x - v.x, pos.y - v.y
    local yaw = math.deg(math.atan2(-fwd.x, fwd.y)) - v.yaw
    while yaw > 180 do yaw = yaw - 360 end
    while yaw <= -180 do yaw = yaw + 360 end
    return { x = c * rx - s * ry, y = s * rx + c * ry, z = pos.z - v.z, yaw = yaw,
             pitch = math.deg(math.atan2(fwd.z, math.sqrt(fwd.x * fwd.x + fwd.y * fwd.y))), src = src }
  end
  local function vehTppCamPose(player)
    local comp = findComponent(player, "vehicleTPPCamera") -- player.script OnRequestComponents: vehicleTPPCameraComponent
    if comp == nil then return nil, "vehicleTPPCamera component not found" end
    local ok, pos, fwd = pcall(componentWorld, comp)
    if not ok then return nil, "vehicleTPPCamera read failed: " .. tostring(pos) end
    return vehFramePose(pos, fwd, "vehicleTPPCamera")
  end
  local function activeCamPose()
    local cs = Game.GetCameraSystem()
    local tr = activeCameraTransform(cs)
    if tr == nil then return nil end
    local pos = tr.position
    if pos == nil and tr.GetPosition then pos = tr:GetPosition() end
    local pose = vehFramePose(pos, cs:GetActiveCameraForward(), "active camera")
    if pose == nil then return nil end
    -- (3): FOV of the rendered camera (nil when unreadable -> "fov unread")
    local okF, fv = pcall(function() return cs:GetActiveCameraFOV() end)
    if okF and type(fv) == "number" then pose.fov = fv end
    -- the transform's own orientation as the alternative reading (used when the forward reads exactly 0 / 0)
    pcall(function()
      local q = tr.orientation
      if q == nil then return end
      local qi, qj, qk, qr = q.i, q.j, q.k, q.r
      if type(qi) ~= "number" then return end
      -- forward axis (0, 1, 0) rotated by the quaternion
      local fx = 2 * (qi * qj - qr * qk)
      local fy = 1 - 2 * (qi * qi + qk * qk)
      local fz = 2 * (qj * qk + qr * qi)
      local alt = vehFramePose(pos, { x = fx, y = fy, z = fz }, "active camera")
      if alt then pose.altYaw, pose.altPitch, pose.altSrc = alt.yaw, alt.pitch, "transform orientation" end
    end)
    return pose
  end
  -- exit match 2: vehicleTPPCamera's GetLocalToWorld stays at the seat ((-0.33,-0.30,0.16) in the vehicle
  -- frame, follows V out) - not the rendered camera. The active camera (before our activation) is the rendered vehicle
  -- TPP camera ("cam veh=" first exit sample (-0.69,-8.30,3.95) / (5.01,-6.84,2.82)): it is the first candidate.
  ctrl.gameCamQuery = function(allowActive)
    local player = Game.GetPlayer()
    if player == nil then return nil, "no player" end
    local list, errs = {}, {}
    if allowActive then
      local ok, p = pcall(activeCamPose)
      if ok and p ~= nil then list[#list + 1] = p
      else errs[#errs + 1] = "active camera " .. (ok and "unread" or ("failed: " .. tostring(p))) end
    end
    local pose, err = vehTppCamPose(player)
    if pose ~= nil then list[#list + 1] = pose elseif err then errs[#errs + 1] = tostring(err) end
    if #list == 0 then return nil, (#errs > 0) and table.concat(errs, "; ") or "no game camera pose" end
    return list, (#errs > 0) and table.concat(errs, "; ") or nil
  end

  local function updatePullTarget(player)
    local want = ((ctrl.keep == "mount" or ctrl.keep == "chase") and (ctrl.cfg.pullOut or (ctrl.rideRetakes or 0) > 0))
      or ctrl.exitPull ~= nil
      or (ctrl.pulling and ctrl.Releasing()) -- mount jump fix: the pose is held through the handover release blend
    if not want or wiring.vehPose == nil then
      ctrl.pullFrame = nil
      if ctrl.keep == "off" and ctrl.exitPull == nil then wiring.vehPose = nil end
      return
    end
    local okP, err = pcall(function()
      local pp, v = player:GetWorldPosition(), wiring.vehPose
      local pyaw = player:GetWorldYaw()
      ctrl.pullFrame = { px = pp.x, py = pp.y, pz = pp.z, pyaw = pyaw, vx = v.x, vy = v.y, vz = v.z, vyaw = v.yaw }
    end)
    if not okP and not wiring.pullErrLogged then
      wiring.pullErrLogged = true
      sink("[utpp_vehiclecam] pull-out pose FAILED: " .. tostring(err))
    end
  end

  -- mount jump fix: which camera is rendered - the active camera (camera system)
  -- in the vehicle frame (x right, y forward, z up from the vehicle origin) and its forward pitch, <= camTraceHz lines/s
  -- around the pull-out / handover / exit. Pull-out target: (0, -pullBack, pullHeight); vehicle TPPFar: unknown (logged).
  local function traceCam()
    if not ctrl.cfg.camTrace then return end
    local wanted = ctrl.CamTraceWanted()
    -- continuity measure: the rendered camera's change PER FRAME (the 20 Hz lines below hide a one-frame snap), in
    -- the vehicle frame; max over the frames since the last line (fstep / frot) and over the window (summary line)
    wiring.cont = wiring.cont or M.NewContinuity()
    if wanted then
      local okA, ap = pcall(activeCamPose)
      if okA and ap then
        local hull = (ctrl.vehCar == true and ctrl.vehBike ~= true and ctrl.cfg.pullHull ~= false) and M.HullFor(ctrl.cfg, ctrl.vehRecKey) or nil
        M.ContinuityStep(wiring.cont, clock, ap, ctrl.handAt, tonumber(ctrl.handBlendDur), hull, tonumber(ctrl.cfg.hullWarnDepth))
        -- the rendered camera inside the box: one WARNING line per 0.5 s (the step / rotation figures stay quiet for a camera that sits inside the body)
        if hull ~= nil and M.ContinuityBoxWarnDue(wiring, M.HullDepth(hull, ap.x, ap.y, ap.z), clock, tonumber(ctrl.cfg.hullWarnDepth)) then
          sink(string.format("[utpp_vehiclecam] t=%.2f camera inside the vehicle box WARNING: rendered camera veh=(%.2f,%.2f,%.2f) %.2f m inside the box (hx %.2f hy %.2f/%.2f top %.2f), keep=%s pull=%.2f%s",
            clock, ap.x, ap.y, ap.z, M.HullDepth(hull, ap.x, ap.y, ap.z), hull.hx, hull.hyF, hull.hyB, hull.top, tostring(ctrl.keep), ctrl.pull or 0,
            hull.topY and string.format(", top profile %.2f here", M.HullTop(hull, ap.y)) or ""))
        end
      end
    elseif wiring.cont.frames > 0 then
      sink(M.ContinuitySummary(wiring.cont, clock))
      wiring.cont = M.NewContinuity()
      wiring.boxWarned, wiring.boxWarnAt = nil, nil
    end
    if not wanted then return end
    local hz = math.max(1, tonumber(ctrl.cfg.camTraceHz) or 20)
    if wiring.camTraceAt ~= nil and clock >= wiring.camTraceAt and clock - wiring.camTraceAt < 1 / hz - 1e-4 then return end
    wiring.camTraceAt = clock
    local ok, line = pcall(function()
      local cs = Game.GetCameraSystem()
      local tr = activeCameraTransform(cs)
      if tr == nil then return "no active camera transform" end
      local pos = tr.position
      if pos == nil and tr.GetPosition then pos = tr:GetPosition() end
      local fwd = cs:GetActiveCameraForward()
      local v = wiring.vehPose or wiring.lastVehPose
      if v == nil then return string.format("world=(%.2f,%.2f,%.2f) (no vehicle pose)", pos.x, pos.y, pos.z) end
      local rad = math.pi / 180
      local c, s = math.cos(-v.yaw * rad), math.sin(-v.yaw * rad)
      local rx, ry = pos.x - v.x, pos.y - v.y
      local pitch = math.deg(math.atan2(fwd.z, math.sqrt(fwd.x * fwd.x + fwd.y * fwd.y)))
      local extra = ""
      local player = Game.GetPlayer()
      local okV, vp = pcall(vehTppCamPose, player)
      if okV and vp then extra = extra .. string.format(" vtpp=(%.2f,%.2f,%.2f)", vp.x, vp.y, vp.z) end
      if wiring.own then -- UTPP's own camera in the vehicle frame
        local okU, up = pcall(function()
          local comp = findComponent(player, "utppCamera")
          if comp == nil then return nil end
          local p, f = componentWorld(comp)
          return vehFramePose(p, f, "utppCamera")
        end)
        if okU and up then extra = extra .. string.format(" utpp=(%.2f,%.2f,%.2f)", up.x, up.y, up.z) end
      end
      if ctrl.match then extra = extra .. string.format(" match=%.2f", ctrl.MatchWeight()) end
      if ctrl.exitFollow and ctrl.exitFollow.fol and ctrl.exitFollow.folAt == ctrl.clock then extra = extra .. " fol=" .. ctrl.exitFollow.fol end -- (append-only): launch follower branch / speed / frame time
      -- the active camera's world position and the vehicle speed (append-only)
      extra = extra .. string.format(" camW=(%.2f,%.2f,%.2f) vspd=%.1f", pos.x, pos.y, pos.z, ctrl.vehSpeed or 0)
      -- (append-only): the active camera's relative yaw, the pitch of the transform's own orientation (the
      -- forward pitch above reads 0 at rest) and its FOV; the glide weight
      local okA, ap = pcall(activeCamPose)
      if okA and ap then
        extra = extra .. string.format(" yaw=%.0f altPitch=%s fov=%s", ap.yaw, ap.altPitch and string.format("%.0f", ap.altPitch)
          or "-", ap.fov and string.format("%.0f", ap.fov) or "-")
      end
      if ctrl.handMatch and ctrl.handMatch.glide then
        extra = extra .. string.format(" glide=%.2f", math.min(1, math.max(0, (clock - ctrl.handMatch.at) / ctrl.handMatch.dur)))
      end
      -- per-frame maxima since the previous line (append-only fields): translation m, rotation deg (yaw + pitch)
      local cn = wiring.cont
      if cn and cn.lineStep ~= nil then
        extra = extra .. string.format(" fstep=%.3f frot=%.1f", cn.lineStep, cn.lineRot)
        cn.lineStep, cn.lineRot = 0, 0
      end
      return string.format("veh=(%.2f,%.2f,%.2f) pitch=%.0f%s", c * rx - s * ry, s * rx + c * ry, pos.z - v.z, pitch, extra)
    end)
    if not ok and wiring.camTraceErr then return end
    if not ok then wiring.camTraceErr = true end
    sink(string.format("[utpp_vehiclecam] t=%.2f cam %s keep=%s hold=%s pull=%.2f%s", clock,
      ok and tostring(line) or ("trace FAILED: " .. tostring(line)), tostring(ctrl.keep), tostring(ctrl.cam.hold),
      ctrl.pull or 0, ctrl.Releasing() and " releasing" or ""))
  end

  -- door probe (camTrace only, test runs): when do the front doors of the test car start / stop moving, against the
  -- mount and the enter clip? The door clip is started by game code (doorOpenFront* anim event), none of the enter clips
  -- carries its timing. Target: the mounted vehicle, else the harness car (DynamicEntitySystem tag UTPPCar). Logs the door
  -- state of the vehicle PS on change and the first / last frame a door component moves (> 1 deg or > 1 cm from its rest pose).
  local DOOR_SIDES = {
    { key = "fl", slot = "seat_front_left", comps = { "door_fl_a", "door_front_left_a", "door_fl", "door_front_left" } },
    { key = "fr", slot = "seat_front_right", comps = { "door_fr_a", "door_front_right_a", "door_fr", "door_front_right" } },
  }
  local function doorProbe()
    if not ctrl.cfg.camTrace then wiring.door = nil; return end
    local D = wiring.door or { sides = {} }
    wiring.door = D
    local okP, player = pcall(Game.GetPlayer)
    if not okP or player == nil then return end
    local okV, veh = pcall(Game.GetMountedVehicle, player)
    if not okV or veh == nil then
      if D.tagVeh == nil or clock - (D.tagAt or -10) > 1 then
        D.tagAt = clock
        pcall(function()
          local list = Game.GetDynamicEntitySystem():GetTagged(CName.new("UTPPCar"))
          D.tagVeh = (list and list[1]) or nil
        end)
      end
      veh = D.tagVeh
    end
    if veh == nil then return end
    local okI, id = pcall(function() return veh:GetEntityID().hash end)
    id = okI and tostring(id) or "?"
    if D.veh ~= id then D.veh, D.sides = id, {} end
    local vp = veh:GetWorldPosition()
    local vyaw = veh:GetWorldYaw()
    local rad = math.pi / 180
    local c, s = math.cos(-vyaw * rad), math.sin(-vyaw * rad)
    for _, side in ipairs(DOOR_SIDES) do
      local S = D.sides[side.key] or {}
      D.sides[side.key] = S
      local okS, st = pcall(function()
        local raw = veh:GetVehiclePS():GetDoorState(Enum.new("EVehicleDoor", side.slot))
        local okN, n = pcall(function() return Game.EnumValueToString("VehicleDoorState", raw) end)
        return (okN and type(n) == "string" and n ~= "") and n or tostring(enumInt(raw) or raw)
      end)
      if okS and st ~= S.state then
        sink(string.format("[utpp_vehiclecam] t=%.3f door %s state %s -> %s", clock, side.key, tostring(S.state), tostring(st)))
        S.state = st
      end
      if S.comp == nil and not S.noComp then
        for _, n in ipairs(side.comps) do
          local comp = findComponent(veh, n)
          if comp ~= nil then S.comp, S.compName = comp, n; break end
        end
        if S.comp == nil then
          S.noComp = true
          sink(string.format("[utpp_vehiclecam] door %s: no door component (%s)", side.key, table.concat(side.comps, ", ")))
        end
      end
      if S.comp ~= nil then
        local okW, pos, fwd = pcall(componentWorld, S.comp)
        if okW and pos ~= nil and fwd ~= nil then
          local rx, ry = pos.x - vp.x, pos.y - vp.y
          local p = { x = c * rx - s * ry, y = s * rx + c * ry, z = pos.z - vp.z,
                      yaw = math.deg(math.atan2(-fwd.x, fwd.y)) - vyaw, pitch = math.deg(math.atan2(fwd.z, math.sqrt(fwd.x ^ 2 + fwd.y ^ 2))) }
          if S.rest == nil then S.rest, S.last = p, p end
          local dRest = math.max(math.abs(((p.yaw - S.rest.yaw + 180) % 360) - 180), math.abs(p.pitch - S.rest.pitch),
            100 * math.sqrt((p.x - S.rest.x) ^ 2 + (p.y - S.rest.y) ^ 2 + (p.z - S.rest.z) ^ 2))
          local dLast = math.max(math.abs(((p.yaw - S.last.yaw + 180) % 360) - 180), math.abs(p.pitch - S.last.pitch),
            100 * math.sqrt((p.x - S.last.x) ^ 2 + (p.y - S.last.y) ^ 2 + (p.z - S.last.z) ^ 2))
          if dLast > 0.05 then S.movedAt = clock end
          if not S.moving and dRest > 1 then
            S.moving, S.startAt = true, clock
            sink(string.format("[utpp_vehiclecam] t=%.3f door %s moving (%s) from rest yaw=%.1f pitch=%.1f pos=(%.2f,%.2f,%.2f)",
              clock, side.key, S.compName, S.rest.yaw, S.rest.pitch, S.rest.x, S.rest.y, S.rest.z))
          elseif S.moving and S.movedAt and clock - S.movedAt > 0.3 then
            S.moving = false
            sink(string.format("[utpp_vehiclecam] t=%.3f door %s settled after %.2f s at yaw=%.1f pitch=%.1f pos=(%.2f,%.2f,%.2f)",
              S.movedAt, side.key, S.movedAt - S.startAt, p.yaw, p.pitch, p.x, p.y, p.z))
            S.rest = p
          end
          S.last = p
        end
      end
    end
  end

  -- jump-out push diagnostic (camTrace only): V in the car frame at ~20 Hz from the jump-out (V out of the car) until
  -- 1.5 s after the landing. The car is the entity captured while mounted (observe), else the harness car of doorProbe; its LIVE pose
  -- is read every sample (wiring.lastVehPose is frozen at the unmount; the car goes on driving in a moving jump-out).
  local function joDiagSample(player)
    local veh = wiring.joVeh or (wiring.door and wiring.door.tagVeh) or nil
    local car, frozen = nil, false
    if veh ~= nil then
      local okC = pcall(function()
        local p = veh:GetWorldPosition()
        car = { x = p.x, y = p.y, z = p.z, yaw = veh:GetWorldYaw() }
      end)
      if not okC then car = nil end
    end
    if car == nil then car, frozen = wiring.vehPose or wiring.lastVehPose, true end
    if car == nil then return nil end
    local pp = player:GetWorldPosition()
    return { car = car, v = { x = pp.x, y = pp.y, z = pp.z }, frozen = frozen }
  end
  local function joDoorText()
    local D = wiring.door
    if D == nil or D.sides == nil then return nil end
    local parts = {}
    for _, k in ipairs({ "fl", "fr" }) do
      local S = D.sides[k]
      if S ~= nil and S.state ~= nil then parts[#parts + 1] = k .. ":" .. tostring(S.state) .. (S.moving and "*" or "") end
    end
    return #parts > 0 and table.concat(parts, ",") or nil
  end
  local function joDiag(player, obs)
    wiring.jo = wiring.jo or M.NewJoDiag()
    local st = wiring.jo
    local active = M.JoDiagActive(ctrl.cfg, ctrl.exitJump, obs.mounted)
    if active and not M.JoDiagDue(st, clock, ctrl.cfg.joDiagHz) then return end
    local sample = nil
    if active and not st.done then
      local okS, s = pcall(joDiagSample, player)
      if okS then sample = s end
    end
    local line = M.JoDiagTick(st, clock, active, sample, active and joDoorText() or nil)
    if line then sink(line) end
  end

  local function request(player, p)
    local how = "?"
    local ok, err = pcall(function()
      local ev = vehicleRequestCameraPerspectiveEvent.new()
      local okEnum, errEnum = pcall(function()
        ev.cameraPerspective = Enum.new("vehicleCameraPerspective", M.PERSP_NAME[p])
      end)
      if okEnum then how = "Enum.new" else
        how = "number (Enum.new failed: " .. tostring(errEnum) .. ")"
        ev.cameraPerspective = p
      end
      player:QueueEvent(ev)
    end)
    if not ok then wiring.requestErrors = wiring.requestErrors + 1 end
    if not wiring.reqDiagDone then -- how the first request of a ride was built / queued
      wiring.reqDiagDone = true
      sink(string.format("[utpp_vehiclecam] request %s built via %s, queued on player: %s", tostring(M.PERSP_NAME[p]), how,
        ok and "ok" or ("ERROR " .. tostring(err))))
    end
  end

  -- Vehicle path: utpp_camera's "vehicle" decision asks here first (synchronous: the hold starts before the
  -- camera would cut to FPP in the same update)
  local function ownDelegate(reason)
    local okP, player = pcall(Game.GetPlayer)
    if not okP or player == nil then return false end
    local obs = observe(player)
    ctrl.clock = clock
    local took = ctrl.TryStartOwn(clock, obs, "camera delegate: " .. tostring(reason))
    if took then
      local okO, orbit = pcall(wiring.cam.OrbitPose)
      M.RunOwnCmds(ctrl, wiring.cam, M.OwnPose(ctrl, ctrl.pullFrame, okO and orbit or nil))
    end
    local okH, held = pcall(wiring.cam.VehicleHeld)
    return took and okH and held == true
  end
  local OWN_API = { "HoldForVehicle", "SetPoseOverride", "ReleaseToOrbit", "HandToVehicle", "TakeFromVehicle",
                    "SetVehiclePriority", "VehicleReady", "VehicleHeld", "OrbitPose" }
  local function tryOwn()
    local okC, cam = pcall(GetMod, "utpp_camera")
    local missing = {}
    if not okC or type(cam) ~= "table" then missing[1] = "utpp_camera not installed"
    else for _, n in ipairs(OWN_API) do if type(cam[n]) ~= "function" then missing[#missing + 1] = n end end end
    if #missing > 0 then
      sink("[utpp_vehiclecam] vehicle camera handoff inactive (utpp_camera vehicle API unavailable: "
        .. table.concat(missing, ", ") .. ")")
      return false
    end
    wiring.cam, wiring.own, ctrl.own = cam, true, true
    ctrl.ownCamReady = function() return cam.VehicleReady() end
    if type(cam.OrbitPoseAt) == "function" then ctrl.orbitAt = function(y, p) return cam.OrbitPoseAt(y, p) end end
    if type(cam.VehicleExitWanted) == "function" then ctrl.ownExitQuery = function() return cam.VehicleExitWanted() end end
    -- exit take-over from the game camera only with vehicle API >= 2 (feature-detected)
    local apiV = cam.VehicleApiVersion
    if type(apiV) == "function" then local okV, v = pcall(apiV); apiV = okV and v or nil end
    ctrl.ownApiMotion = (tonumber(apiV) or 1) >= 2 and type(cam.TakeForVehicleExit) == "function"
    local okD = type(cam.SetVehicleDelegate) == "function" and pcall(cam.SetVehicleDelegate, ownDelegate, "utpp_vehiclecam")
    sink(string.format("[utpp_vehiclecam] vehicle path on utppCamera (utpp_camera %s, delegate %s): "
      .. "keepTpp=%s pullOut=%s forceTppOnEnter=%s exitFromGame=%s", tostring(cam.VERSION),
      okD and "ok" or "unavailable", tostring(ctrl.cfg.keepTpp), tostring(ctrl.cfg.pullOut),
      tostring(ctrl.cfg.forceTppOnEnter), ctrl.ownApiMotion and "TakeForVehicleExit" or "unavailable (vehicle API < 2)"))
    return true
  end

  -- The vehicle path is started once, after startDelay CET frames (the game systems and utpp_camera are up by then).
  local startDelay = 300
  local function startPath(final)
    if not final then return false end
    if type(GetMod) ~= "function" then
      wiring.err = "GetMod unavailable"
      sink("[utpp_vehiclecam] GetMod unavailable - vehicle camera handoff inactive")
    else
      tryOwn()
    end
    return true
  end

  registerForEvent("onInit", function()
    if not announced then
      announced = true
      sink("[utpp_vehiclecam] onInit: events registered")
      pcall(buildNativeSettings)
    end
    done = startPath(false)
  end)

  registerForEvent("onUpdate", function(delta)
    clock = clock + math.max(delta or 0, 0)
    if not done then
      tries = tries + 1
      done = startPath(tries >= startDelay)
      return
    end
    do -- W9 safe teleport exit setting (independent of the vehicle path)
      local okP0, pl0 = pcall(Game.GetPlayer)
      if okP0 and pl0 ~= nil then pcall(M.PushSafeTeleportExit, clock, pl0, sink) else M.SafeTeleportExitPlayerGone() end
      if okP0 and pl0 ~= nil then pcall(M.PushApproachWalk, clock, pl0, sink) else M.ApproachWalkPlayerGone() end
    end
    if not wiring.own then return end
    local okP, player = pcall(Game.GetPlayer)
    if not okP or player == nil then return end
    -- controller -> pose -> utpp_camera API commands -> perspective request -> trace
    local obs, vehEnt = observe(player)
    if vehEnt ~= nil then wiring.joVeh = vehEnt end -- the car entity of the ride (jo diag reads its live pose after the unmount)
    if wiring.camDiag ~= nil then
      sink(string.format("[utpp_vehiclecam] t=%.2f camera read %s", clock, wiring.camDiag))
      wiring.camDiag = nil
    end
    if not obs.mounted then
      wiring.camDiagDone, wiring.reqDiagDone = false, false
      -- the class stays known through the exit hold / release after the unmount (bike poses)
      if ctrl.keep == "off" and ctrl.exitPull == nil then ctrl.vehBlend, ctrl.vehBike, ctrl.vehCar, ctrl.vehRecKey = nil, nil, nil, nil end
    end
    -- review: utpp_camera limits the boom pull-in during a mount hold only for a positively identified car
    if type(wiring.cam.SetVehicleCar) == "function" then pcall(wiring.cam.SetVehicleCar, ctrl.vehCar == true) end
    local key = string.format("state=%s mounted=%s driver=%s active=%s persistent=%s keep=%s phase=%s",
      tostring(obs.vehicleState), tostring(obs.mounted), tostring(obs.driver), tostring(obs.activePersp),
      tostring(obs.persistentPersp), tostring(ctrl.keep), tostring(ctrl.phase))
    if key ~= lastObsKey then
      lastObsKey = key
      sink(string.format("[utpp_vehiclecam] t=%.2f obs %s", clock, key))
    end
    local okS, p = pcall(ctrl.Step, clock, obs)
    if not okS then sink("[utpp_vehiclecam] Step error: " .. tostring(p)) end
    pcall(updatePullTarget, player)
    local okC, errC = pcall(function()
      local cam = wiring.cam
      local okO, orbit = pcall(cam.OrbitPose)
      local pose = M.OwnPose(ctrl, ctrl.pullFrame, okO and orbit or nil)
      M.RunOwnCmds(ctrl, cam, pose)
      if cam.VehicleHeld() then
        if pose then cam.SetPoseOverride(pose.pos, pose.yaw, pose.pitch, "utpp_vehiclecam", pose.fov, pose.keepYaw)
        else cam.SetPoseOverride(nil, nil, nil, "utpp_vehiclecam") end
      end
    end)
    if not okC and not wiring.camCmdErrLogged then
      wiring.camCmdErrLogged = true
      sink("[utpp_vehiclecam] utppCamera command error: " .. tostring(errC))
    end
    if okS and p ~= nil then request(player, p) end
    pcall(traceCam)
    pcall(joDiag, player, obs)
    if ctrl.joPush ~= nil then -- jump-out side push: once per native launch, with the live car / V poses
      local okJ, errJ = pcall(function()
        local okS, s = pcall(joDiagSample, player)
        M.JoPushRun(ctrl, clock, okS and s or nil, sink)
      end)
      if not okJ then
        ctrl.joPush = nil
        sink("[utpp_vehiclecam] jump-out side push error: " .. tostring(errJ))
      end
    end
    local okD, errD = pcall(doorProbe)
    if not okD and not wiring.doorErrLogged then
      wiring.doorErrLogged = true
      sink("[utpp_vehiclecam] door probe FAILED: " .. tostring(errD))
    end
  end)
end

return M

-- UTPP vehicle path on utppCamera (2026-10-07): utpp_camera 0.1.2 vehicle API + utpp_vehiclecam
-- (own mode), scripted mount / ride / exit timelines with the REAL utpp_camera (camera_math) and utpp_vehiclecam modules
-- loaded in a CET-like mocked game (registerForEvent per module, CET update order utpp_camera -> utpp_vehiclecam),
-- utpp_perspective / utpp_body stubs (TPP on foot and while entering, FPP "vehicle" from Driving, TPP from the exit
-- start - as utpp_perspective 0.2.x) and a camera stack model: the rendered camera is the active high-priority one, else
-- the latest activated; a change of the rendered camera blends over the Activate / Deactivate blend of the call that
-- caused it. The vehicle accepts a TPP request 0.29 s after it (in Driving), activates its camera and shows its own
-- 0.449 s FPP -> TPPFar transition; at the exit start it switches to FPP (0.45 s transition back to the eye).
-- Scenarios: car, short ride (exit inside the handover release window), exit before the handover, motorcycle, handover
-- pose match (readable vehicleTPPCamera), vehicle camera switched to FPP in the ride, passenger, camera delegate
-- (teleport into the seat), negative control SetEnabled(false) (old FPP handoff).
-- Camera round 3: 4b) jump-out (exit while driving 12 m/s, V launched sideways): the exit hold follows V (orbit pose
-- around V, pitch kept) instead of staying at the vehicle; negative control SetExitFollow(false) = V out of view.
-- Globals from the runner: UTPP_CAMERA_PATH, UTPP_CAMERA_DIR, UTPP_CAMERA_MATH_PATH, UTPP_VEHICLECAM_PATH.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail ~= nil and (" -- " .. tostring(detail)) or ""))
end
-- A/B setter of the module under test; a setter that does not exist (the base code) is skipped, the checks then show the base
local function ab(v, name, ...) local f = v[name]; if f then return f(...) end end
local CM = dofile(UTPP_CAMERA_MATH_PATH)
package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. package.path

local FPS = 60
local DT = 1 / FPS
local function smooth(t) t = math.max(0, math.min(1, t)); return t * t * (3 - 2 * t) end
local function lerp(a, b, w) return a + (b - a) * w end
local function lerp3(a, b, w) return CM.v3(lerp(a.x, b.x, w), lerp(a.y, b.y, w), lerp(a.z, b.z, w)) end
local function rot(x, y, deg)
  local r = math.rad(deg)
  return math.cos(r) * x - math.sin(r) * y, math.sin(r) * x + math.cos(r) * y
end
local function rowsMatrix(pos, fwd)
  local f = CM.norm(fwd)
  local r = CM.norm(CM.v3(f.y, -f.x, 0))
  local u = CM.v3(r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x)
  return { X = { x = r.x, y = r.y, z = r.z, w = 0 }, Y = { x = f.x, y = f.y, z = f.z, w = 0 },
           Z = { x = u.x, y = u.y, z = u.z, w = 0 }, W = { x = pos.x, y = pos.y, z = pos.z, w = 1 } }
end
local PERSP = { FPP = 0, TPPClose = 1, TPPMedium = 2, TPPFar = 3 }
-- vehicle TPPFar camera in the vehicle frame (x right, y forward, z up) - own model values
local FAR = { car = { y = -6.8, z = 2.6, pitch = -12 }, bike = { y = -4.6, z = 1.9, pitch = -10 } }
local BOX = { car = { 1.0, 2.4, 1.5 }, bike = { 0.45, 1.1, 1.2 } } -- half extents x / y, height

-- ================================================================ mocked game
local function makeWorld(opts)
  opts = opts or {}
  local W = { t = 0, lines = {}, logged = {}, tppCalls = {}, hg = {}, handlers = { cam = {}, vc = {} }, cur = nil, mods = {},
              deacts = {}, acts = {}, fppActs = {}, reqs = {}, rec = {}, switches = {}, body = {}, opts = opts,
              p = { pos = CM.v3(-6, 0.3, 0), yaw = -90 }, mounted = false, vs = 0, driver = opts.driver ~= false,
              v = { pos = CM.v3(0, 0, 0), yaw = 0, bike = opts.bike or false, active = 0, persistent = 3, acceptAt = nil,
                    outAt = nil, pending = nil } }
  local kind = W.v.bike and "bike" or "car"
  W.kind = kind
  local function playerQ() return CM.qFromYawPitch(W.p.yaw, 0) end
  local function eye() return CM.add(W.p.pos, CM.qrot(playerQ(), CM.v3(0, 0.05, 1.62))) end
  W.eye = eye
  local function vehToWorld(x, y, z)
    local wx, wy = rot(x, y, W.v.yaw)
    return CM.v3(W.v.pos.x + wx, W.v.pos.y + wy, W.v.pos.z + z)
  end
  W.vehToWorld = vehToWorld
  function W.worldToVeh(p)
    local x, y = rot(p.x - W.v.pos.x, p.y - W.v.pos.y, -W.v.yaw)
    return CM.v3(x, y, p.z - W.v.pos.z)
  end
  local function vehFar()
    local f = opts.far or FAR[kind] -- vehicle batch: opts.far = a scenario's vehicle TPP camera
    return vehToWorld(f.x or 0, f.y, f.z), CM.forward(W.v.yaw + (f.yaw or 0), f.pitch) -- (r13e: optional f.yaw, vehicle relative)
  end
  local function vehCamPose()
    local fp, ff = vehFar()
    local ep, ef = eye(), CM.forward(W.p.yaw, 0)
    if W.v.outAt then
      local s = smooth((W.t - W.v.outAt) / 0.45)
      return lerp3(fp, ep, s), CM.norm(lerp3(ff, ef, s))
    end
    if W.v.acceptAt and W.t - W.v.acceptAt < 0.449 then
      local s = smooth((W.t - W.v.acceptAt) / 0.449)
      return lerp3(ep, fp, s), CM.norm(lerp3(ef, ff, s))
    end
    return fp, ff
  end
  -- utppCamera component (player root space)
  local utpp = { lp = CM.v3(0, 0, 0), lq = CM.q(0, 0, 0, 1), fov = 60, writes = 0 }
  W.utpp = utpp
  local function utppWorld()
    return CM.add(W.p.pos, CM.qrot(playerQ(), utpp.lp)), CM.qmul(playerQ(), utpp.lq)
  end
  W.utppWorld = utppWorld
  -- camera stack
  local S = { seq = 1, winner = "fpp", blend = nil, lastPose = nil, lastFwd = nil, pending = nil,
              cams = { fpp = { active = true, seq = 1, prio = false }, utpp = { active = false, seq = 0, prio = false },
                       veh = { active = false, seq = 0, prio = false },
                       wc = { active = false, seq = 0, prio = false } } } -- 2026-10-12 world camera entity
  W.S = S
  function S.activate(n, b) S.seq = S.seq + 1; S.cams[n].active, S.cams[n].seq = true, S.seq; S.pending = b or 0 end
  function S.deactivate(n, b) S.cams[n].active = false; S.pending = b or 0 end
  local function winnerName()
    local best, bp, bs = "fpp", false, -1
    for n, c in pairs(S.cams) do
      if c.active and ((c.prio and not bp) or (c.prio == bp and c.seq > bs)) then best, bp, bs = n, c.prio, c.seq end
    end
    return best
  end
  local function camPose(n)
    if n == "utpp" then local p, q = utppWorld(); return p, CM.qrot(q, CM.v3(0, 1, 0)) end
    if n == "veh" then
      -- batch 6 (opts.vehCamStatic): the vehicle camera renders at a world-fixed pose once its blend is over (female v1
      -- test 22:59:41.7: the camera stayed in the world while the car drove off)
      if opts.vehCamStatic and W.v.acceptAt ~= nil and W.t - W.v.acceptAt >= 0.449 then
        if W.v.staticPose == nil then local p, f = vehCamPose(); W.v.staticPose = { p = p, f = f } end
        return W.v.staticPose.p, W.v.staticPose.f
      end
      return vehCamPose()
    end
    if n == "wc" then return W.wc.wp, CM.qrot(W.wc.wq, CM.v3(0, 1, 0)) end -- world camera: its pose is world space
    return eye(), CM.forward(W.p.yaw, W.p.fppPitch or -5)
  end
  -- the UTPP camera (utppCamera component or the world camera entity) is reported as "utpp"
  local function shown(n) return n == "wc" and "utpp" or n end
  -- rendered FOV per camera (2026-11-01): the vehicle camera 69 (log 07-50-12 :507), the player cameras as set
  local function camFov(n)
    if n == "utpp" then return utpp.fov end
    if n == "wc" then return W.wc.fov end
    if n == "veh" then return opts.vehFov or 69 end
    return 80
  end
  function S.render()
    local w = winnerName()
    local pos, fwd = camPose(w)
    local fov = camFov(w)
    if w ~= S.winner then
      W.switches[#W.switches + 1] = { t = W.t, from = shown(S.winner), to = shown(w), dur = S.pending or 0 }
      -- 2026-11-01: the camera blended away from is evaluated live (the engine keeps it running during the blend; the
      -- deactivated utppCamera and the vehicle camera move with the vehicle); a switch during a running blend, or
      -- opts.frozenBlend, starts from the shown pose of the previous frame as before
      local live = S.blend == nil and not opts.frozenBlend
      S.blend = S.lastPose and { from = S.lastPose, fromF = S.lastFwd, fromFov = S.lastFov, fromName = live and S.winner or nil,
                                 at = W.t, dur = S.pending or 0 } or nil
      S.winner, S.pending = w, nil
    end
    if S.blend then
      local k = S.blend.dur > 0 and (W.t - S.blend.at) / S.blend.dur or 1
      if k < 1 then
        local s = smooth(k)
        local from, fromF, fromFov = S.blend.from, S.blend.fromF, S.blend.fromFov
        if S.blend.fromName then
          local p2, f2 = camPose(S.blend.fromName)
          from, fromF, fromFov = p2, f2, camFov(S.blend.fromName)
        end
        pos, fwd = lerp3(from, pos, s), CM.norm(lerp3(fromF, fwd, s))
        fov = lerp(fromFov or fov, fov, s)
      else S.blend = nil end
    end
    S.lastPose, S.lastFwd, S.lastFov = pos, fwd, fov
    return pos, fwd, shown(w), w, fov
  end
  function utpp:Activate(b) W.acts[#W.acts + 1] = { t = W.t, b = b }; S.activate("utpp", b) end
  function utpp:Deactivate(b)
    W.deacts[#W.deacts + 1] = { t = W.t, b = b, vehActive = W.v.active, acceptAt = W.v.acceptAt, wasActive = S.cams.utpp.active }
    S.deactivate("utpp", b)
  end
  function utpp:SetIsHighPriority(on) S.cams.utpp.prio = on and true or false end
  function utpp:IsHighPriority() return S.cams.utpp.prio end
  function utpp:SetLocalTransform(p, q) self.lp = CM.v3(p.x, p.y, p.z); self.lq = CM.q(q.i, q.j, q.k, q.r); self.writes = self.writes + 1 end
  function utpp:SetFOV(f) self.fov = f end
  function utpp:GetLocalToWorld() local p, q = utppWorld(); return rowsMatrix(p, CM.qrot(q, CM.v3(0, 1, 0))) end
  -- 2026-10-12 world camera: Codeware StaticEntitySystem + the spawned entity (world origin, identity: its camera
  -- component's local transform is the world pose); the entity resolves at once (opts.noWcam: setting off)
  local wc = { wp = CM.v3(0, 0, 0), wq = CM.q(0, 0, 0, 1), fov = 60, writes = 0, activations = 0 }
  W.wc = wc
  function wc:Activate(b) self.activations = self.activations + 1; W.acts[#W.acts + 1] = { t = W.t, b = b, wc = true }; S.activate("wc", b) end
  function wc:Deactivate(b)
    W.deacts[#W.deacts + 1] = { t = W.t, b = b, vehActive = W.v.active, acceptAt = W.v.acceptAt, wasActive = S.cams.wc.active, wc = true }
    S.deactivate("wc", b)
  end
  function wc:SetIsHighPriority(on) S.cams.wc.prio = on and true or false end
  function wc:SetLocalTransform(p, q) self.wp = CM.v3(p.x, p.y, p.z); self.wq = CM.q(q.i, q.j, q.k, q.r); self.writes = self.writes + 1 end
  function wc:SetFOV(f) self.fov = f end
  function wc:GetLocalToWorld() return rowsMatrix(self.wp, CM.qrot(self.wq, CM.v3(0, 1, 0))) end
  local wcEnt = {}
  function wcEnt:FindComponentByName(n) if n == "utppCamera" then return wc end return nil end
  local ses = { spawns = 0, despawns = 0, id = nil }
  W.ses = ses
  function ses:IsReady() return true end
  function ses:DespawnTagged() end
  function ses:SpawnEntity(spec) self.spawns = self.spawns + 1; self.spec = spec; self.id = { hash = 9001 }; return self.id end
  function ses:GetEntity(id) if self.id and id and id.hash == self.id.hash then return wcEnt end return nil end
  function ses:DespawnEntity() self.despawns = self.despawns + 1; self.id = nil; return true end
  local fpp = { headingLocked = false }
  function fpp:Activate(b) W.fppActs[#W.fppActs + 1] = { t = W.t }; S.activate("fpp", b) end
  function fpp:Deactivate() end
  function fpp:GetLocalToWorld() return rowsMatrix(eye(), CM.forward(W.p.yaw, W.p.fppPitch or -5)) end
  local vtpp = {}
  function vtpp:GetLocalToWorld()
    if opts.vtppReal then local p, f = vehFar(); return rowsMatrix(p, f) end
    return rowsMatrix(vehToWorld(-0.33, -0.30, 0.16), CM.forward(W.v.yaw, 0)) -- at the seat, as read in game (0.5.9 log)
  end
  local player = {}
  W.playerObj = player
  function player:GetEntityID() return { hash = 4242 } end
  function player:FindComponentByName(n)
    if n == "utppCamera" then return utpp end
    if n == "camera" then return fpp end
    if n == "vehicleTPPCamera" then return vtpp end
    return nil
  end
  function player:GetWorldPosition() return { x = W.p.pos.x, y = W.p.pos.y, z = W.p.pos.z, w = 1 } end
  function player:GetWorldOrientation() return playerQ() end
  function player:GetWorldForward() local f = CM.forward(W.p.yaw, 0); return { x = f.x, y = f.y, z = f.z, w = 0 } end
  function player:GetWorldYaw() return W.p.yaw end
  function player:GetVelocity() return { x = W.p.vel and W.p.vel.x or 0, y = W.p.vel and W.p.vel.y or 0, z = 0 } end
  function player:QueueEvent(e)
    if e.className == "vehicleRequestCameraPerspectiveEvent" then
      local p = PERSP[e.cameraPerspective] or tonumber(e.cameraPerspective)
      W.reqs[#W.reqs + 1] = { t = W.t, p = p }
      if W.v.pending == nil then W.v.pending = { t = W.t, p = p } end
    end
  end
  local mgr = {}
  function mgr:GetActivePerspective() return W.v.active end
  function mgr:GetPersistentPerspective() return W.v.persistent end
  function mgr:IsTPPActive() return W.v.active ~= 0 end
  local veh = {}
  function veh:GetCameraManager() return mgr end
  function veh:IsA(cls)
    if opts.isaFails then error("IsA failed") end -- the class cannot be read: neither car nor bike is known
    return W.v.bike and cls == "vehicleBikeBaseObject"
  end
  function veh:GetRecord()
    return { CameraManagerParams = function()
      return { GetID = function() return W.v.bike and "Camera.VehicleCameraManager_2w_Default" or "Camera.VehicleCameraManager_Default" end }
    end }
  end
  function veh:GetRecordID() return W.v.bike and "Vehicle.v_mock_bike" or (opts.recordId or "Vehicle.v_mock_car") end
  function veh:GetWorldPosition() return { x = W.v.pos.x, y = W.v.pos.y, z = W.v.pos.z, w = 1 } end
  function veh:GetWorldYaw() return W.v.yaw end
  function veh:GetGame() return "game" end -- argument of VehicleComponent.IsSlotAvailable (driver seat probe)
  local function bb(src) return { GetInt = function(_, k) return src[k] or 0 end, GetBool = function(_, k) return src[k] == true end } end
  local camSys = {}
  function camSys:GetActiveCameraWorldTransform(xf)
    local p = S.render()
    if opts.cameraReturnForm then
      if xf then error("out-parameter unsupported") end
      local tr = { position = { x = p.x, y = p.y, z = p.z } }
      if opts.cameraReturnForm == "both" then
        return { position = { x = -0.33, y = -0.30, z = 0.16 } }, tr
      end
      if opts.cameraReturnForm == "second" then return true, tr end
      return tr
    end
    if xf then xf.position = { x = p.x, y = p.y, z = p.z } end
    return true
  end
  function camSys:GetActiveCameraForward()
    local _, f = S.render()
    if opts.canonFwd then return { x = 0, y = 1, z = 0 } end -- the canonical forward the game reads at times (log 07-50-12 :507)
    return { x = f.x, y = f.y, z = f.z }
  end
  function camSys:GetActiveCameraFOV() local _, _, _, _, fov = S.render(); return fov end
  local sqs = {}
  function sqs:SyncRaycastByCollisionGroup() return false, { position = { x = 0, y = 0, z = 0 } } end
  -- utpp_perspective / utpp_body stubs
  W.persp = {
    Desired = function()
      local fppNow = W.vs == 1 or W.vs == 2 or W.vs == 3 or W.vs == 5 or W.vs == 6 or W.vs == 7
      -- opts.perspExitDelay (vehicle batch scenarios with the view left to the game): utpp_perspective's vehicle rule
      -- keeps its FPP decision exit = 0.4 s after the seat state ends (log: "TPP <- user" about 0.4 s after the exit start)
      if fppNow then W.lastFppVsAt = W.t
      elseif opts.perspExitDelay and W.lastFppVsAt and W.t - W.lastFppVsAt < opts.perspExitDelay then fppNow = true end
      if opts.fppUser then return { tpp = false, perspective = "FPP", reason = fppNow and "vehicle" or "user", style = "instant",
                                    mode = "active", wantsTpp = false } end
      return { tpp = not fppNow, perspective = fppNow and "FPP" or "TPP", reason = fppNow and "vehicle" or "user",
               style = "instant", mode = "active", wantsTpp = true }
    end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" }, -- CAM-9 scenario: aiming in third person
  }
  W.bodyMod = { Enabled = function() return true end,
                SetTppBody = function(on, why) W.body[#W.body + 1] = { t = W.t, on = on }; return true end }
  W.env = {
    registerForEvent = function(ev, fn) W.handlers[W.cur][ev] = fn end,
    registerInput = function() end,
    Observe = function() end,
    GetMod = function(n)
      if n == "utpp_perspective" then return W.persp end
      if n == "utpp_body" then return W.bodyMod end
      if n == "utpp_camera" then return W.mods.cam end
      if n == "utpp_vehiclecam" then return W.mods.vc end
      return nil
    end,
    Game = {
      GetPlayer = function() return player end,
      GetCameraSystem = function() return camSys end,
      GetStaticEntitySystem = function() return ses end,
      GetSpatialQueriesSystem = function() return sqs end,
      GetTransactionSystem = function()
        return { GetItemInSlot = function(_, _, slot)
          if slot == "AttachmentSlots.WeaponRight" and W.weapon then return W.weapon end
          return nil
        end }
      end,
      GetStatsSystem = function() return { GetStatValue = function() return 48 end } end,
      GetBlackboardSystem = function()
        return { GetLocalInstanced = function() return bb({ Vehicle = W.vs, LocomotionDetailed = 1, UpperBody = W.upperBody or 0 }) end,
                 Get = function() return bb({}) end }
      end,
      GetPhotoModeSystem = function() return { IsPhotoModeActive = function() return false end } end,
      GetSystemRequestsHandler = function() return { IsGamePaused = function() return false end } end,
      GetMountedVehicle = function() return W.mounted and veh or nil end,
      GetMountingFacility = function()
        return { GetMountingInfoSingleWithObjects = function()
          return { slotId = { id = { value = W.driver and "seat_front_left" or "seat_front_right" } } }
        end }
      end,
      GetScriptableSystemsContainer = function() return { Get = function() return nil end } end,
      NameToString = function(n) return tostring(n) end,
      UTPPLogLine = function(l) W.logged[#W.logged + 1] = l end,
      UTPPSetTppOverride = function(on) W.tppCalls[#W.tppCalls + 1] = { t = W.t, on = on }; return true end,
      -- 2026-11-01 NP-3 (C2): HeadGuard TPP value writes, recorded with the mount state (no write on the vehicle path)
      UTPPHeadGuardSetValue = function(v) W.hg[#W.hg + 1] = { t = W.t, v = v, mounted = W.mounted }; return true end,
    },
    Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end },
    Quaternion = { new = function(i, j, k, r) return { i = i, j = j, k = k, r = r } end },
    Transform = { new = function() return {} end },
    StaticEntitySpec = { new = function() return {} end },
    TweakDBID = { new = function(s) return s end },
    TDBID = { ToStringDEBUG = function(t) return t end },
    TweakDB ={ GetFlat = function() return 0.449 end, SetFlat = function() return true end },
    NewObject = function(n) return { className = n } end,
    ItemID = { GetTDBID = function(id) return id.tdb end },
    TweakDBInterface = { GetWeaponItemRecord = function(t)
      if t ~= "Items.Preset_Lexington_Default" then return nil end
      return { ItemType = function() return { Type = function() return "Wea_Handgun" end } end }
    end },
    gamedataStatType = { AimFOV = "AimFOV" },
    AnimationControllerComponent = { ApplyFeature = function() end, PushEvent = function() end },
    ListenerAction = { GetName = function(a) return a.name end, GetValue = function(a) return a.value end },
    GetAllBlackboardDefs = function()
      return { PlayerStateMachine = { Vehicle = "Vehicle", IsInWorkspot = "IsInWorkspot", HighLevel = "HighLevel",
                                      IsControllingDevice = "IsControllingDevice", LocomotionDetailed = "LocomotionDetailed",
                                      UpperBody = "UpperBody", MeleeWeapon = "MeleeWeapon" },
               UI_System = { IsInMenu = "IsInMenu" }, UI_PlayerStats = { isReplacer = "isReplacer" },
               Braindance = { IsActive = "IsActive" } }
    end,
    GetSingleton = function(n)
      return { ToQuat = function(_, e) return CM.qFromYawPitch(e.yaw, e.pitch) end }
    end,
    EulerAngles = { new = function(r, p, y) return { roll = r, pitch = p, yaw = y } end },
    vehicleRequestCameraPerspectiveEvent = { new = function() return { className = "vehicleRequestCameraPerspectiveEvent" } end },
    Enum = { new = function(_, name) return name end },
    EnumInt = function(v) if type(v) == "number" then return v end error("not an enum") end,
    CName = { new = function(s) return s end },
    -- passenger-side entry inputs (vanilla PassangerSwitchSeatsCondition): opts.driverSeatFree=false = the driver seat is taken,
    -- so the passenger ride is a real passenger ride (no seat switch expected)
    VehicleComponent = { IsSlotAvailable = function() return opts.driverSeatFree ~= false end },
    print = function(...)
      local parts = {}
      for i = 1, select("#", ...) do parts[#parts + 1] = tostring(select(i, ...)) end
      W.lines[#W.lines + 1] = string.format("%7.3f ", W.t) .. table.concat(parts, " ")
    end,
  }
  -- vehicle side of the frame (before the mods): request acceptance, exit switch, vehicle camera life time
  function W.vehicleStep()
    local v = W.v
    -- opts.passengerFlicker: the vanilla PassengerEvents.OnEnter queues TPPClose and then FPP - the vehicle reports a TPP
    -- perspective for a moment (0.12-0.35 s after the passenger state began) before the FPP one lands
    if opts.passengerFlicker and W.vs == 3 then
      v.flickT0 = v.flickT0 or W.t
      local dt = W.t - v.flickT0
      if dt >= 0.12 and dt < 0.35 then v.active = 1 elseif v.active == 1 and dt >= 0.35 then v.active = 0 end
    end
    if v.pending and (W.vs == 1 or (opts.passengerAccept and W.vs == 3)) and v.active == 0 and W.t >= v.pending.t + 0.29 then
      v.active, v.acceptAt, v.outAt, v.pending = v.pending.p, W.t, nil, nil
      S.activate("veh", 0)
    end
    if v.outAt and W.t - v.outAt >= 0.45 and S.cams.veh.active then S.deactivate("veh", 0) end
  end
  function W.exitStart()
    W.vs = 4
    if W.v.active ~= 0 then W.v.active, W.v.outAt = 0, W.t end
    W.v.pending = nil
  end
  return W
end

local GLOBALS = { "registerForEvent", "registerInput", "Observe", "GetMod", "Game", "Vector4", "Quaternion", "Transform",
                  "TweakDBID", "TDBID", "TweakDB", "NewObject", "AnimationControllerComponent", "ListenerAction",
                  "GetAllBlackboardDefs", "GetSingleton", "EulerAngles", "vehicleRequestCameraPerspectiveEvent", "Enum",
                  "EnumInt", "CName", "print", "StaticEntitySpec", "ResRef", "EntityID", "exEntitySpawner", "WorldTransform",
                  "ItemID", "TweakDBInterface", "gamedataStatType", "VehicleComponent" }
local function withEnv(W, fn)
  local saved = {}
  for _, g in ipairs(GLOBALS) do saved[g] = _G[g]; _G[g] = W.env[g] end
  local ok, err = pcall(fn)
  for _, g in ipairs(GLOBALS) do _G[g] = saved[g] end
  if not ok then error(err, 0) end
end
local function load(W)
  withEnv(W, function()
    W.cur = "cam"; W.mods.cam = dofile(UTPP_CAMERA_PATH)
    if W.opts.noWcam then W.mods.cam.rt.worldCamera = false end -- 2026-10-12: component path (camera.worldCamera off)
    W.cur = "vc"; W.mods.vc = dofile(UTPP_VEHICLECAM_PATH)
    W.mods.vc.SetCamTrace(true) -- the release default is off; the camTrace assertions below need it
    W.cur = nil
    W.handlers.cam.onInit()
    W.handlers.vc.onInit()
  end)
  return W.mods.cam, W.mods.vc
end
local function count(W, needle)
  local n = 0
  for _, l in ipairs(W.lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end
local function firstAt(W, needle)
  for _, l in ipairs(W.lines) do if string.find(l, needle, 1, true) then return tonumber(l:sub(1, 7)) end end
  return nil
end
local function dump(W, needles, from)
  local out = {}
  for _, l in ipairs(W.lines) do
    local t = tonumber(l:sub(1, 7)) or 0
    if t >= (from or 0) then
      for _, n in ipairs(needles) do if string.find(l, n, 1, true) then out[#out + 1] = l; break end end
    end
  end
  return table.concat(out, "\n")
end

-- one frame: script -> vehicle -> utpp_camera.onUpdate -> utpp_vehiclecam.onUpdate -> rendered camera recorded
local function frame(W, script)
  -- W.stall = { at = s, dt = s }: one frame of dt s (a hitch) from the first frame at or after `at` (the script and the vehicle still step one frame)
  local dt = DT
  if W.stall and not W.stall.done and W.t + DT >= W.stall.at - 1e-9 then dt, W.stall.done = W.stall.dt, true end
  W.t = W.t + dt
  if script then script(W, W.t) end
  W.vehicleStep()
  withEnv(W, function()
    W.handlers.cam.onUpdate(dt)
    W.handlers.vc.onUpdate(dt)
  end)
  local pos, fwd, w, _, fov = W.S.render()
  W.S.pending = nil
  local cam = W.mods.cam
  -- upos = the pose utppCamera itself shows (the override), 2026-11-01
  W.rec[#W.rec + 1] = { t = W.t, pos = pos, fwd = fwd, win = w, fov = fov,
                        upos = (W.wc.activations > 0) and W.wc.wp or (W.utppWorld()),
                        prio = W.S.cams.utpp.prio or W.S.cams.wc.prio, sig = cam.tppSig.sent,
                        held = cam.vh and cam.vh.held, state = cam.T.state, vs = W.vs, mounted = W.mounted,
                        vpos = CM.v3(W.v.pos.x, W.v.pos.y, W.v.pos.z), vyaw = W.v.yaw, -- batch 6: vehicle pose of the frame
                        ppos = CM.v3(W.p.pos.x, W.p.pos.y, W.p.pos.z), -- r13e: V's position of the frame (step beyond V's own motion)
                        pz = cam.S and cam.S.pivot and cam.S.pivot.z, pbase = cam.S and cam.S.pivot and cam.S.pivot.base,
                        joOn = cam.S and cam.S.pivot and cam.S.pivot.joOn, piv = cam.G and cam.G.last and cam.G.last.pivot, -- r13e: pivot state (landing traces)
                        jp = cam.G and cam.G.jpShown or 0 } -- 2026-11-05: the jump-out landing framing tilt in the written pitch (cfg.jumpOutFrame)
end
local function run(W, untilT, script) while W.t < untilT - 1e-9 do frame(W, script) end end

-- metrics over [t0, t1]: max rendered position change over 50 ms (3 frames at 60 fps), max forward angle change
local function continuity(W, t0, t1)
  local maxD, atD, maxA, atA = 0, nil, 0, nil
  local R = W.rec
  for i = 4, #R do
    local a, b = R[i - 3], R[i]
    if a.t >= t0 and b.t <= t1 then
      local d = CM.dist(a.pos, b.pos)
      if d > maxD then maxD, atD = d, b.t end
      local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd))))))
      if ang > maxA then maxA, atA = ang, b.t end
    end
  end
  return maxD, atD, maxA, atA
end
local function vhs(cam) return cam.vh or { held = "n/a (no vehicle API)", prio = "n/a" } end
local function insideBox(W, p)
  local b = BOX[W.kind]
  local l = W.worldToVeh(p)
  return math.abs(l.x) < b[1] and math.abs(l.y) < b[2] and l.z > -0.2 and l.z < b[3]
end

-- ================================================================ scripted timeline (car / bike)
-- on foot until tm (walk to the door 4.5-6.5 s), mount tm .. td (1.9 s: door -> seat, yaw -90 -> 0), Driving from td,
-- the vehicle drives td+0.8 .. td+rideMove, exit start tx = td + ride, unmount tu = tx + exitDur, walk away after.
local function timeline(opts)
  opts = opts or {}
  local T = { tm = opts.tm or 7.0 } -- opts.tm: the walk-up time (low frame rates need more than 300 CET frames before the vehicle path starts)
  T.td = T.tm + (opts.mountDur or 1.9)
  T.tx = T.td + (opts.ride or 4.5)
  T.tu = T.tx + (opts.exitDur or 1.6)
  T.te = T.tu + (opts.after or 3.0)
  local bike = opts.bike
  local door = bike and CM.v3(-0.9, -0.2, 0) or CM.v3(-1.3, 0.3, 0)
  local seat = bike and CM.v3(0, -0.2, 0.15) or CM.v3(-0.35, 0.25, -0.3)
  local out = bike and CM.v3(-1.0, -0.4, 0) or CM.v3(-1.4, 0.4, 0)
  T.script = function(W, t)
    local v = W.v
    local function rel(p) return W.vehToWorld(p.x, p.y, p.z) end
    if t < T.tm then
      local s = math.max(0, math.min(1, (t - 4.5) / 2.0))
      W.p.pos = lerp3(CM.v3(-6, door.y, 0), rel(door), s)
      W.p.yaw, W.p.vel = -90, (s > 0 and s < 1) and CM.v3(2.35, 0, 0) or nil
    elseif t < T.td then
      W.mounted, W.vs, W.p.vel = true, opts.teleport and 1 or 4, nil
      local s = smooth((t - T.tm) / (T.td - T.tm))
      if opts.teleport then s = 1 end
      W.p.pos, W.p.yaw = lerp3(rel(door), rel(seat), s), lerp(-90, 0, s)
    elseif t < T.tx then
      -- vanilla: driver ride = PSM Vehicle state 1 (DriveEvents), passenger ride = state 3 (PassengerEvents); a passenger in
      -- state 1 with a passenger slot is not something the game produces (the seat is final only after the seat switch)
      W.vs = W.driver and 1 or 3
      if opts.userFppAt and t >= T.td + opts.userFppAt and not W.userFpp then
        W.userFpp = true
        if v.active ~= 0 then v.active, v.outAt = 0, t end
        v.persistent = 0
      end
      local speed = (t > T.td + 0.8 and t < T.td + (opts.rideMove or 3.5)) and 6 or 0
      v.pos = CM.v3(v.pos.x, v.pos.y + speed * DT, v.pos.z)
      W.p.pos, W.p.yaw = rel(seat), v.yaw
    elseif t < T.tu then
      if W.vs ~= 4 then W.exitStart() end
      local s = smooth((t - T.tx) / (T.tu - T.tx))
      W.p.pos, W.p.yaw = lerp3(rel(seat), rel(out), s), lerp(0, 90, s)
    else
      W.mounted, W.vs = false, 0
      local s = math.max(0, t - T.tu - 0.3)
      W.p.pos = CM.add(rel(out), CM.v3(-1.5 * math.min(s, 2.0), 0, 0))
      W.p.yaw, W.p.vel = 90, (s > 0 and s < 2) and CM.v3(-1.5, 0, 0) or nil
    end
  end
  return T
end

-- common run: load, 6.5 s on foot (CET patch retries 300 frames, the camera enters TPP), then the timeline
local function session(opts)
  opts = opts or {}
  local W = makeWorld(opts)
  local cam, vc = load(W)
  local T = timeline(opts)
  if opts.before then opts.before(W, cam, vc) end
  run(W, T.tm - 0.01, T.script)
  if opts.atMount then opts.atMount(W, cam, vc) end
  run(W, T.te, T.script)
  return W, cam, vc, T
end


-- ================================================================ 0) API guards (camera in first person)
do
  local W = makeWorld({ fppUser = true })
  local cam = load(W)
  run(W, 2.0)
  local ok1, why1, ok2, ok3
  if type(cam.HoldForVehicle) == "function" then
    withEnv(W, function()
      ok1, why1 = cam.HoldForVehicle(true, "test")
      ok2 = cam.SetPoseOverride({ x = 0, y = 0, z = 0 }, 0, 0, "test")
      ok3 = cam.TakeFromVehicle(0.2, "test")
    end)
  end
  check("API: hold refused while the camera is in first person (logged), no pose / take without a hold", ok1 == false
        and string.find(tostring(why1), "not in third person", 1, true) ~= nil and count(W, "vehicle hold refused (test)") == 1
        and ok2 == false and ok3 == false and #W.acts == 0, tostring(why1))
end

-- ================================================================ 1) car: mount + ride + exit on utppCamera
do
  local W, cam, vc, T = session({})
  local ctrl = vc.ctrl
  check("car: vehicle path on utppCamera (one line, own mode)", ctrl.own == true
        and count(W, "vehicle path on utppCamera (utpp_camera 0.1.6, delegate ok)") == 1,
        dump(W, { "vehicle path on" }))
  check("car: camera in TPP before the mount (utppCamera rendered, TPP signal on)", W.rec[math.floor(T.tm * FPS) - 5].win == "utpp"
        and W.rec[math.floor(T.tm * FPS) - 5].sig == true)
  local holdAt = firstAt(W, "vehicle hold ON")
  check("car: hold ON at the mount start (<= 2 frames)", holdAt ~= nil and holdAt >= T.tm - 1e-6 and holdAt <= T.tm + 2.5 * DT,
        tostring(holdAt))
  local fppAfter, epochLines = 0, 0
  for _, a in ipairs(W.fppActs) do if a.t >= T.tm then fppAfter = fppAfter + 1 end end
  for _, l in ipairs(W.lines) do
    if (tonumber(l:sub(1, 7)) or 0) >= T.tm and string.find(l, "exit epoch=", 1, true) then epochLines = epochLines + 1 end
  end
  check("car: no FPP handoff (no camera.Activate, no 'exit epoch' line) from the mount to the end", fppAfter == 0 and epochLines == 0,
        string.format("fpp activations %d, exit epoch lines %d", fppAfter, epochLines))
  check("car: the vehicle decision FPP is ignored while held (logged once)", count(W, "vehicle hold: decision FPP (vehicle) ignored") == 1)
  -- TPP signal stays on
  local sigOff, sigFrames = 0, 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t <= T.te then sigFrames = sigFrames + 1; if r.sig ~= true then sigOff = sigOff + 1 end end end
  local offCalls = 0
  for _, c in ipairs(W.tppCalls) do if c.t >= T.tm and c.on == false then offCalls = offCalls + 1 end end
  check("car: TPP signal stays ON through mount / ride / exit / release", sigOff == 0 and offCalls == 0 and sigFrames > 600,
        string.format("%d of %d frames off, %d OFF calls", sigOff, sigFrames, offCalls))
  -- pull-out
  local pullAt = firstAt(W, "pull-out start")
  check("car: pull-out starts 0.2-0.9 s after the mount, car pull time 0.90 s", pullAt and pullAt - T.tm >= 0.2 - 1e-6
        and pullAt - T.tm <= 0.9 + 2 * DT and count(W, "path around the vehicle, 0.90 s)") == 1, dump(W, { "pull-out" }))
  check("car: pull-out path frozen from the utppCamera orbit", count(W, "pull-out path: start frozen at") == 1
        and count(W, "(utppCamera orbit)") == 1)
  -- handover after the vehicle TPP camera and its blend
  local acceptAt = W.v.acceptAt or -1e9 -- (nil-safe for the negative control with the old modules)
  check("car: vehicle accepted TPPFar (request from Driving)", W.reqs[1] and W.reqs[1].p == 3 and W.reqs[1].t >= T.td - 1e-6
        and W.v.acceptAt ~= nil, tostring(W.reqs[1] and W.reqs[1].p))
  local early, deactsBeforeExit = 0, 0
  for _, d in ipairs(W.deacts) do
    if d.t < T.tx then
      deactsBeforeExit = deactsBeforeExit + 1
      if d.vehActive == 0 or d.acceptAt == nil or d.t - d.acceptAt < 0.449 - 1e-6 then early = early + 1 end
    end
  end
  check("car: utppCamera never deactivated before the vehicle TPP camera is active and blended (one hand-over)",
        early == 0 and deactsBeforeExit == 1, string.format("early %d, deactivations %d", early, deactsBeforeExit))
  local hd = W.deacts[1] or { t = -1, b = -1 }
  -- 2026-11-01 car entry glide: the stock mock's vehicle camera is not the measured one, so the class default is the glide
  -- target (a wrong-default case): Deactivate(handGlideDefaultBlend 0.45) after the 0.9 s glide that starts when the request
  -- phase is done (the vehicle accepted 0.29 s after the first request)
  check("car: hand-over = Deactivate(0.80) after the 1.05 s glide (1.0-1.6 s after the vehicle accepted)", hd
        and math.abs(hd.b - 0.80) < 1e-6 and hd.t - acceptAt >= 0.9 and hd.t - acceptAt <= 1.6,
        hd and string.format("%.3f s after, blend %.2f", hd.t - acceptAt, hd.b))
  local notUtpp, inBox = 0, 0
  for _, r in ipairs(W.rec) do
    if r.t >= T.tm and r.t < hd.t then
      if r.win ~= "utpp" then notUtpp = notUtpp + 1 end
      if insideBox(W, r.pos) then inBox = inBox + 1 end
    end
  end
  check("car: utppCamera rendered from the mount to the hand-over (vehicle camera transition hidden by the priority)",
        notUtpp == 0 and count(W, "SetVehiclePriority(true)=ok") >= 1, string.format("%d frames", notUtpp))
  check("car: camera never inside the vehicle during the mount", inBox == 0, string.format("%d frames", inBox))
  check("car: seat pose (vehicleTPPCamera at the seat) is no target: glide to the class default, once, no engine-blend fallback",
        count(W, "handover glide: default target (0.00,-9.00,2.50)") == 1 and count(W, "handover match:") == 0
        and count(W, "handover glide: no vehicle TPP pose") == 0 and count(W, "glide matched: default)") == 1,
        dump(W, { "handover" }))
  local vehRender = 0
  for _, r in ipairs(W.rec) do if r.t > hd.t + 0.5 and r.t < T.tx and r.win == "veh" then vehRender = vehRender + 1 end end
  check("car: the vehicle camera renders the ride", vehRender > 100, vehRender)
  -- exit
  local takeAt = firstAt(W, "vehicle take-over") or 1e9
  check("car: exit start: utppCamera takes the view at the matched vehicle camera pose", takeAt and takeAt - T.tx <= 2 * DT
        and count(W, "exit match: cam=") == 1 and count(W, "(active camera)") >= 1
        -- vehicle batch CAM-6 (2): earlyExitBlend 0.20 -> 0.30 s
        -- 2026-11-01: a car take from the rendered camera matched in the same frame is a cut (no mix with the vehicle
        -- camera that the native exit moves to FPP)
        and count(W, "SetPoseOverride=ok (matched active camera), TakeFromVehicle(0.00)=ok") == 1,
        dump(W, { "exit match", "take-over", "exit (" }, T.tx - 0.1))
  local deactsAfterExit, notUtppExit, noPrio = 0, 0, 0
  for _, d in ipairs(W.deacts) do if d.t >= T.tx then deactsAfterExit = deactsAfterExit + 1 end end
  local relAt = firstAt(W, "vehicle release to orbit over") or -1e9
  for _, r in ipairs(W.rec) do
    if r.t >= takeAt + DT and r.t <= T.te then
      if r.win ~= "utpp" then notUtppExit = notUtppExit + 1 end
      if r.t < relAt and not r.prio then noPrio = noPrio + 1 end
    end
  end
  check("car: exit: no Deactivate, utppCamera rendered (high priority) until the release", deactsAfterExit == 0 and notUtppExit == 0
        and noPrio == 0, string.format("deact %d, other camera %d, no prio %d", deactsAfterExit, notUtppExit, noPrio))
  check("car: exit hold until the unmount + 0.5 s, then release 0.80 s", relAt and relAt >= T.tu + 0.5 - DT - 1e-6
        and relAt <= T.tu + 0.5 + 2 * DT and count(W, "vehicle release to orbit over 0.80 s") == 1,
        string.format("release %.3f s after the unmount", (relAt or 0) - T.tu))
  -- exit pose fix: the unmount triggers the "foot" command exactly once -> utpp_camera.VehicleOnFoot(true) (the MM pose
  -- write is allowed again during the rest of the hold); the hold's end clears it (a new mount starts mounted)
  local footAt = firstAt(W, "VehicleOnFoot(true)=ok")
  check("car: unmount -> one 'foot' command, VehicleOnFoot(true) executed once at the unmount", count(W, "utppCamera foot (unmount)") == 1
        and count(W, "VehicleOnFoot(true)=ok") == 1 and count(W, "vehicle hold: on foot = true") == 1 and footAt
        and footAt >= T.tu - 1e-6 and footAt <= T.tu + 2 * DT, string.format("%s vs unmount %.3f", tostring(footAt), T.tu))
  check("car: the end of the hold clears onFoot", vhs(cam).onFoot == false, tostring(vhs(cam).onFoot))
  local behind = nil
  for _, r in ipairs(W.rec) do if r.t >= T.tu - 0.1 - DT / 2 and not behind then behind = W.worldToVeh(r.pos) end end
  check("car: vehicle-behind pose (0, -5.5, 2.4) held before the unmount", behind and math.abs(behind.x) < 0.05
        and math.abs(behind.y + 5.5) < 0.05 and math.abs(behind.z - 2.4) < 0.05,
        behind and string.format("(%.2f,%.2f,%.2f)", behind.x, behind.y, behind.z))
  local endAt = firstAt(W, "vehicle hold OFF (release to orbit done)")
  -- r13e: the release may take longer than the caller's 0.80 s (cfg.vehReleaseSpeed: 'vehicle release: blend X s instead of 0.80 s')
  local relDur = 0.8
  for _, l in ipairs(W.lines) do local d = l:match("vehicle release: blend ([%d%.]+) s instead of") if d then relDur = tonumber(d) end end
  check("car: release ends the hold in the orbit (priority off, TPP state)", endAt and endAt >= relAt + relDur - 1e-6
        and endAt <= relAt + relDur + 2 * DT and relDur >= 0.8 and relDur <= 1.8 and vhs(cam).held == false and vhs(cam).prio == false and cam.T.state == "tpp"
        and W.S.cams.utpp.prio == false and W.S.cams.wc.prio == false and ctrl.ownHeld == false, string.format("%s %s", tostring(endAt), (cam.VehicleStatus and cam.VehicleStatus() or "no VehicleStatus")))
  local maxD, atD, maxA, atA = continuity(W, T.tm - 0.5, T.te)
  check("car: continuous rendered camera, mount -> exit -> orbit (<= 0.8 m per 50 ms)", maxD <= 0.8,
        string.format("max %.3f m at %.2f s, max %.1f deg", maxD, atD or -1, maxA))
  check("car: no orientation snap (<= 25 deg per 50 ms)", maxA <= 25, string.format("%.1f deg at %.2f s", maxA, atA or -1))
  check("car: normal exit (vehicle standing, V within 2 m): no jump-out follow, vehicle-behind hold kept",
        count(W, "exit follow (") == 0 and count(W, "exit while the vehicle moves") == 0, dump(W, { "exit follow" }))
  do -- 2026-11-01 NP-3 (C2): the near-camera head hide never acts on the vehicle path (mount, hold, hand-over, exit, release)
    local bad, last = 0, nil
    for _, h in ipairs(W.hg) do
      if h.mounted or (h.t >= T.tm - 1e-6 and h.t <= (endAt or T.te) + 1.0) then bad = bad + 1 end
      last = h.v
    end
    check("car: C2 head hide: no HeadGuard write from the mount to 1 s after the hold, none mounted, ends at the TPP value 0",
      bad == 0 and (last == nil or last == 0) and count(W, "body clearance") == 0, string.format("%d writes, %d on the vehicle path", #W.hg, bad))
  end
  check("car: no errors (camera / vehiclecam)", cam.stats.errors == 0 and count(W, "Step error") == 0 and count(W, "command error") == 0
        and count(W, "FAILED") == 0, dump(W, { "error", "FAILED" }))
  local st = vc.Status()
  check("car: Status() shows the vehicle path fields", string.find(st, "own=true ownHeld=false ownMounts=1 ownHandovers=1", 1, true)
        ~= nil and string.find(st, "ownExits=1 ownTakes=1", 1, true) ~= nil, st)
  check("car: cam veh= trace with the utpp= field", count(W, " cam veh=(") > 20 and count(W, " utpp=(") > 20
        and count(W, " vtpp=(") > 20)
  print(string.format("  car: max %.3f m / 50 ms at %.2f s, %.1f deg; hand-over %.3f s after accept; release %.3f s after the unmount",
    maxD, atD or -1, maxA, hd.t - acceptAt, relAt - T.tu))
  -- 2026-10-12 world camera: the whole run on the separate camera entity (utppCamera component never activated)
  local compActs, wcActs = 0, 0
  for _, a in ipairs(W.acts) do if a.wc then wcActs = wcActs + 1 else compActs = compActs + 1 end end
  check("car (world camera): spawned once through the StaticEntitySystem, used for the entry, hand-over and take-over; "
        .. "utppCamera never activated", W.ses.spawns == 1 and count(W, "world camera: spawned Codeware StaticEntitySystem") == 1
        and count(W, "world camera: active (enter epoch=") == 1 and compActs == 0 and wcActs >= 2 and hd.wc == true
        and W.wc.writes > 300 and count(W, "world camera: FAILED") == 0,
        string.format("spawns %d, comp acts %d, wc acts %d, hand-over on wc %s, writes %d", W.ses.spawns, compActs, wcActs,
          tostring(hd.wc), W.wc.writes))
end

-- ================================================================ 1c) car with camera.worldCamera off: utppCamera path unchanged
do
  local W, cam, vc, T = session({ noWcam = true })
  local compActs, wcActs = 0, 0
  for _, a in ipairs(W.acts) do if a.wc then wcActs = wcActs + 1 else compActs = compActs + 1 end end
  local deactsBeforeExit = 0
  for _, d in ipairs(W.deacts) do if d.t < T.tx and not d.wc then deactsBeforeExit = deactsBeforeExit + 1 end end
  local notUtpp = 0
  local hd = W.deacts[1] or { t = 1e9 }
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t < hd.t and r.win ~= "utpp" then notUtpp = notUtpp + 1 end end
  local maxD = continuity(W, T.tm - 0.5, T.te)
  check("car (worldCamera off): no spawn, utppCamera entry / hand-over / take-over as before, hold ends in the orbit",
        W.ses.spawns == 0 and wcActs == 0 and compActs >= 2 and deactsBeforeExit == 1 and notUtpp == 0
        and vhs(cam).held == false and cam.T.state == "tpp" and maxD <= 0.8 and count(W, "FAILED") == 0
        and count(W, "world camera") == 0,
        string.format("spawns %d, wc acts %d, comp acts %d, deacts %d, other %d, maxD %.2f", W.ses.spawns, wcActs, compActs,
          deactsBeforeExit, notUtpp, maxD))
end

-- ================================================================ 2) short ride: exit inside the hand-over release window
do
  -- 2026-11-01: the hand-over follows the 1.05 s entry glide (about td + 1.37 s): the exit 0.33 s after it
  -- 2026-11-05: the pull-out of the mock car runs at pullMaxSpeed 6 m/s (7.3 m in 1.82 s; 1.56 s at 7 m/s) and the glide waits for
  -- its end (td + 0.70 s): the hand-over is at about td + 1.77 s, the exit 0.28 s after it (inside the 0.80 s blend + 0.10 s release window)
  local W, cam, vc, T = session({ ride = 2.05, rideMove = 0 })
  local hd = W.deacts[1]
  check("short: hand-over happened before the exit", hd ~= nil and hd.t < T.tx, hd and string.format("%.2f vs %.2f", hd.t, T.tx))
  check("short: exit start flushed the pending mount release", count(W, "exit start: pending mount release flushed") == 1,
        dump(W, { "flushed", "release done", "handover" }))
  local after = 0
  for _, d in ipairs(W.deacts) do if d.t >= T.tx then after = after + 1 end end
  check("short: no Deactivate after the exit start", after == 0, after)
  local maxD, atD = continuity(W, T.tm - 0.5, T.te)
  check("short: continuous (<= 0.8 m per 50 ms)", maxD <= 0.8, string.format("max %.3f m at %.2f s", maxD, atD or -1))
  local off = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.sig ~= true then off = off + 1 end end
  check("short: TPP signal on, hold released to the orbit at the end", off == 0 and vhs(cam).held == false and cam.T.state == "tpp")
  print(string.format("  short ride: exit %.2f s after the hand-over, max %.3f m / 50 ms", T.tx - (hd and hd.t or 0), maxD))
end

-- ================================================================ 3) exit before the hand-over (vehicle not in TPP yet)
do
  local W, cam, vc, T = session({ ride = 0.15, rideMove = 0 })
  check("before: no hand-over, no Deactivate", #W.deacts == 0 and W.v.acceptAt == nil, #W.deacts)
  check("before: exit without activation, blend from the shown pose", count(W, "exit (exit before the handover): utppCamera still shown") == 1
        and count(W, "vehicle take-over") == 0, dump(W, { "exit (" }))
  local notUtpp = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.win ~= "utpp" then notUtpp = notUtpp + 1 end end
  local maxD, atD = continuity(W, T.tm - 0.5, T.te)
  check("before: utppCamera rendered throughout, continuous", notUtpp == 0 and maxD <= 0.8,
        string.format("%d frames, max %.3f m at %.2f", notUtpp, maxD, atD or -1))
  check("before: hold released to the orbit", vhs(cam).held == false and cam.T.state == "tpp" and #W.fppActs == 0)
end

-- ================================================================ 4) motorcycle
do
  local W, cam, vc, T = session({ bike = true, mountDur = 1.6 })
  check("bike: pull-out over 1.30 s (motorcycle)", count(W, "1.30 s, motorcycle)") == 1, dump(W, { "pull-out start" }))
  local early = 0
  for _, d in ipairs(W.deacts) do if d.vehActive == 0 or d.t - (d.acceptAt or 1e9) < 0.449 - 1e-6 then early = early + 1 end end
  local maxD, atD, maxA = continuity(W, T.tm - 0.5, T.te)
  check("bike: hand-over after the vehicle camera, continuous (<= 0.8 m per 50 ms)", early == 0 and #W.deacts == 1 and maxD <= 0.8,
        string.format("early %d, deact %d, max %.3f m at %.2f, %.1f deg", early, #W.deacts, maxD, atD or -1, maxA))
  local relAt = firstAt(W, "vehicle release to orbit over")
  -- vehicle batch CAM-8: bikes hold bikeExitHoldAfterUnmount 0.2 s after the unmount (cars 0.5 s)
  check("bike: exit hold until the unmount + 0.2 s, back in the orbit, no FPP", relAt and math.abs(relAt - (T.tu + 0.2)) <= 2 * DT
        and vhs(cam).held == false and cam.T.state == "tpp" and #W.fppActs == 0)
  print(string.format("  bike: max %.3f m / 50 ms at %.2f s, %.1f deg", maxD, atD or -1, maxA))
end

-- ================================================================ 4b) jump-out: exit while driving 12 m/s (user report 7)
-- V is launched sideways out of the moving car (exit 0.45 s, unmount), keeps the car's momentum and lands ~1.5 s later;
-- the car drives on. Before (SetExitFollow(false)): the camera stayed at the vehicle-behind pose until the unmount + 0.5 s
-- and then needed 0.8 s to reach V (V far away / out of view).
-- ex (optional, 2026-11-04): { rise = m of V's climb during the exit animation (1.2 default; the game log has V about 1 m above the seat at the unmount),
-- world = makeWorld opts, tl = timeline opts merged over the default, userFppAt = s after the mount: the player's vehicle camera
-- goes first person (the exit then runs through the take-over from the game camera), before = fn(W, cam, vc) after the load,
-- speed = vehicle speed (default 12 m/s; 0 = standing vehicle), launchAfter = s after the unmount when the mock native counts a launch (nil = never) }
local function jumpOut(follow, lateral, vz0, grav, camCfg, ex)
  ex = ex or {}
  local W = makeWorld(ex.world or {})
  local cam, vc = load(W)
  for k, v in pairs(camCfg or {}) do cam.cfg[k] = v end -- utpp_camera config override (A/B)
  if follow == false then vc.SetExitFollow(false) end
  if ex.before then ex.before(W, cam, vc) end
  local tl = { ride = 3.0, exitDur = 0.45, after = 3.0 }
  for k, v in pairs(ex.tl or {}) do tl[k] = v end
  local T = timeline(tl)
  local base = T.script
  W.track = {}
  local speed = ex.speed or 12
  lateral = lateral or 5.0
  local script = function(W, t)
    local v = W.v
    local function rel(p) return W.vehToWorld(p.x, p.y, p.z) end
    local seat = CM.v3(-0.35, 0.25, -0.3)
    if t < T.td then
      base(W, t)
    elseif t < T.tx then
      W.vs, W.mounted = 1, true
      if ex.userFppAt and t >= T.td + ex.userFppAt and not W.userFpp then -- the user switched the vehicle camera to first person (as the timeline script does)
        W.userFpp = true
        if v.active ~= 0 then v.active, v.outAt = 0, t end
        v.persistent = 0
      end
      local sp = math.max(0, math.min(speed, speed * (t - T.td - 0.5)))
      v.pos = CM.v3(v.pos.x, v.pos.y + sp * DT, v.pos.z)
      W.p.pos, W.p.yaw, W.p.vel = rel(seat), v.yaw, CM.v3(0, sp, 0)
    elseif t < T.tu then
      if W.vs ~= 4 then W.exitStart() end
      v.pos = CM.v3(v.pos.x, v.pos.y + speed * DT, v.pos.z)
      local k = t - T.tx
      W.p.pos = CM.add(rel(seat), CM.v3(-lateral * k, 0, (ex.rise or 1.2) * k))
      W.p.yaw, W.p.vel = 0, CM.v3(-lateral, speed, 0)
    else
      W.mounted, W.vs = false, 0
      v.pos = CM.v3(v.pos.x, v.pos.y + speed * DT, v.pos.z)
      if not W.jump then W.jump = { vx = -lateral, vy = speed, vz = vz0 or 1.5 } end
      if ex.launchAfter and not W.joFired and t >= T.tu + ex.launchAfter then W.joFired, W.joEpoch = true, (W.joEpoch or 0) + 1 end -- the native counts the launch
      local j = W.jump
      local p = W.p.pos
      local z = math.max(0, p.z + j.vz * DT)
      j.vz = (z > 0) and (j.vz - (grav or 9.81) * DT) or 0
      W.p.pos = CM.v3(p.x + j.vx * DT, p.y + j.vy * DT, z)
      if z <= 0 then j.vx, j.vy = j.vx * (1 - 3 * DT), j.vy * (1 - 3 * DT) end
      local s2 = math.sqrt(j.vx * j.vx + j.vy * j.vy)
      W.p.yaw, W.p.vel = 0, s2 > 0.3 and CM.v3(j.vx, j.vy, 0) or nil
    end
    W.track[#W.track + 1] = { t = t, pos = CM.v3(W.p.pos.x, W.p.pos.y, W.p.pos.z) }
  end
  run(W, T.te, script)
  -- V in view: angle between the rendered camera forward and the direction to V's chest, camera distance to V
  local byT = {}
  for _, k in ipairs(W.track) do byT[string.format("%.4f", k.t)] = k.pos end
  local function view(t0, t1)
    local maxAng, maxDist, atAng = 0, 0, nil
    for _, r in ipairs(W.rec) do
      local pp = byT[string.format("%.4f", r.t)]
      if pp and r.t >= t0 and r.t <= t1 then
        local d = CM.sub(CM.add(pp, CM.v3(0, 0, 1.2)), r.pos)
        local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(d), CM.norm(r.fwd))))))
        if ang > maxAng then maxAng, atAng = ang, r.t end
        maxDist = math.max(maxDist, CM.len(d))
      end
    end
    return maxAng, maxDist, atAng
  end
  -- continuity relative to V (what the eye sees with V framed): change of (camera - V) over 50 ms
  local function rel(t0, t1)
    local R, maxD, atD = W.rec, 0, nil
    for i = 4, #R do
      local a, b = R[i - 3], R[i]
      local pa, pb = byT[string.format("%.4f", a.t)], byT[string.format("%.4f", b.t)]
      if pa and pb and a.t >= t0 and b.t <= t1 then
        local d = CM.dist(CM.sub(a.pos, pa), CM.sub(b.pos, pb))
        if d > maxD then maxD, atD = d, b.t end
      end
    end
    return maxD, atD
  end
  return W, cam, vc, T, view, rel
end
do
  local W, cam, vc, T, view, rel = jumpOut(true)
  local fAt = firstAt(W, "exit follow (")
  check("jump-out: vehicle speed seen at the exit (jump-out logged)", count(W, "exit while the vehicle moves 12.0 m/s") == 1,
        dump(W, { "exit while", "exit follow" }))
  check("jump-out: hold target switches to the orbit around V when V is > 2.0 m from the vehicle (before the unmount)",
        fAt and fAt > T.tx and fAt < T.tu and count(W, "exit follow (V ") == 1 and count(W, "exit follow (") == 1,
        dump(W, { "exit follow", "exit (" }, T.tx - 0.1))
  local ang, dist, atA = view(fAt + 0.5, T.te)
  check("jump-out: V stays in view after the 0.5 s follow blend (<= 25 deg off the view axis, <= 4.5 m)", ang <= 25 and dist <= 4.5,
        string.format("max %.1f deg at %.2f s, max %.2f m", ang, atA or -1, dist))
  local ang2, dist2 = view(T.tx, fAt + 0.5)
  check("jump-out: during the blend V stays near the view (<= 40 deg, <= 8 m)", ang2 <= 40 and dist2 <= 8,
        string.format("max %.1f deg, %.2f m", ang2, dist2))
  local maxD, atD, maxA, atA2 = continuity(W, T.tx - 0.2, T.te)
  local relD, relAtD = rel(T.tx - 0.2, T.te)
  check("jump-out: continuous relative to V (<= 0.8 m per 50 ms; absolute <= 1.6 m with V flying at 13 m/s), <= 25 deg",
        relD <= 0.8 and maxD <= 1.6 and maxA <= 25, string.format("relative %.3f m at %.2f s, absolute %.3f m at %.2f s, %.1f deg at %.2f s",
        relD, relAtD or -1, maxD, atD or -1, maxA, atA2 or -1))
  check("jump-out: release after the follow carries the offset with the orbit", count(W, "ReleaseToOrbit(0.80, carry)=ok") == 1
        and count(W, "offset carried with the orbit (moving V)") == 1, dump(W, { "ReleaseToOrbit" }))
  local relAt = firstAt(W, "vehicle release to orbit over")
  check("jump-out: hold / release timing unchanged (unmount + 0.5 s, release 0.8 s), back in the orbit, no FPP",
        relAt and math.abs(relAt - (T.tu + 0.5)) <= 2 * DT and vhs(cam).held == false and cam.T.state == "tpp"
        and #W.fppActs == 0 and count(W, "FAILED") == 0, tostring(relAt))
  check("jump-out: Status() counts the follow", string.find(vc.Status(), "exitFollow=true exitFollows=1", 1, true) ~= nil)
  print(string.format("  jump-out: follow %.2f s after the exit start; after the blend max %.1f deg / %.2f m; max %.3f m / 50 ms "
    .. "(relative to V %.3f m)", fAt - T.tx, ang, dist, maxD, relD))
  -- negative control: the old exit hold (vehicle-behind pose until the unmount + 0.5 s)
  local W2, _, _, T2, view2, rel2 = jumpOut(false)
  local fAt2 = T2.tx + (fAt - T.tx)
  local angN, distN = view2(fAt2 + 0.5, T2.te)
  check("jump-out negative control: SetExitFollow(false) -> V out of view / far from the camera (the reported behaviour)",
        count(W2, "exit follow (") == 0 and (angN > 25 or distN > 4.5), string.format("max %.1f deg, %.2f m", angN, distN))
  print(string.format("  jump-out negative control: max %.1f deg off the view axis, %.2f m from V, relative %.3f m / 50 ms", angN,
    distN, (rel2(T2.tx - 0.2, T2.te))))
  -- smaller sideways launch (V within 2 m until the unmount): the follow starts at the unmount (vehicle speed > 4 m/s)
  local W3, cam3, _, T3, view3, rel3 = jumpOut(true, 2.0)
  local fAt3 = firstAt(W3, "exit follow (unmount after a jump-out, vehicle 12.0 m/s")
  local ang3, dist3 = view3((fAt3 or T3.te) + 0.5, T3.te)
  local maxD3 = rel3(T3.tx - 0.2, T3.te)
  check("jump-out (V within 2 m until the unmount): follow at the unmount, V in view after the blend, continuous",
        fAt3 and math.abs(fAt3 - T3.tu) <= 2 * DT and count(W3, "exit follow (") == 1 and ang3 <= 25 and dist3 <= 4.5
        and maxD3 <= 0.8 and vhs(cam3).held == false,
        string.format("at %s, %.1f deg, %.2f m, %.3f m / 50 ms", tostring(fAt3), ang3, dist3, maxD3))
end

-- ================================================================ 5) readable vehicleTPPCamera: pose match before the hand-over
do
  for _, variant in ipairs({ "first", "second" }) do
    local Wr, _, vcr = session({ cameraReturnForm = variant })
    check("CET " .. variant .. " transform return: exit match uses rendered active camera, not seat fallback",
      (vcr.ctrl.stats.exitMatches or 0) > 0 and count(Wr, "exit match: cam=") > 0
      and count(Wr, "no game camera pose at exit start") == 0,
      dump(Wr, { "exit match:" }))
  end
  local Wb, _, vcb = session({ cameraReturnForm = "both" })
  check("CET two valid transforms: rendered second beats stale seat first at exit",
    (vcb.ctrl.stats.exitMatches or 0) > 0 and count(Wb, "exit match: cam=(0.00,-6.80,2.60)") > 0
    and count(Wb, "no game camera pose at exit start") == 0,
    dump(Wb, { "exit match:" }))
  local Wt = select(1, session({ cameraReturnForm = "first" }))
  check("CET return-only transform: camera trace retains valid vehicle-frame telemetry",
    count(Wt, "cam veh=") > 0 and count(Wt, "cam no active camera transform") == 0
    and count(Wt, "cam trace FAILED") == 0,
    dump(Wt, { " cam " }))
  local W, cam, vc, T = session({ vtppReal = true })
  -- 2026-11-01: a readable vehicle camera pose is the glide target of a car (handGlideTime 0.9 s, residual blend 0.20 s)
  local mAt = firstAt(W, "handover glide: read vehicleTPPCamera target")
  local hd = W.deacts[1]
  check("match: a readable vehicle camera pose is the glide target, hand-over 0.9 s later with Deactivate(0.20)", mAt and hd
        and hd.t - mAt >= 0.9 - 0.002 and hd.t - mAt <= 0.9 + 2 * DT and math.abs(hd.b - 0.20) < 1e-6
        and count(W, "handover match:") == 0, string.format("%s %s %s", tostring(mAt), tostring(hd and hd.t), tostring(hd and hd.b)))
  local maxD, atD = continuity(W, T.tm - 0.5, T.td + 2)
  check("match: continuous through the hand-over", maxD <= 0.8, string.format("max %.3f m at %.2f", maxD, atD or -1))
  -- the old 0.25 s match (bikes, glide off): the hand-over 0.25 s after the pose match
  local Wo = select(1, session({ vtppReal = true, before = function(_, _, v) ab(v, "SetHandGlide", false) end }))
  local moAt, hdo = firstAt(Wo, "handover match: utppCamera -> vehicleTPPCamera"), Wo.deacts[1]
  check("match: SetHandGlide(false) -> the previous 0.25 s match, hand-over 0.25 s later", moAt and hdo
        and hdo.t - moAt >= 0.25 - 1e-6 and hdo.t - moAt <= 0.25 + 2 * DT and count(Wo, "handover glide") == 0)
  local W2 = select(1, session({ vtppReal = true, before = function(_, _, v) v.SetOwnHandoverMatch(false); ab(v, "SetHandGlide", false) end }))
  check("match: A/B SetOwnHandoverMatch(false) + SetHandGlide(false) -> engine blend only", count(W2, "handover match:") == 0
        and count(W2, "engine blend") >= 1 and count(W2, "handover glide") == 0, dump(W2, { "glide" }))
  local W2b = select(1, session({ vtppReal = true, before = function(_, _, v) v.SetOwnHandoverMatch(false) end }))
  check("match: SetOwnHandoverMatch(false) alone -> no pose read, the glide goes to the class default",
        count(W2b, "handover glide: default target") == 1 and count(W2b, "handover glide: read") == 0)
end

-- ================================================================ 6) vehicle camera switched to FPP in the ride
do
  local W, cam, vc, T = session({ userFppAt = 2.0 })
  local gAt = firstAt(W, "view left to the game's vehicle camera")
  check("ride FPP: hold handed to the game camera ~0.3 s after the vehicle switched to FPP", gAt
        and gAt - (T.td + 2.0) >= 0.3 - 1e-6 and gAt - (T.td + 2.0) <= 0.3 + 3 * DT, dump(W, { "keepTpp", "hold OFF" }))
  local sigAt = nil
  for _, c in ipairs(W.tppCalls) do if c.t >= T.tm and c.on == false and not sigAt then sigAt = c.t end end
  local bodyOff = false
  for _, b in ipairs(W.body) do if b.t >= gAt - 1e-6 and b.on == false then bodyOff = true end end
  local fppDuringRide = 0
  for _, a in ipairs(W.fppActs) do if a.t < T.tx then fppDuringRide = fppDuringRide + 1 end end
  check("ride FPP: TPP signal OFF and body released, no FPP camera activation in the ride", sigAt and sigAt - gAt <= 2 * DT
        and bodyOff and fppDuringRide == 0, string.format("sig %s body %s fpp %d", tostring(sigAt), tostring(bodyOff), fppDuringRide))
  check("ride FPP: exit as before (no hold, camera re-enters TPP from the eyes after the exit)", count(W, "vehicle take-over") == 0
        and cam.T.state == "tpp" and vhs(cam).held == false)
end

-- ================================================================ 7) passenger: camera keeps its own rules (old behaviour)
do
  -- (2026-11-01: the car passenger now stays on utppCamera's chase pose; SetPassengerChase(false) is the previous behaviour)
  local W, cam, vc, T = session({ driver = false, driverSeatFree = false, before = function(_, _, v) ab(v, "SetPassengerChase", false) end })
  check("passenger: no hold, logged", (count(W, "passenger seat") >= 1 or count(W, "seat not final yet") >= 1)
        and count(W, "vehicle hold ON") == 0,
        string.format("passenger seat %d, seat not final %d, hold ON %d", count(W, "passenger seat"),
          count(W, "seat not final yet"), count(W, "vehicle hold ON")))
  check("passenger: FPP handoff as before (delegate declined)", count(W, "-> declined - first person as before") == 1
        and count(W, "exit epoch=") >= 1 and #W.fppActs >= 1)
end

-- ================================================================ 7b) mount in first person (user choice)
-- vehicle batch 2026-10-22: the rule "every entry starts in third person" (vehicle.forceTppOnEnter, default on)
-- also applies when V walks in first person: one vehicle TPP request, still no UTPP hold (the camera stays first person;
-- the game's vehicle TPP camera renders the ride). 7c: the setting off = the old behaviour (no request).
do
  local W, cam, vc, T = session({ fppUser = true })
  check("FPP user: no hold, one vehicle TPP request (always third person on entry), camera stays first person",
        count(W, "vehicle hold ON") == 0 and #W.reqs >= 1 and W.reqs[1].t >= T.td - 1e-6 and W.reqs[#W.reqs].t < T.tx
        and vc.ctrl.stats.forcedRides == 1 and vc.ctrl.stats.done == 1 and W.v.acceptAt ~= nil
        and count(W, "always third person on entry: driver ride without the UTPP hold") == 1
        and count(W, "mount while the camera is not held-ready") == 1 and cam.T.state == "fpp" and #W.acts == 0,
        dump(W, { "utpp_vehiclecam] t=" }))
end
do
  local W, cam, vc, T = session({ fppUser = true, before = function(_, _, v) v.ctrl.cfg.forceTppOnEnter = false end })
  check("FPP user, forceTppOnEnter off: no hold, no vehicle request, camera stays first person (old behaviour)",
        count(W, "vehicle hold ON") == 0 and #W.reqs == 0
        and count(W, "mount while the camera is not held-ready") == 1 and cam.T.state == "fpp" and #W.acts == 0,
        dump(W, { "utpp_vehiclecam] t=" }))
end

-- ================================================================ 8) camera delegate: teleport into the seat (Driving at once)
do
  local W, cam, vc, T = session({ teleport = true })
  check("delegate: the vehicle decision asked utpp_vehiclecam first -> held, no FPP handoff",
        count(W, "vehicle decision (utpp_perspective: vehicle) in third person: asked utpp_vehiclecam -> held (no FPP handoff)") == 1
        and count(W, "camera delegate: utpp_perspective: vehicle") >= 1 and #W.fppActs == 0, dump(W, { "delegate", "vehicle decision" }))
  local early = 0
  for _, d in ipairs(W.deacts) do if d.t < T.tx and (d.vehActive == 0 or d.t - (d.acceptAt or 1e9) < 0.449 - 1e-6) then early = early + 1 end end
  check("delegate: hand-over only after the vehicle TPP camera", early == 0 and W.v.acceptAt ~= nil)
  -- 2026-10-28: the teleport into the seat (0.95 m in one frame) starts the mount pivot blend; never inside the car
  local inBox = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t <= T.tm + 1.0 and insideBox(W, r.pos) then inBox = inBox + 1 end end
  check("delegate: teleport into the seat -> one mount pivot blend, camera never inside the car",
    count(W, "mount pivot blend start") == 1 and inBox == 0, string.format("%d frames, %d starts", inBox, count(W, "mount pivot blend start")))
end

-- ================================================================ 9) negative control: SetEnabled(false) = old FPP handoff
do
  local W, cam, vc, T = session({ before = function(_, _, v) v.SetEnabled(false) end })
  local fppAt = W.fppActs[1] and W.fppActs[1].t
  check("negative: FPP camera activated at the vehicle decision (old behaviour)", fppAt and math.abs(fppAt - T.td) <= 2 * DT
        and count(W, "exit epoch=") >= 1 and count(W, "vehicle hold ON") == 0, tostring(fppAt))
  local maxD, atD = continuity(W, T.tm - 0.5, T.td + 0.3)
  check("negative: the cut is a > 0.8 m jump per 50 ms (the near-FPP zoom of the log)", maxD > 0.8,
        string.format("max %.3f m at %.2f", maxD, atD or -1))
  local off = false
  for _, c in ipairs(W.tppCalls) do if c.t >= T.tm and c.on == false then off = true end end
  check("negative: TPP signal OFF in the vehicle", off)
  print(string.format("  negative control: max %.3f m / 50 ms at %.2f s (Driving %.2f s)", maxD, atD or -1, T.td))
end

-- ================================================================ 11) batch 6: ride watchdog (female v1 test 22:59:41.7-45)
-- The vehicle camera renders at a world-fixed pose after the hand-over while the car drives 6 m/s: one TPPFar request
-- again, then utppCamera takes the view back on the chase pose (retake 1/2, hand-over again), retake 2/2, then the chase
-- pose is kept for the ride; the exit starts from the shown camera. Off = the camera stays behind in the world.
do
  local W, cam, vc, T = session({ vehCamStatic = true, ride = 8.0, rideMove = 7.5 })
  local hd = W.deacts[1]
  check("watchdog: first hand-over as before (engine blend)", hd ~= nil and count(W, "engine blend from the pull-out pose") >= 1
        and count(W, "handover gated") == 0, dump(W, { "handover" }))
  check("watchdog: static camera while driving -> trip 1 requests TPPFar again, trips 2-3 retake (1/2, 2/2), then chase",
        count(W, "ride watchdog: active camera moved 0.00 m in 0.3") >= 4 and count(W, "TPPFar requested again (trip 1)") == 1
        and count(W, "utppCamera takes the view back (retake 1/2") == 1 and count(W, "utppCamera takes the view back (retake 2/2") == 1
        and count(W, "retakes exhausted (2) - utppCamera keeps the chase pose for this ride") == 1
        and count(W, "keepTpp ride -> mount (ride watchdog retake") == 2 and count(W, "keepTpp ride -> chase (ride watchdog: chase for the ride)") == 1,
        dump(W, { "ride watchdog", "keepTpp" }))
  check("watchdog: the retakes went through utpp_camera's take-over (pose override + Activate, high priority), hand-overs again",
        count(W, "utppCamera take (ride watchdog retake 1/2): SetPoseOverride=ok") == 1
        and count(W, "utppCamera take (ride watchdog retake 2/2): SetPoseOverride=ok") == 1
        and count(W, "utppCamera take (ride watchdog: chase for the ride): SetPoseOverride=ok") == 1
        and count(W, "vehicle hand-over (utpp_vehiclecam: handover)") == 3, dump(W, { "utppCamera take", "hand-over" }))
  -- after the final retake the rendered camera follows the vehicle: its vehicle-frame position is steady while driving
  local chaseAt = firstAt(W, "retakes exhausted") or 1e9
  -- vehicle-frame position of a recorded frame (the vehicle pose of THAT frame, not the final one)
  local function recVeh(r)
    local x, y = rot(r.pos.x - r.vpos.x, r.pos.y - r.vpos.y, -r.vyaw)
    return CM.v3(x, y, r.pos.z - r.vpos.z)
  end
  local maxDev, n, ref = 0, 0, nil
  for _, r in ipairs(W.rec) do
    if r.t >= chaseAt + 0.5 and r.t <= T.td + 7.4 then
      local l = recVeh(r)
      if ref == nil then ref = l else maxDev = math.max(maxDev, CM.dist(l, ref)) end
      n = n + 1
    end
  end
  check("watchdog: after the final retake the camera follows the vehicle (vehicle-frame position steady, behind the car)",
        n > 30 and maxDev < 0.3 and ref ~= nil and ref.y < -3 and ref.z > 1, string.format("dev %.2f over %d frames, ref (%.1f,%.1f,%.1f) chaseAt %.2f",
          maxDev, n, ref and ref.x or 0, ref and ref.y or 0, ref and ref.z or 0, chaseAt))
  check("watchdog: exit from the chase on the shown camera (no take-over), release to the orbit, hold ended, no errors",
        count(W, "exit (exit start (chase)): utppCamera still shown") == 1 and count(W, "vehicle release to orbit over") == 1
        and cam.T.state == "tpp" and vhs(cam).held == false and count(W, "Step error") == 0 and count(W, "command error") == 0
        and cam.stats.errors == 0, dump(W, { "exit (", "release to orbit", "error" }))
  check("watchdog: camW= / vspd= in the trace, Status() counters", count(W, " camW=(") > 20 and count(W, " vspd=") > 20
        and string.find(vc.Status(), "rideWatchdog=true wdTrips=4 wdRequests=1 rideRetakes=2 rideChases=1 handoverGate=false handGates=0", 1, true) ~= nil,
        vc.Status())
  -- negative control: watchdog off (old behaviour) -> the car drives off, the camera stays in the world
  local W2, cam2, vc2, T2 = session({ vehCamStatic = true, ride = 8.0, rideMove = 7.5,
                                      before = function(_, _, v) v.SetRideWatchdog(false) end })
  local far = 0
  for _, r in ipairs(W2.rec) do
    if r.t >= T2.td + 5.0 and r.t <= T2.td + 7.4 then
      local l = recVeh(r)
      if math.sqrt(l.x * l.x + l.y * l.y) > 9 then far = far + 1 end
    end
  end
  check("watchdog off: the camera stays world-fixed (> 9 m behind the car), no watchdog line (old behaviour)", far > 60
        and count(W2, "ride watchdog") == 0 and count(W2, "keepTpp ride -> mount") == 0, far)
  -- positive control: a vehicle camera that follows (default car) never trips the watchdog
  local W3, _, vc3 = session({ ride = 8.0, rideMove = 7.5 })
  check("watchdog: no trip when the vehicle camera follows the car", count(W3, "ride watchdog") == 0
        and string.find(vc3.Status(), "wdTrips=0 wdRequests=0 rideRetakes=0 rideChases=0", 1, true) ~= nil, vc3.Status())
  check("watchdog: setting vehicle.rideWatchdog default on", vc3.ctrl.cfg.rideWatchdog == true)
end

-- ================================================================ 12) batch 6: hand-over gate (A/B SetOwnHandoverGate(true), default off)
do
  local W, cam, vc, T = session({ before = function(_, _, v) v.SetOwnHandoverGate(true) end })
  local gAt = firstAt(W, "handover gated: no vehicle camera pose")
  local hd = W.deacts[1]
  check("gate: no readable pose -> utppCamera keeps the chase pose, pose read every 0.25 s, engine blend after 2 s",
        gAt ~= nil and hd ~= nil and hd.t - gAt >= 2.0 - 2 * DT and hd.t - gAt <= 2.0 + 0.3
        and count(W, "gate timed out after") == 1 and count(W, "engine blend from the pull-out pose (gate timed out") == 1
        and count(W, "keepTpp mount -> ride") == 1, dump(W, { "handover" }))
  local notUtpp = 0
  for _, r in ipairs(W.rec) do if r.t >= gAt and r.t < hd.t and r.win ~= "utpp" then notUtpp = notUtpp + 1 end end
  check("gate: utppCamera rendered on the chase pose while gated", notUtpp == 0, notUtpp)
  local W2 = select(1, session({ vtppReal = true, before = function(_, _, v) v.SetOwnHandoverGate(true) end }))
  check("gate: readable vehicle camera pose -> matched at once, never gated", count(W2, "handover gated") == 0
        and count(W2, "handover match: utppCamera -> vehicleTPPCamera") == 1)
  local W3 = select(1, session({}))
  check("gate: default off -> no gate line, the car glides to the class default", count(W3, "handover gated") == 0
        and count(W3, "handover glide: default target") == 1)
  local W4 = select(1, session({ before = function(_, _, v) ab(v, "SetHandGlide", false) end }))
  check("gate: SetHandGlide(false), default off -> the hand-over as before (no gate line, engine blend, Deactivate(0.45))",
        count(W4, "handover gated") == 0 and count(W4, "handover match: no vehicle camera pose") == 1
        and W4.deacts[1] ~= nil and math.abs(W4.deacts[1].b - 0.45) < 1e-6)
end

-- ================================================================ 13) vehicle batch 2026-10-22 (recording 14-57-06): session checks
-- with utpp_camera's vehicle API 2 (TakeForVehicleExit, game mode before the hand-over, own pitch, frozen release yaw,
-- framing band, soft first aim) and utpp_vehiclecam's keep "game" / exit / follow changes. Scenarios SC1-SC8 (camera owner's
-- list) plus CAM-5 T1, CAM-6 T3, CAM-10 T1.
local function sessionX(opts, extra)
  opts = opts or {}
  local W = makeWorld(opts)
  local cam, vc = load(W)
  local T = timeline(opts)
  local script = function(w, t) T.script(w, t); if extra then extra(w, t, T) end end
  if opts.before then opts.before(W, cam, vc) end
  run(W, T.tm - 0.01, script)
  if opts.atMount then opts.atMount(W, cam, vc) end
  run(W, T.te, script)
  return W, cam, vc, T
end
local function yawPitch(f)
  local n = CM.norm(f)
  return math.deg(math.atan2(-n.x, n.y)), math.deg(math.asin(math.max(-1, math.min(1, n.z))))
end
local function recAt(W, t)
  for _, r in ipairs(W.rec) do if r.t >= t - 1e-6 then return r end end
  return W.rec[#W.rec]
end
local function linesFrom(W, needle, t0)
  local n = 0
  for _, l in ipairs(W.lines) do
    local t = tonumber(l:sub(1, 7)) or 0
    if t >= (t0 or 0) - 1e-6 and string.find(l, needle, 1, true) then n = n + 1 end
  end
  return n
end
local function boxFrames(W, t0, t1)
  local n = 0
  for _, r in ipairs(W.rec) do if r.t >= t0 and r.t <= t1 and insideBox(W, r.pos) then n = n + 1 end end
  return n
end
local function frameJumps(W, t0, t1, lim)
  local n, at = 0, nil
  for i = 2, #W.rec do
    local a, b = W.rec[i - 1], W.rec[i]
    if b.t >= t0 and b.t <= t1 and CM.dist(a.pos, b.pos) > lim then n = n + 1; at = at or b.t end
  end
  return n, at
end

do -- SC1 CAM-1 T3: no hold (vehicle.keepTpp off), the vehicle camera is TPP at the vehicle decision -> no FPP camera
  local W, cam, vc, T = sessionX({ before = function(_, _, v) v.ctrl.cfg.keepTpp = false end,
                                   atMount = function(w) w.v.active = 3 end })
  local fpp = 0
  for _, a in ipairs(W.fppActs) do if a.t >= T.tm then fpp = fpp + 1 end end
  local sigOff = false
  for _, c in ipairs(W.tppCalls) do if c.t >= T.tm and c.t < T.tx and c.on == false then sigOff = true end end
  local deact = 0
  for _, d in ipairs(W.deacts) do if d.t >= T.td - 1e-6 then deact = deact + 1 end end
  check("SC1 CAM-1 T3: vehicle decision without a hold while the vehicle camera is TPP -> no FPP camera activation, "
    .. "TPP signal off, UTPP camera deactivated", fpp == 0 and count(W, "vehicle TPP camera renders (") >= 1 and sigOff
    and deact >= 1 and count(W, "vehicle hold ON") == 0, string.format("fpp %d, sigOff %s, deact %d", fpp, tostring(sigOff), deact))
end

do -- SC2 CAM-2 T3: seat state 7 with a TPP vehicle camera before the exit (keep game) -> exit taken from the game camera
  local function sc2(before)
    return sessionX({ before = before, perspExitDelay = 0.4 }, function(w, t, T)
      if t >= T.tx - 1.0 and t < T.tx then
        w.vs = 7
        if w.v.active ~= 0 then w.v.active = 5 end
      end
    end)
  end
  local W, cam, vc, T = sc2()
  local takeAt = firstAt(W, "vehicle exit take-over (")
  local maxD, atD, maxA, atA = continuity(W, T.tx + 0.05, T.tu + 1.5)
  local inBox = boxFrames(W, T.tx, T.tu + 1.0)
  check("SC2 CAM-2 T3: keep ride -> game (state 7) -> exit: one take-over from the game camera", takeAt ~= nil
    and math.abs(takeAt - T.tx) <= 2 * DT and count(W, "vehicle exit take-over (") == 1
    and count(W, "from the game camera") >= 1 and count(W, "keepTpp ride -> game (vehicle state 7)") == 1
    and count(W, "keepTpp game -> exit") == 1, dump(W, { "keepTpp", "take-over", "exit (" }, T.tx - 1.1))
  check("SC2 CAM-2 T3: exit + unmount never inside the vehicle, continuous (<= 0.8 m / 25 deg per 50 ms), no enter from "
    .. "the eyes, no FPP camera", inBox == 0 and maxD <= 0.8 and maxA <= 25 and linesFrom(W, "enter epoch=", T.tx) == 0
    and #W.fppActs == 0 and count(W, "vehicle hold: decision TPP (") <= 1,
    string.format("box %d, %.3f m at %.2f, %.1f deg at %.2f, enter %d", inBox, maxD, atD or -1, maxA, atA or -1,
      linesFrom(W, "enter epoch=", T.tx)))
  check("SC2: the hold ends in the orbit (third person)", vhs(cam).held == false and cam.T.state == "tpp")
  -- negative control: the take-over refused (as without TakeForVehicleExit) -> the camera re-enters from the eyes
  local Wn, _, _, Tn = sc2(function(_, c) c.TakeForVehicleExit = function() return false, "refused (negative control)" end end)
  local inBoxN = boxFrames(Wn, Tn.tx, Tn.tu + 1.0)
  check("SC2 negative control: take-over refused -> frames inside the vehicle (the reported behaviour)", inBoxN > 0
    and count(Wn, "camera refused the exit take-over") == 1, string.format("box %d", inBoxN))
end

do -- SC3 CAM-2 T4: exit from the vehicle FPP camera -> one clean cut to an outside pose, then continuous
  local W, cam, vc, T = session({ userFppAt = 2.0, perspExitDelay = 0.4 })
  local takeAt = firstAt(W, "vehicle exit take-over (") or 1e9
  local jumps, jAt = frameJumps(W, T.tx - 0.05, T.te, 0.8)
  local maxD, atD = continuity(W, takeAt + 2 * DT, T.te)
  local inBox = boxFrames(W, takeAt + DT, T.te)
  check("SC3 CAM-2 T4: vehicle FPP exit -> TakeForVehicleExit(0.00): exactly one cut at the take, then continuous, never "
    .. "inside the vehicle", count(W, "TakeForVehicleExit(0.00)=ok") == 1 and jumps == 1 and jAt and math.abs(jAt - takeAt) <= 2 * DT
    and maxD <= 0.8 and inBox == 0, string.format("take %.2f, jumps %d at %s, %.3f m at %.2f, box %d", takeAt, jumps,
    tostring(jAt), maxD, atD or -1, inBox))
end

do -- SC4 CAM-3 T3: the game's FPP pitch -60 from the exit start -> the release ends near -14, inside [-25, 5]
  local W, cam, vc, T = sessionX({}, function(w, t, T) if t >= T.tx then w.p.fppPitch = -60 end end)
  local relAt = firstAt(W, "vehicle release to orbit over")
  local endAt = firstAt(W, "vehicle hold OFF (release to orbit done)")
  local pEnd = endAt and select(2, yawPitch(recAt(W, endAt).fwd))
  local p1 = endAt and select(2, yawPitch(recAt(W, endAt + 1.0).fwd))
  local _, _, maxA = continuity(W, relAt or 0, (endAt or 0) + 0.1)
  check("SC4 CAM-3 T3: release end pitch within 11 deg of -14 and inside [-25, 5], no orientation snap, kept 1 s later "
    .. "while the game pitch stays -60", pEnd and math.abs(pEnd + 14) <= 11 and pEnd >= -25 and pEnd <= 5 and maxA <= 25
    and p1 and math.abs(p1 - pEnd) <= 1, string.format("end %s, +1 s %s, %.1f deg", tostring(pEnd), tostring(p1), maxA))
end

do -- SC5 CAM-3 T4 / CAM-4 T1: body turn 120 deg in 0.15 s at the mount start, game pitch 0 -> -30 at +0.55 s
  local W, cam, vc, T = sessionX({}, function(w, t, T)
    if t >= T.tm and t < T.td then
      w.p.yaw = w.p.yaw + 120 * math.min(1, (t - T.tm) / 0.15)
      w.p.fppPitch = (t >= T.tm + 0.55) and -30 or 0
    end
  end)
  local pullAt = firstAt(W, "pull-out start") or T.td
  local _, _, maxA, atA = continuity(W, T.tm, pullAt)
  local maxP = 0
  for i = 2, #W.rec do
    local a, b = W.rec[i - 1], W.rec[i]
    if b.t >= T.tm and b.t <= pullAt then
      local _, pa = yawPitch(a.fwd)
      local _, pb = yawPitch(b.fwd)
      maxP = math.max(maxP, math.abs(pb - pa))
    end
  end
  check("SC5 CAM-3 T4 / CAM-4 T1: mount start to the pull-out: <= 25 deg per 50 ms, pitch <= 2 deg per frame",
    maxA <= 25 and maxP <= 2, string.format("%.1f deg at %.2f, pitch %.2f deg/frame (pull-out %.2f)", maxA, atA or -1, maxP, pullAt))
end

do -- SC6 CAM-4 T2: V turned +150 deg from the vehicle at the unmount -> the release yaw moves <= 30 deg in total
  local W, cam, vc, T = sessionX({}, function(w, t, T) if t >= T.tu then w.p.yaw = w.v.yaw + 150 end end)
  local relAt = firstAt(W, "vehicle release to orbit over") or T.tu
  local y0 = yawPitch(recAt(W, relAt).fwd)
  local maxDev = 0
  for _, r in ipairs(W.rec) do
    if r.t >= relAt and r.t <= relAt + 0.8 then
      local y = yawPitch(r.fwd)
      local d = math.abs(((y - y0 + 180) % 360) - 180)
      maxDev = math.max(maxDev, d)
    end
  end
  local maxD, _, maxA = continuity(W, T.tm - 0.5, T.te)
  check("SC6 CAM-4 T2: release yaw change <= 30 deg over 0.8 s with V turned 150 deg, continuous", maxDev <= 30
    and maxD <= 0.8 and maxA <= 25, string.format("%.1f deg, %.3f m, %.1f deg", maxDev, maxD, maxA))
end

do -- SC7 CAM-9 T1: aim requested 0.4 s before the release end -> no aim while held, soft first aim, no snaps
  local pistol = { GetItemID = function() return { tdb = "Items.Preset_Lexington_Default" } end,
                   GetEntityID = function() return { hash = "pistol" } end }
  local W, cam, vc, T = sessionX({}, function(w, t, T)
    if t >= T.tu + 0.5 + 0.8 - 0.4 then w.upperBody, w.weapon = 6, pistol end
  end)
  local relAt = firstAt(W, "vehicle release to orbit over") or T.tu
  local endAt = firstAt(W, "vehicle hold OFF (release to orbit done)") or 1e9
  local aimAt = firstAt(W, "cam aim enter")
  local early = 0
  for _, l in ipairs(W.lines) do
    local t = tonumber(l:sub(1, 7)) or 0
    if t < endAt - 1e-6 and string.find(l, "cam aim enter", 1, true) then early = early + 1 end
  end
  local maxD, atD, maxA, atA = continuity(W, relAt, (aimAt or relAt) + 0.5)
  check("SC7 CAM-9 T1: no aim while held, the first aim after the hold blends in 0.20 s, continuous (<= 0.8 m / 25 deg)",
    aimAt ~= nil and aimAt >= endAt - 1e-6 and early == 0
    and count(W, "(first aim after a vehicle hold: blend in 0.20 s)") == 1 and maxD <= 0.8 and maxA <= 25,
    string.format("aim %s, hold end %.2f, %.3f m at %.2f, %.1f deg at %.2f", tostring(aimAt), endAt, maxD, atD or -1, maxA,
    atA or -1))
end

do -- SC8 CAR-3 framing + CAM-8 T1: bike exit, V's capsule rises 2.5 m over 0.5 s after the unmount (bike standing)
  local function framing(W, T, from)
    local maxAng, maxP = 0, 0
    local prevP = nil
    for _, r in ipairs(W.rec) do
      if r.t >= T.tu + (from or 0.05) and r.t <= T.tu + 1.5 then
        local head = W.track8 and W.track8[r.t] or nil
        local _, p = yawPitch(r.fwd)
        if prevP then maxP = math.max(maxP, math.abs(p - prevP)) end
        prevP = p
        if head then
          local d = CM.sub(head, r.pos)
          maxAng = math.max(maxAng, math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(d), CM.norm(r.fwd)))))))
        end
      end
    end
    return maxAng, maxP
  end
  -- V's pivot (+1.4 m) per frame: recorded from the scripted position (the frame records the camera after the script)
  local function withTrack(before, height, riseT)
    local W, cam, vc, T
    local track = {}
    W, cam, vc, T = sessionX({ bike = true, mountDur = 1.6, before = before }, function(w, t, T)
      if t >= T.tu then
        local k = t - T.tu
        local h = (k < riseT) and (height * k / riseT) or math.max(0, height - height * (k - riseT) / 0.5)
        w.p.pos = CM.v3(w.p.pos.x, w.p.pos.y, h)
      end
      track[t] = CM.add(w.p.pos, CM.v3(0, 0, 1.4))
    end)
    W.track8 = track
    return W, cam, vc, T
  end
  local W, cam, vc, T = withTrack(nil, 2.5, 0.5)
  local ang, dp = framing(W, T)
  check("SC8 CAR-3 framing: V's pivot stays within 30 deg of the view axis after the unmount, one framing line",
    ang <= 30 and count(W, "vehicle framing:") == 1, string.format("%.1f deg, framing lines %d", ang, count(W, "vehicle framing:")))
  -- KNOWN cross-worker defect (utpp_camera, not this file): the framing offset is added in VehicleCompose but the per-frame
  -- SetPoseOverride -> writeNow writes the raw override pitch, so the band is not shown while held and the release starts
  -- from the framed vh.last pitch (one-frame pitch step). This check stays strict and fails until writeNow applies frameOff.
  check("SC8 CAR-3 framing: pitch change <= 3 deg per frame after the unmount", dp <= 3, string.format("%.2f deg/frame", dp))
  local Wn, _, _, Tn = withTrack(function(_, c, v) v.SetExitFollow(false); c.cfg.vehFrame = false end, 2.5, 0.5)
  local angN = framing(Wn, Tn)
  check("SC8 negative control: follow and framing off -> V leaves the frame (the reported behaviour)", angN > 30,
    string.format("%.1f deg", angN))
  -- CAM-8 T1: the recorded fast launch (V about a body height above the seat within 50 ms at 545.0): follow on the rise
  -- (the first frames of such a launch are faster than any reactive follow; checked from the end of the follow blend on)
  local W8, _, _, T8 = withTrack(nil, 2.0, 0.15)
  local ang8 = framing(W8, T8, 0.4)
  local fAt = firstAt(W8, "above the seat height after the unmount")
  local W8n, _, _, T8n = withTrack(function(_, _, v) v.ctrl.cfg.exitFollowRise = 100 end, 2.0, 0.15)
  local ang8n = framing(W8n, T8n, 0.4)
  check("CAM-8 T1: fast launch after the unmount -> follow on the rise (CAM-8 (a)) within 0.2 s, V's pivot within 30 deg of "
    .. "the view axis from unmount + 0.4 s; without the rise trigger it is not", fAt ~= nil and fAt - T8.tu <= 0.2
    and ang8 <= 30 and ang8n > ang8 and count(W8n, "above the seat height") == 0,
    string.format("follow %s, %.1f deg (without the trigger %.1f deg)", tostring(fAt), ang8, ang8n))
end

do -- CAM-5 T1: exit 0.18 s after the hand-over Deactivate (during the engine blend), vehicle camera pose far from the pull-out
  -- (2026-11-05: ride 1.55 -> 1.95 s: the hand-over moved from about td + 1.37 to td + 1.77 s with the 6 m/s pull-out and the glide after it)
  local W, cam, vc, T = session({ far = { y = -8.5, z = 3.4, pitch = -15 }, ride = 1.95, rideMove = 0 })
  local hd = W.deacts[1]
  local maxD, atD, maxA, atA = continuity(W, T.tx, T.tx + 0.3)
  check("CAM-5 T1: exit during the hand-over blend -> matched to the blended pose (logged), take-over blend 0, continuous "
    .. "(<= 0.8 m / 25 deg per 50 ms over the exit + 0.3 s)", hd and hd.t < T.tx and count(W, "exit during the hand-over blend (w=") == 1
    and count(W, "TakeFromVehicle(0.00)=ok") == 1 and maxD <= 0.8 and maxA <= 25,
    string.format("exit %.2f s after the hand-over, %.3f m at %.2f, %.1f deg at %.2f", T.tx - (hd and hd.t or 0), maxD, atD or -1,
    maxA, atA or -1))
end

do -- CAM-6 T3: bike TPPClose vehicle camera 1.55 m behind -> matched at the exit (no vehicle-behind fallback), continuous
  local W, cam, vc, T = session({ bike = true, mountDur = 1.6, far = { y = -1.55, z = 0.98, pitch = -8 } })
  local maxD, atD = continuity(W, T.tx, T.tu)
  check("CAM-6 T3: close bike camera accepted as the exit match, no vehicle-behind fallback, continuous",
    linesFrom(W, "exit match: cam=(0.00,-1.55,0.98)", T.tx - 0.01) == 1 and linesFrom(W, "is inside the vehicle", T.tx - 0.01) == 0
    and maxD <= 0.8, dump(W, { "exit match", "exit (" }, T.tx - 0.05) .. string.format(" | %.3f m at %.2f", maxD, atD or -1))
end

do -- CAM-10 T1: bike hand-over: the rendered camera moves <= 6 m/s relative to the bike
  -- a close bike camera (TPPClose 1.55 m behind), bike standing: the dolly from the pull-out pose (about 14 m/s with the
  -- car's 5.5 / 2.4 target before CAM-10)
  local W, cam, vc, T = session({ bike = true, mountDur = 1.6, far = { y = -1.55, z = 0.98, pitch = -8 }, rideMove = 0 })
  local hd = W.deacts[1]
  local maxV = 0
  if hd then
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if b.t >= hd.t and b.t <= hd.t + 0.75 then
        local ax, ay = rot(a.pos.x - a.vpos.x, a.pos.y - a.vpos.y, -a.vyaw)
        local bx, by = rot(b.pos.x - b.vpos.x, b.pos.y - b.vpos.y, -b.vyaw)
        local d = math.sqrt((bx - ax) ^ 2 + (by - ay) ^ 2 + ((b.pos.z - b.vpos.z) - (a.pos.z - a.vpos.z)) ^ 2)
        maxV = math.max(maxV, d / DT)
      end
    end
  end
  check("CAM-10 T1: bike hand-over Deactivate(0.65), camera <= 6 m/s relative to the bike", hd and math.abs(hd.b - 0.65) < 1e-6
    and maxV <= 6, string.format("blend %s, %.2f m/s", tostring(hd and hd.b), maxV))
end

-- ================================================================ 2026-10-28 mount pivot continuity (bike seat-slot snap)
-- recorded bike mount (log 15-45-08 15:48:40.72 -> 40.85, video 15-56-56 26.600 -> 26.625): V stands at the bike's right
-- side, the game's mount puts the entity on the seat slot in ONE frame (about 1.2 m, yaw to the vehicle's, +0.68 m z) and
-- holds it there during the entry clip; the camera pivot xy followed in one frame (the camera cut), the pull-out started
-- 0.2 s later (bike within pullDist). Scripted: V at the right side until tm, the seat slot from tm.
local function snapSession(opts)
  opts = opts or {}
  local W = makeWorld({ bike = true })
  local cam, vc = load(W)
  local T = timeline({ bike = true, mountDur = 1.6 })
  local side, slot = CM.v3(0.95, -0.55, 0), CM.v3(0, -0.2, 0.68)
  local yaw0 = opts.yaw0 or 0
  local base = T.script
  local script = function(W2, t)
    base(W2, t)
    if t < T.tm then
      W2.p.pos, W2.p.yaw, W2.p.vel = W2.vehToWorld(side.x, side.y, side.z), yaw0, nil
    elseif t < T.td then
      W2.p.pos, W2.p.yaw = W2.vehToWorld(slot.x, slot.y, slot.z), opts.spin and opts.spin(t - T.tm) or 0
      -- c9 M3: the recorded mount slide moves V (camTrace speed 0.5-1.6 m/s): not the idle orbit while it lasts
      if opts.spin then W2.p.vel = (t - T.tm < 0.9) and CM.v3(0.8, 0, 0) or nil end
    end
  end
  if opts.off then withEnv(W, function() cam.SetMountBlend(false) end) end
  if opts.trace then cam.rt.trace = true end -- camTrace lines (camera.trace)
  run(W, T.tm + 1.2, script)
  -- per-frame rendered camera step and pivot step over [tm - 0.1, tm + 0.6]
  local maxCam, atCam, maxPiv = 0, nil, 0
  for i = 2, #W.rec do
    local a, b = W.rec[i - 1], W.rec[i]
    if b.t >= T.tm - 0.1 and b.t <= T.tm + 0.6 then
      local d = math.sqrt((b.pos.x - a.pos.x) ^ 2 + (b.pos.y - a.pos.y) ^ 2)
      if d > maxCam then maxCam, atCam = d, b.t end
    end
  end
  return W, cam, vc, T, maxCam, atCam
end
do
  local Woff, _, _, Toff, offCam, offAt = snapSession({ off = true })
  local W, cam, vc, T, onCam, onAt = snapSession()
  local startAt = firstAt(W, "mount pivot blend start")
  check("mount snap: one blend start on the snap frame, before the vehicle hold (the camera updates first)",
    count(W, "mount pivot blend start") == 1 and startAt ~= nil and startAt >= T.tm - 1e-6 and startAt <= T.tm + DT + 0.001 -- log time printed with 3 decimals
    and count(W, "(before the vehicle hold)") == 1 and count(W, "mount pivot blend aborted") == 0,
    dump(W, { "mount pivot", "vehicle hold ON" }, T.tm - 0.05))
  check("mount snap: SetMountBlend(false) = the old cut (camera xy step about the snap in one frame)",
    count(Woff, "mount pivot blend") == 1 and offCam >= 0.9, string.format("%.3f m at %.3f", offCam, offAt or -1))
  check("mount snap: rendered camera xy <= 0.10 m per frame from the mount to +0.6 s (pull-out start included)",
    onCam <= 0.10, string.format("%.3f m at %.3f (off: %.3f m at %.3f)", onCam, onAt or -1, offCam, offAt or -1))
  local pullAt = firstAt(W, "pull-out start")
  -- the near trigger (0.2 s) waits for the end of the camera's blend (0.38 s): the path is frozen from a camera at rest
  check("mount snap: pull-out starts after the camera's mount pivot blend (0.38-0.45 s after the mount), logged",
    pullAt ~= nil and pullAt - T.tm >= 0.38 - 1e-6 and pullAt - T.tm <= 0.45
    and count(W, "after the camera's mount pivot blend)") == 1, dump(W, { "pull-out", "mount pivot" }, T.tm))
  local pullOff = firstAt(Woff, "pull-out start")
  check("mount snap: without the blend the pull-out keeps its 0.2 s near trigger", pullOff ~= nil and pullOff - Toff.tm >= 0.2 - 1e-6
    and pullOff - Toff.tm <= 0.25 and count(Woff, "after the camera's mount pivot blend") == 0, tostring(pullOff))
  -- the pull-out keeps its first frame on the last shown pose (frozen from the blended orbit): step across its start frame
  local stepPull = 0
  for i = 2, #W.rec do
    local a, b = W.rec[i - 1], W.rec[i]
    if pullAt and b.t >= pullAt - 1e-6 and b.t <= pullAt + 2 * DT + 1e-6 then
      stepPull = math.max(stepPull, CM.dist(a.pos, b.pos))
    end
  end
  check("mount snap: no jump at the pull-out start (<= 0.01 m per frame: blend over, path starts at rest)", stepPull <= 0.01,
    string.format("%.3f m", stepPull))
  local maxD, atD, maxA = continuity(W, T.tm - 0.5, T.tm + 1.2)
  print(string.format("  mount snap: max camera xy step %.3f m/frame (off %.3f m), max %.3f m / 50 ms at %.2f s, %.1f deg",
    onCam, offCam, maxD, atD or -1, maxA))
  -- the blended pivot crosses the bike (right side -> seat): the rendered camera never inside the bike box
  local inBox = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t <= T.tm + 1.2 and insideBox(W, r.pos) then inBox = inBox + 1 end end
  check("mount snap: camera never inside the bike during the blend and the pull-out start", inBox == 0, string.format("%d frames", inBox))
  -- yaw snap of the body (V turned 75 deg from the vehicle before the mount, as in the recording): same continuity bound
  local Wy, _, _, Ty, yCam, yAt = snapSession({ yaw0 = 75 })
  check("mount snap: with a 75 deg body yaw snap the camera xy still moves <= 0.12 m per frame",
    yCam <= 0.12 and count(Wy, "mount pivot blend start") == 1, string.format("%.3f m at %.3f", yCam, yAt or -1))
  -- gate: outside the mount window / on foot no blend (ride motion, on-foot steps), the window line once per mount
  local st = {}
  local cfg = CM.config()
  local function g(clock, n, state, mounted, vh, x)
    local a, k, l = cam.MountShiftGate(st, cfg, clock, n, state, mounted, vh, { x = x or 0, y = 0, z = 0 })
    return a, k, l
  end
  local a1, k1 = g(0, 1, "tpp", false, { held = false })
  local a2, k2 = g(DT, 2, "tpp", true, { held = false })
  local a3 = g(0.4, 3, "tpp", true, { held = true, override = { x = 0 } })
  local a4, k4 = g(0.45, 4, "tpp", true, { held = true })
  local a5, k5, l5 = g(0.6, 5, "tpp", true, { held = true })
  local a6, k6, l6 = g(0.62, 6, "tpp", true, { held = true }, 2)
  local a7, k7 = g(0.64, 7, "fpp", true, { held = true })
  local a8, k8 = g(0.66, 8, "tpp", true, { held = true, onFoot = true })
  check("mount gate: on foot no arm / keep; first mounted frame arms; override hides the orbit (no arm); window closes "
    .. "after 0.5 s with one line; FPP / on foot release keep",
    not a1 and not k1 and a2 and k2 and not a3 and a4 and k4 and not a5 and k5 and l5 ~= nil and not a6 and k6 and l6 == nil
    and not a7 and not k7 and not a8 and not k8,
    string.format("%s %s %s %s %s %s %s %s %s", tostring(a1), tostring(a2), tostring(a3), tostring(a4), tostring(a5), tostring(l5),
      tostring(a6), tostring(k7), tostring(k8)))
  -- a gap in the updates (menu / photo mode / camera inactive) while mounted continues the same mount: no new window, no arm
  st = {}
  g(0, 8, "tpp", false, { held = false })
  g(DT, 9, "tpp", true, { held = false })
  local anc, _, lnc = g(0.70, 10, "tpp", true, { held = false }) -- gap after the window closed
  local a11, _, l11 = g(0.70 + DT, 11, "tpp", true, { held = false })
  check("mount gate: a gap in the updates while mounted never re-arms the window (the window line stays one per mount)",
    anc == false and a11 == false and lnc ~= nil and l11 == nil)
  -- riding fast (0.31 m per 60 fps frame), the pause menu closes while the orbit is shown: no arm after the gap, so the
  -- solver gets no start for the 0.3+ m steps (CM.mountShiftStep needs arm)
  st = {}
  local DT60 = 1 / 60
  local clk, n, x = 0, 100, 0
  local armed = false
  g(clk, n, "tpp", false, { held = false }, x) -- on foot / bike not yet mounted
  for _ = 1, 4 do clk, n, x = clk + DT60, n + 1, x + 0.05; g(clk, n, "tpp", true, { held = false }, x) end
  for _ = 1, 120 do clk, n, x = clk + DT60, n + 1, x + 0.31; g(clk, n, "tpp", true, { held = false }, x) end -- 2 s riding
  clk, n, x = clk + 3.0, n + 40, x + 0.31 -- menu: frozen clock jump, update counter gap
  local aGap = g(clk, n, "tpp", true, { held = false }, x)
  for _ = 1, 40 do
    clk, n, x = clk + DT60, n + 1, x + 0.31
    if g(clk, n, "tpp", true, { held = false }, x) then armed = true end
  end
  check("mount gate: fast ride (18 m/s) + menu gap while the orbit is shown -> never armed, no blend", aGap == false and not armed)
  -- hitch with a large dt inside the real mount window after a blend start: no second start (one blend per mount)
  st = {}
  g(0, 200, "tpp", false, { held = false }, 0)
  local ah1 = g(DT, 201, "tpp", true, { held = false }, 0)
  local ah2 = g(2 * DT, 202, "tpp", true, { held = false }, 0)
  st.mshStarted = true -- the caller marks the start the solver reported on that frame
  local ah3 = g(0.40, 203, "tpp", true, { held = false }, 1.0) -- hitch: 0.4 s frame, 1 m step inside the 0.5 s window
  local ah4 = g(0.40 + DT, 204, "tpp", true, { held = false }, 2.0)
  check("mount gate: hitch inside the window after a blend start -> no restart; the first mounted frames did arm",
    ah1 == true and ah2 == true and ah3 == false and ah4 == false,
    string.format("%s %s %s %s", tostring(ah1), tostring(ah2), tostring(ah3), tostring(ah4)))
  -- unmount and mount again: a new window and a new blend allowed
  g(1.0, 205, "tpp", false, { held = false }, 2.0)
  g(1.0 + DT, 206, "tpp", true, { held = false }, 2.0)
  local am = g(1.0 + 2 * DT, 207, "tpp", true, { held = false }, 2.0)
  check("mount gate: the next mount arms again (blend flag reset per mount)", am == true and st.mshStarted == false)
  -- a snap one frame BEFORE the mount read is not blended; the window line names it (next game run: decides whether the
  -- arm must move earlier)
  st = {}
  g(0, 20, "tpp", false, { held = false }, 0)
  g(DT, 21, "tpp", false, { held = false }, 1.2) -- snap while the mount is not readable yet
  local aS = g(2 * DT, 22, "tpp", true, { held = false }, 1.2)
  local _, _, lS = g(0.6, 23, "tpp", true, { held = false }, 1.2)
  check("mount gate: snap one frame before the mount read -> not blended (already shown), the window line reports it (1.20 m)",
    aS == true and lS ~= nil and string.find(lS, "one frame before the mount read 1.20 m", 1, true) ~= nil, tostring(lS))
end

-- ================================================================ 2026-10-28 (c9 M3) mount slide spin during the pull-out
-- recorded bike mount (log 19-54-19 07.85-08.75, camTrace body -8.4 -> 58.7 -> -8.4): the game slides / turns the entity
-- about +67 deg in 0.3 s on the frozen enter pose, then the clip root unwinds it over about 0.6 s, while the pull-out runs
-- (it starts 0.2 s after the mount). The pull-out lerped from the LIVE orbit yaw, which followed the body under the
-- override: the shown camera whipped (camTrace out -8.4 -> 12.1 -> 20.2 -> -10.8, video 24 px/frame). Now the orbit yaw /
-- pitch are frozen with the path start and the hidden orbit keeps its world yaw under the mount override (keepYaw).
do
  local function spin(dt)
    if dt < 0.3 then return 67 * smooth(dt / 0.3) end
    if dt < 0.9 then return 67 * (1 - smooth((dt - 0.3) / 0.6)) end
    return 0
  end
  local function yawOf(r) return CM.yawPitchFromForward(r.fwd) end
  local Ws, _, _, Ts = snapSession({ off = true, spin = spin, trace = true })
  local W0 = snapSession({ off = true, trace = true })
  local pullAt = firstAt(Ws, "pull-out start")
  local maxStep, atStep, maxDiff, atDiff, firstStep = 0, nil, 0, nil, nil
  for i = 2, #Ws.rec do
    local a, b = Ws.rec[i - 1], Ws.rec[i]
    if pullAt and b.t >= pullAt - DT - 1e-6 and b.t <= pullAt + 0.6 then
      local d = math.abs(CM.wrapDeg(yawOf(b) - yawOf(a)))
      if d > maxStep then maxStep, atStep = d, b.t end
      if firstStep == nil and b.t >= pullAt + DT - 1e-6 then firstStep = d end -- first frame after the pull start frame
    end
    local c = W0.rec[i]
    if c and math.abs(c.t - b.t) < 1e-6 and b.t >= Ts.tm and b.t <= Ts.tm + 1.2 then
      local d = math.abs(CM.wrapDeg(yawOf(b) - yawOf(c)))
      if d > maxDiff then maxDiff, atDiff = d, b.t end
    end
  end
  check("c9 M3: body spin +67 deg during the mount -> rendered camera yaw <= 4 deg per frame from the pull-out start to +0.6 s,"
    .. " zero yaw velocity on the first pull frame (<= 0.1 deg)", pullAt ~= nil and maxStep <= 4 and firstStep ~= nil
    and firstStep <= 0.1, string.format("pull %s, %.2f deg at %.3f, first frame %.3f deg", tostring(pullAt), maxStep,
    atStep or -1, firstStep or -1))
  check("c9 M3: the rendered camera is the same with and without the body spin (mount to +1.2 s, <= 0.5 deg)", maxDiff <= 0.5,
    string.format("%.2f deg at %.3f", maxDiff, atDiff or -1))
  -- camTrace (10 Hz): out = the orbit solve, shown = the written yaw; neither may step > 4 deg per sample at the pull start
  local maxOut, maxShown, nTr, nShown, prevO, prevS = 0, 0, 0, 0, nil, nil
  for _, l in ipairs(Ws.lines) do
    local t = tonumber(l:sub(1, 7))
    if t and pullAt and t >= pullAt - 0.1 and t <= pullAt + 0.6 and string.find(l, "camTrace t=", 1, true) then
      local o = tonumber(string.match(l, " out=(%-?[%d%.]+)"))
      local s = tonumber(string.match(l, " shown=(%-?[%d%.]+)/"))
      nTr = nTr + 1
      if o and prevO then maxOut = math.max(maxOut, math.abs(CM.wrapDeg(o - prevO))) end
      if s then nShown = nShown + 1 end
      if s and prevS then maxShown = math.max(maxShown, math.abs(CM.wrapDeg(s - prevS))) end
      prevO, prevS = o, s
    end
  end
  check("c9 M3: camTrace out and shown change <= 4 deg per sample at the pull-out start (orbit keeps its world yaw)",
    nTr >= 5 and nShown >= 5 and maxOut <= 4 and maxShown <= 4,
    string.format("%d lines (%d shown), out %.2f, shown %.2f deg", nTr, nShown, maxOut, maxShown))
  check("c9 M3: frozen orbit yaw / pitch logged with the path start, the override marks keepYaw",
    count(Ws, "orbit yaw ") >= 1 and count(Ws, ", orbit yaw ") == count(Ws, "pull-out path: start frozen at")
    and count(Ws, "- orbit keeps its world yaw (mount)") == 1, dump(Ws, { "pull-out path", "vehicle pose override on" }, Ts.tm))
end
do -- c9 E3 diagnostic (read only): one unmount yaw line at the vehicle edge of a held exit and one 0.4 s later
  local W, _, _, T = session({ bike = true, mountDur = 1.6 })
  local at = firstAt(W, "vehicle unmount yaw: entity")
  check("c9 E3: unmount yaw diagnostic once at the unmount (entity / shown camera / FPP view / heading lock) and once +0.4 s",
    count(W, "vehicle unmount yaw: entity") == 1 and count(W, "vehicle unmount yaw +") == 1 and at ~= nil
    and math.abs(at - T.tu) <= 2 * DT + 0.001
    -- the test world's FPP camera stub is unlocked: false must be printed as false (not nil)
    and count(W, "heading lock false (last mounted frame false)") == 1,
    dump(W, { "vehicle unmount yaw", "hold", "release" }, T.tu - 0.1))
end

-- ================================================================ 2026-11-01 car camera: entry glide, exit take / match, passengers
-- a block that raises (e.g. on the base code without the new API) is one failed check, not an abort
local function runBlock(fn)
  local ok, err = pcall(fn)
  if not ok then check("camera 2026-11-01: a test block raised an error", false, tostring(err)) end
end
-- Log 07-50-12: after the own pull-out (0,-5.50,2.40) the hand-over zoomed out 3.5 m in 0.45 s (the vehicleTPPCamera component
-- reads at V's seat, so no pose match engaged); at the exit the native FPP switch of the vehicle camera mixed into the
-- 0.30 s take (lurch toward the cabin) and the 0.70 s match moved 3.8 m (peak ~7 m/s). The mock now blends away from a live
-- camera (the vehicle camera moving to the eye at the exit) and renders per-camera FOV; opts.far = the measured vehicle camera.
-- 2026-11-01: pitch 0 - the settled vehicle camera reads forward pitch 0 on every car (log 07-50-12 :1474-1490 Hella TPPFar,
-- logs 15-00-36 / 15-20-28 the four new classes); the earlier -9 was the look-at estimate of the glide, which ended 9 deg off
local MEAS = { y = -9.0, z = 2.5, pitch = 0 } -- Archer Hella TPPFar at rest, FOV 69
local MEAS_EXIT = { y = -9.0, z = 2.5, pitch = -9 } -- the same camera pitched by the player during the ride: a proven (non 0 / 0) orientation at the exit

local function maxFovStep(W, t0, t1)
  local m, at = 0, nil
  for i = 4, #W.rec do
    local a, b = W.rec[i - 3], W.rec[i]
    if a.t >= t0 and b.t <= t1 and a.fov and b.fov and math.abs(b.fov - a.fov) > m then m, at = math.abs(b.fov - a.fov), b.t end
  end
  return m, at
end
local function recNear(W, t)
  local best = W.rec[1]
  for _, r in ipairs(W.rec) do if math.abs(r.t - t) < math.abs(best.t - t) then best = r end end
  return best
end
local function vehDeacts(W, T) -- the Deactivate calls before the exit
  local n = 0
  for _, d in ipairs(W.deacts) do if d.t < T.tx then n = n + 1 end end
  return n
end

runBlock(function() -- E1: the entry glide to the measured vehicle camera: eased, no zoom pop, no step at the hand-over
  local pullDoneAt = nil
  local W, cam, vc, T = sessionX({ far = MEAS, rideMove = 0, ride = 3.0 }, function(w)
    if not pullDoneAt and w.mods.vc.ctrl.pull >= 1 and w.mods.vc.ctrl.keep == "mount" then pullDoneAt = w.t end
  end)
  local doneAt = firstAt(W, "done (vehicle TPPFar)")
  local gAt = firstAt(W, "handover glide: default target (0.00,-9.00,2.50) yaw=0 pitch=0 fov=69")
  local hd = W.deacts[1]
  check("E1: the glide starts when the pull-out is over and the request phase is done (<= 3 frames after the later), logged "
    .. "with source / target / shown pose", doneAt and gAt and pullDoneAt and gAt >= doneAt - 1e-6 and gAt >= pullDoneAt - 1e-3
    and gAt - math.max(doneAt, pullDoneAt) <= 3 * DT
    and count(W, "from shown (0.00,-5.50,2.40) yaw=0 pitch=-14 over 1.05 s, then Deactivate(0.80)") == 1,
    string.format("pull end %s done %s glide %s", tostring(pullDoneAt), tostring(doneAt), tostring(gAt)))
  check("E1: hand-over = one Deactivate(0.80) when the 1.05 s glide ends (+-1 frame), after the vehicle camera blend",
    hd and gAt and vehDeacts(W, T) == 1 and math.abs(hd.b - 0.80) < 1e-6 and hd.t - gAt >= 1.05 - 0.002
    and hd.t - gAt <= 1.05 + 2 * DT and hd.t - (W.v.acceptAt or 0) >= 0.449,
    hd and string.format("%.3f s after the glide start, blend %.2f", hd.t - (gAt or 0), hd.b))
  -- 2026-11-01 test-window fix, not a tolerance change: the window used to start 0.3 s before the glide, which now lies inside the
  -- slowed pull-out of this 87 deg sweep (83 deg/s peak = 4.2 deg per 50 ms; it was 145 deg/s before and outside the window). The
  -- check is about the glide and the hand-over (its label), so the window starts at the glide start; the pull-out has its own gate
  -- in test_utpp_vehiclecam.lua (peak turn rate <= 110 deg/s, max step per frame).
  local maxD, atD, maxA, atA = continuity(W, (gAt or 0), (hd and hd.t or 0) + 0.8)
  check("E1: no zoom pop from the glide start to 0.8 s after the hand-over (<= 0.45 m and <= 4 deg per 50 ms)",
    maxD <= 0.45 and maxA <= 4, string.format("%.3f m at %.2f, %.1f deg at %.2f", maxD, atD or -1, maxA, atA or -1))
  local atH = recNear(W, hd and hd.t or 0)
  local tgt = W.vehToWorld(0, MEAS.y, MEAS.z)
  check("E1: at the hand-over the shown camera is on the vehicle camera pose (position < 0.1 m, FOV 69 +-0.5, forward < 1 deg)",
    CM.dist(atH.pos, tgt) < 0.1 and math.abs(atH.fov - 69) < 0.5
    and math.deg(math.acos(math.min(1, CM.dot(CM.norm(atH.fwd), CM.forward(0, MEAS.pitch))))) < 1,
    string.format("%.3f m, fov %.2f", CM.dist(atH.pos, tgt), atH.fov))
  local fovStep, fovAt = maxFovStep(W, (gAt or 0) - 0.3, (hd and hd.t or 0) + 0.8)
  check("E1: FOV continuous (<= 3 deg per 50 ms) from the profile FOV to the vehicle camera's 69", fovStep <= 3,
    string.format("%.2f deg at %.2f", fovStep, fovAt or -1))
  local inBox = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t < (hd and hd.t or 0) + 0.8 and insideBox(W, r.pos) then inBox = inBox + 1 end end
  check("E1: camera never inside the vehicle through the glide and the hand-over", inBox == 0, inBox)
  -- the camTrace continuity measure (per-frame step / rotation): lines carry fstep= / frot=, one summary per window with the handover frames
  local winFrames = 0
  for _, l in ipairs(W.lines) do
    local n = string.match(l, "camera continuity: %d+ frames.-handover window %((%d+) frames%)")
    if n then winFrames = math.max(winFrames, tonumber(n)) end
  end
  check("E1: camTrace continuity: fstep= / frot= on the trace lines, a 'camera continuity:' summary with handover-window frames > 0",
    count(W, " fstep=") > 20 and count(W, "camera continuity:") >= 1 and winFrames > 20, string.format("fstep lines %d, summaries %d, window frames %d",
    count(W, " fstep="), count(W, "camera continuity:"), winFrames))
  check("E1: the vehicle camera renders after the hand-over, the exit is a hold from there (no errors)",
    recNear(W, T.tx - 0.2).win == "veh" and cam.stats.errors == 0 and count(W, "Step error") == 0 and count(W, "FAILED") == 0)
  -- negative control: without the glide the engine blend zooms out 3.5 m in 0.45 s
  local W0, _, _, T0 = session({ far = MEAS, rideMove = 0, ride = 3.0, before = function(_, _, v) ab(v, "SetHandGlide", false) end })
  local hd0 = W0.deacts[1]
  local m0 = continuity(W0, (hd0 and hd0.t or 0) - 0.1, (hd0 and hd0.t or 0) + 0.8)
  check("E1 negative control: SetHandGlide(false) = Deactivate(0.45) from the pull-out pose, the zoom step is > 0.5 m per 50 ms",
    hd0 and math.abs(hd0.b - 0.45) < 1e-6 and m0 > 0.5 and count(W0, "handover glide") == 0, string.format("%.3f m", m0))
  print(string.format("  E1 glide: hand-over %.3f s after the glide start, max %.3f m / %.1f deg per 50 ms (without the glide %.3f m)",
    (hd and gAt) and hd.t - gAt or -1, maxD, maxA, m0))
end)

runBlock(function() -- E2: a wrong class default (stock mock camera (0,-6.8,2.6), 2.2 m from the default) is still smooth; learning fixes later rides
  local W1, _, vc1, T1 = session({ rideMove = 0, ride = 4.0 })
  local hd = W1.deacts[1]
  local maxD, atD, maxA = continuity(W1, (hd and hd.t or 0) - 0.9, (hd and hd.t or 0) + 0.9)
  check("E2: wrong default (2.2 m off): the glide + 0.80 s residual blend stay <= 0.45 m / 5 deg per 50 ms",
    hd and math.abs(hd.b - 0.80) < 1e-6 and maxD <= 0.45 and maxA <= 5, string.format("%.3f m at %.2f, %.1f deg", maxD, atD or -1, maxA))
  local lAt = firstAt(W1, "vehicle TPP pose learned: Vehicle.v_mock_car|3 TPPFar (0.00,-6.80,2.60) yaw=0 pitch=-12 fov=69")
  check("E2: after the release the standing vehicle's settled camera is learned (record + perspective), error vs the default "
    .. "glide target logged", lAt ~= nil and hd and lAt >= hd.t + 0.80 + 0.1 and count(W1, "error vs the default glide target 2.2") == 1
    and count(W1, "vehicle TPP pose learned") == 1, dump(W1, { "learned", "glide" }))
  -- the next ride of the same session (same record / perspective) glides to the learned pose; unconfirmed = the 0.45 s residual
  local learned = vc1.ctrl.vehTppLearned
  check("E2: the first reading is stored unconfirmed", learned["Vehicle.v_mock_car|3"] and learned["Vehicle.v_mock_car|3"].confirmed == false)
  local W2 = select(1, session({ rideMove = 0, ride = 4.0, before = function(_, _, v) v.ctrl.vehTppLearned = learned end }))
  local hd2 = W2.deacts[1]
  local g2 = firstAt(W2, "handover glide: learned target (0.00,-6.80,2.60)")
  local atH2 = recNear(W2, hd2 and hd2.t or 0)
  local maxD2, atD2, maxA2 = continuity(W2, (hd2 and hd2.t or 0) - 0.9, (hd2 and hd2.t or 0) + 0.9)
  check("E2: learned (unconfirmed) target: Deactivate(0.80), shown camera on the vehicle camera at the hand-over (< 0.1 m), no step",
    g2 and hd2 and math.abs(hd2.b - 0.80) < 1e-6 and CM.dist(atH2.pos, W2.vehToWorld(0, -6.8, 2.6)) < 0.1 and maxD2 <= 0.35
    and maxA2 <= 4 and count(W2, "handover glide: default") == 0,
    string.format("%.3f m, step %.3f m", CM.dist(atH2.pos, W2.vehToWorld(0, -6.8, 2.6)), maxD2))
  check("E2: the second ride's reading within 0.3 m confirms the pose (logged)", learned["Vehicle.v_mock_car|3"].confirmed == true
    and count(W2, "confirmed by an earlier reading") == 1)
  local W3 = select(1, session({ rideMove = 0, ride = 4.0, before = function(_, _, v) v.ctrl.vehTppLearned = learned end }))
  local hd3 = W3.deacts[1]
  check("E2: the confirmed pose is handed over with the short residual blend Deactivate(0.20)", hd3 and math.abs(hd3.b - 0.20) < 1e-6
    and count(W3, "handover glide: learned target") == 1)
  -- a car that drives off right after the hand-over: nothing is learned until it stands again (the mock car moves 6 m/s from
  -- td + 0.8 to td + 3.5)
  local Wm, _, _, Tm = session({})
  local lm = firstAt(Wm, "vehicle TPP pose learned")
  check("E2: a moving vehicle learns nothing; the pose is read only once it stands again", lm == nil or lm >= Tm.td + 3.5,
    tostring(lm))
  -- a camera orbited sideways by the player / one inside the vehicle is never stored
  local Wo = select(1, session({ rideMove = 0, ride = 4.0, far = { x = 3.4, y = -9.0, z = 2.5, pitch = -9 } }))
  check("E2: a camera orbited sideways (20 deg) is not stored (logged), no learned pose",
    count(Wo, "vehicle TPP pose not learned: Vehicle.v_mock_car|3 TPPFar") == 1 and count(Wo, "orbited") == 1
    and count(Wo, "vehicle TPP pose learned") == 0, dump(Wo, { "learned" }))
  local Wi = select(1, session({ rideMove = 0, ride = 4.0, far = { y = -1.0, z = 2.0, pitch = -10 } }))
  check("E2: a camera inside the vehicle's radius (first-person-like) is never stored", count(Wi, "vehicle TPP pose learned") == 0)
end)

runBlock(function() -- E3: unknown vehicle class (the IsA read failed) and bikes keep the previous hand-over and exit
  local W, _, _, T = session({ isaFails = true, rideMove = 0, ride = 3.0 })
  local hd = W.deacts[1]
  check("E3: unknown class (IsA failed): no glide, engine blend Deactivate(0.45) after the pose-match read, exit take 0.30",
    count(W, "handover glide") == 0 and hd and math.abs(hd.b - 0.45) < 1e-6 and count(W, "handover match: no vehicle camera pose") == 1
    and count(W, "TakeFromVehicle(0.30)=ok") == 1 and count(W, "target in 0.70 s (exit start)") == 1,
    dump(W, { "handover", "TakeFromVehicle" }))
  local Wb, _, _, Tb = session({ bike = true, mountDur = 1.6, far = { y = -4.6, z = 1.9, pitch = -10 } })
  local hdb = Wb.deacts[1]
  local keepReqs = 0
  for _, q in ipairs(Wb.reqs) do if q.t >= Tb.tx then keepReqs = keepReqs + 1 end end
  check("E3: bike: no glide, Deactivate(0.65), exit take 0.30, exit match 0.70 s, exit keep requests as before",
    count(Wb, "handover glide") == 0 and hdb and math.abs(hdb.b - 0.65) < 1e-6 and count(Wb, "TakeFromVehicle(0.30)=ok") == 1
    and count(Wb, "target in 0.70 s (exit start)") == 1 and keepReqs >= 1 and count(Wb, "exit keep vehicle TPP: request") == 1,
    string.format("keep requests %d", keepReqs))
end)

runBlock(function() -- E4: the car exit: a cut to the matched rendered camera, a gentle 0.7-1.0 s dolly, no lurch toward the cabin, no keep requests
  local W, cam, vc, T = session({ far = MEAS_EXIT, rideMove = 0, ride = 3.0 })
  local matchLine = nil
  for _, l in ipairs(W.lines) do if string.find(l, "exit match: cam=", 1, true) then matchLine = l; break end end
  local dur = matchLine and tonumber(string.match(matchLine, "target in ([%d%.]+) s")) or 0
  local d = math.sqrt(0 + 3.5 ^ 2 + 0.1 ^ 2)
  check("E4: matched take from the rendered camera = TakeFromVehicle(0.00) (cut); the match duration scales with the distance "
    .. "(3.5 m at 4 m/s = 0.88 s, clamped 0.7-1.0 s)", count(W, "SetPoseOverride=ok (matched active camera), TakeFromVehicle(0.00)=ok") == 1
    and dur >= 0.7 and dur <= 1.0 and math.abs(dur - d / 4.0) < 0.011, matchLine)
  local dev, devAt = 0, nil
  for _, r in ipairs(W.rec) do
    if r.t >= T.tx + DT and r.t <= T.tx + 0.6 then
      local e = CM.dist(r.pos, r.upos)
      if e > dev then dev, devAt = e, r.t end
    end
  end
  check("E4: no lurch toward the cabin: the rendered camera is the commanded pose from the first frame (<= 0.1 m)", dev <= 0.1,
    string.format("%.3f m at %.2f", dev, devAt or -1))
  local backwards, ys = 0, {}
  for _, r in ipairs(W.rec) do
    if r.t >= T.tx - 2 * DT and r.t <= T.tx + dur then ys[#ys + 1] = r.pos.y end
  end
  for i = 2, #ys do if ys[i] < ys[i - 1] - 0.02 then backwards = backwards + 1 end end
  check("E4: the dolly is monotonic (no move toward the cabin and back)", backwards == 0 and #ys > 30, string.format("%d backward frames", backwards))
  local maxD = continuity(W, T.tx, T.tx + dur + 0.1)
  check("E4: peak speed bound (<= 1.5 x distance / duration + 10%, per 50 ms)", maxD / 0.05 <= 1.5 * 3.5 / dur * 1.1,
    string.format("%.2f m/s vs %.2f", maxD / 0.05, 1.5 * 3.5 / dur * 1.1))
  local fStep = maxFovStep(W, T.tx - 0.1, T.tx + 1.5)
  check("E4: no FOV pop in the take (<= 3 deg per 50 ms)", fStep <= 3, string.format("%.2f deg", fStep))
  local keepReq = 0
  for _, q in ipairs(W.reqs) do if q.t >= T.tx then keepReq = keepReq + 1 end end
  check("E4: car exit: no exit keep requests, the passive diagnostic counts the FPP frames once",
    keepReq == 0 and count(W, "exit keep vehicle TPP: request") == 0
    and count(W, "exit vehicle camera (no requests, diagnostic): active stayed TPP=false") == 1, dump(W, { "exit keep", "exit vehicle camera" }))
  -- negative control (the previous behaviour): blend 0.30 into a vehicle camera that moves to the cabin, 0.70 s match
  local Wn = select(1, session({ far = MEAS_EXIT, rideMove = 0, ride = 3.0, before = function(_, _, v)
    ab(v, "SetExitTakeBlendMatched", 0.30); ab(v, "SetExitMatchScale", false); ab(v, "SetExitKeepVehicleTpp", true) end }))
  local devN = 0
  for _, r in ipairs(Wn.rec) do if r.t >= T.tx + DT and r.t <= T.tx + 0.6 then devN = math.max(devN, CM.dist(r.pos, r.upos)) end end
  local keepN = 0
  for _, q in ipairs(Wn.reqs) do if q.t >= T.tx then keepN = keepN + 1 end end
  check("E4 negative control: take blend 0.30, 0.70 s match, keep requests on = the lurch (> 0.5 m from the commanded pose) "
    .. "and the keep requests", devN > 0.5 and count(Wn, "target in 0.70 s (exit start)") == 1 and keepN >= 1, string.format("%.3f m, %d", devN, keepN))
  -- an orientation that reads 0 / 0 (the canonical forward) keeps a short blend: its pitch is not proven
  local Wu = select(1, session({ far = MEAS, rideMove = 0, ride = 3.0, canonFwd = true }))
  check("E4: orientation read 0 / 0 (not proven): short take blend 0.12 s instead of a cut",
    count(Wu, "SetPoseOverride=ok (matched active camera), TakeFromVehicle(0.12)=ok") == 1, dump(Wu, { "TakeFromVehicle" }))
  print(string.format("  E4 car exit: match %.2f s, deviation from the commanded pose %.3f m (before %.3f m), peak %.2f m/s", dur, dev, devN,
    maxD / 0.05))
end)

runBlock(function() -- E5: an exit during the glide and during the hand-over blend after a glide: continuous position, orientation and FOV
  -- 2026-11-05: the 1.05 s glide starts at the end of the 6 m/s pull-out (about td + 0.70 s): ride 0.75 -> 1.15 s keeps the exit in the middle of
  -- the glide (0.45 s in; at 0.75 s it came 0.05 s after the glide start), ride 1.55 -> 1.95 s keeps it 0.18 s into the hand-over blend
  local W, cam, vc, T = session({ far = MEAS, rideMove = 0, ride = 1.15 })
  local maxD, atD, maxA = continuity(W, T.tx - 0.3, T.tx + 1.5)
  local fStep, fAt = maxFovStep(W, T.tx - 0.3, T.tx + 1.5)
  local glideAt = firstAt(W, "handover glide:")
  check("E5: exit during the glide: utppCamera still shown, the match starts at the glide pose, continuous (<= 0.8 m, 25 deg, "
    .. "3 deg FOV per 50 ms)", count(W, "exit (exit before the handover): utppCamera still shown") == 1 and #W.deacts == 0
    and glideAt ~= nil and T.tx - glideAt >= 0.25 and T.tx - glideAt <= 0.8
    and maxD <= 0.8 and maxA <= 25 and fStep <= 3, string.format("exit %.2f s into the glide, %.3f m, %.1f deg, fov %.2f at %.2f",
      T.tx - (glideAt or 0), maxD, maxA, fStep, fAt or -1))
  local cap = nil
  local W2, _, _, T2 = sessionX({ far = MEAS, rideMove = 0, ride = 1.95 }, function(w)
    local c = w.mods.vc.ctrl
    if c.handOwnPose and not cap then
      cap = { x = c.handOwnPose.x, y = c.handOwnPose.y, z = c.handOwnPose.z, fov = c.handOwnPose.fov, pitch = c.handOwnPose.pitch }
    end
  end)
  local maxD2, atD2, maxA2 = continuity(W2, T2.tx, T2.tx + 0.5)
  local fStep2 = maxFovStep(W2, T2.tx - 0.1, T2.tx + 1.5)
  check("E5: exit during the hand-over blend: c.handOwnPose is the glide's end pose (0,-9,2.5, FOV 69), take blend 0, continuous",
    cap and math.abs(cap.x) < 0.1 and math.abs(cap.y + 9) < 0.1 and math.abs(cap.z - 2.5) < 0.1 and cap.fov and math.abs(cap.fov - 69) < 0.5
    and count(W2, "exit during the hand-over blend (w=") == 1 and count(W2, "TakeFromVehicle(0.00)=ok") == 1 and maxD2 <= 0.8
    and maxA2 <= 25 and fStep2 <= 3, cap and string.format("pose (%.2f,%.2f,%.2f) fov %s; %.3f m %.1f deg fov %.2f", cap.x, cap.y, cap.z,
      tostring(cap.fov), maxD2, maxA2, fStep2) or "no hand-over pose captured")
end)

runBlock(function() -- E6: passengers (SetPassengerChase(false)): the vehicle camera is asked for third person after the vanilla FPP request; the ride runs as keep game
  local W_cb = {}
  -- perspExitDelay: utpp_perspective keeps its FPP decision 0.4 s after the seat state ends (the take-over from the game camera
  -- happens inside it, as in the driver rides left to the game)
  local W, cam, vc, T = session({ driver = false, driverSeatFree = false, passengerAccept = true, rideMove = 0, ride = 3.0,
                                  perspExitDelay = 0.4, before = function(_, _, v)
                                    ab(v, "SetPassengerChase", false)
                                    v.onVehiclePerspective = function(a, b) W_cb[#W_cb + 1] = { a = a, b = b } end end })
  local req = W.reqs[1]
  -- (the request repeats every 0.25 s until the vehicle accepts, 0.29 s in the mock: like the driver's)
  check("E6: passenger entry: a TPP request in the passenger state, 0.15-0.4 s after it began (after the vanilla FPP request)",
    #W.reqs >= 1 and #W.reqs <= 2 and req and req.t >= T.td + 0.15 - 1e-6 and req.t <= T.td + 0.4 and recNear(W, T.tx - 0.2).win == "veh"
    and count(W, "always third person on entry: passenger ride - request") == 1,
    string.format("reqs %d first %s td %.2f; %s", #W.reqs, tostring(req and req.t), T.td,
      dump(W, { "passenger", "always third", "keepTpp" })))
  check("E6: the ride runs as keep game (exit taken from the game camera), the camera's FPP handoff of the passenger rule is kept",
    count(W, "keepTpp off -> game (passenger ride in the game's third-person vehicle camera)") == 1
    and count(W, "-> declined - first person as before") == 1 and count(W, "Step error") == 0 and cam.stats.errors == 0
    and count(W, "exit (exit start (from the game camera))") == 1 and count(W, "refused the exit take-over") == 0,
    dump(W, { "keepTpp", "exit (" }))
  check("E6: utpp_body's vehicle camera callback fires for the entry (FPP 0 -> TPPFar 3), head pre-equip trigger",
    #W_cb >= 1 and W_cb[1].a == 0 and W_cb[1].b == 3, W_cb[1] and string.format("%s -> %s", tostring(W_cb[1].a), tostring(W_cb[1].b)) or "none")
  -- the vanilla OnEnter queues TPPClose and then FPP: the transient TPP reading must not end the request
  local Wf2, camf, _, Tf = session({ driver = false, driverSeatFree = false, passengerAccept = true, passengerFlicker = true, rideMove = 0,
                                      ride = 3.0, perspExitDelay = 0.4, before = function(_, _, v) ab(v, "SetPassengerChase", false) end })
  local rf = Wf2.reqs[1]
  check("E6: a TPP reading that the vanilla FPP request follows (0.12-0.35 s) does not end the request: it is sent when FPP reads again",
    rf and rf.t >= Tf.td + 0.35 - 1e-6 and rf.t <= Tf.td + 0.6 and recNear(Wf2, Tf.tx - 0.2).win == "veh"
    and count(Wf2, "the vehicle camera reads TPPClose") == 1, string.format("first %s; %s", tostring(rf and rf.t - Tf.td),
      dump(Wf2, { "passenger", "always third" })))
  local Wf = select(1, session({ driver = false, driverSeatFree = false, passengerAccept = true, rideMove = 0, ride = 3.0,
    before = function(_, _, v) ab(v, "SetPassengerChase", false); ab(v, "SetForceTppPassenger", false) end }))
  check("E6: forceTppPassenger off: no request (previous behaviour)", #Wf.reqs == 0 and count(Wf, "keepTpp off -> game") == 0)
  -- passenger-side entry with the seat switch to the driver seat stays a driver entry: no passenger request
  local Ws = select(1, session({ driver = true }))
  check("E6: a driver entry sends no passenger request", count(Ws, "passenger ride - request") == 0)
end)

-- ================================================================ review round (2026-11-01): M4 slow entry, L1 read-pose sanity, M2 in the mock
runBlock(function() -- M4: a slow entry: the glide starts just before keepTimeout (6 s after the mount start) and must not be cut by it
  -- mount 5.2 s: Driving at td = tm + 5.2, the glide runs td + 0.3 .. td + 1.2, the cap falls at tm + 6.0 = mid-glide
  local W, cam, vc, T = session({ far = MEAS, rideMove = 0, ride = 4.0, mountDur = 5.2 })
  local gAt = firstAt(W, "handover glide: default target")
  local hd = W.deacts[1]
  check("M4: the cap (keepTimeout) falls inside the glide in this scenario", gAt and gAt < T.tm + 6.0 and gAt + 0.9 > T.tm + 6.0,
    tostring(gAt))
  check("M4: the glide is not cut by keepTimeout: no timeout line, hand-over after the glide with one Deactivate, continuous",
    count(W, "keepTpp mount -> off (timeout)") == 0 and hd and gAt and hd.t - gAt >= 0.9 - 0.002 and vehDeacts(W, T) == 1
    and continuity(W, T.tm, T.td + 4) <= 0.8, dump(W, { "keepTpp", "handover" }))
end)

runBlock(function() -- L1: the old hand-over read (handGlide off / not applicable) applies the same sanity check to a car
  local W, cam, vc, T = session({ vtppReal = true, far = { y = -3.0, z = 0.1, pitch = -5 }, rideMove = 0, ride = 3.0,
    before = function(_, _, v) ab(v, "SetHandGlide", false) end })
  check("L1: a car's readable pose below 0.3 m is rejected (engine blend, logged), not matched",
    count(W, "handover match: utppCamera -> vehicleTPPCamera") == 0 and count(W, "handover match: no vehicle camera pose") == 1,
    dump(W, { "handover" }))
  local Wb = select(1, session({ bike = true, mountDur = 1.6, vtppReal = true, far = { y = -3.0, z = 0.1, pitch = -5 },
    rideMove = 0, ride = 3.0 }))
  check("L1: a bike keeps the previous read (no sanity check): the pose is matched", count(Wb, "handover match: utppCamera -> vehicleTPPCamera") == 1,
    dump(Wb, { "handover" }))
end)

-- ================================================================ 2026-11-01 passenger on the own camera (game log 10-34-22) and the passenger exit pitch
-- The passenger's vehicle camera takes the TPP request into the persistent perspective but stays FPP (active=0 for the whole ride,
-- 10 s in the log). The mock keeps that: passengerAccept is NOT set, so a request changes nothing.
local function pitchOf(r) return math.deg(math.asin(math.max(-1, math.min(1, CM.norm(r.fwd).z)))) end
local function maxPitchDev(W, t0, t1)
  local lo, hi = 999, -999
  for _, r in ipairs(W.rec) do
    if r.t >= t0 and r.t <= t1 then local p = pitchOf(r); lo, hi = math.min(lo, p), math.max(hi, p) end
  end
  return lo, hi
end
local function lookPitchScript(T) -- the exit animation drives the game's look pitch to -60 at the end of the exit
  return function(w, t) if t >= T.tu - 0.1 and t <= T.tu + 2.5 then w.p.fppPitch = -60 else w.p.fppPitch = nil end end
end

runBlock(function() -- E7: a car passenger stays on utppCamera's chase pose (seat switch expected: the hold from the mount start)
  local W, cam, vc, T = sessionX({ driver = false, rideMove = 0, ride = 3.0, perspExitDelay = 0.4, far = MEAS }, nil)
  local notUtpp = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t <= T.tx + 1.0 and r.win ~= "utpp" then notUtpp = notUtpp + 1 end end
  local behind = nil
  for _, r in ipairs(W.rec) do if r.t >= T.tx - 0.2 and not behind then behind = W.worldToVeh(r.pos) end end
  local maxD, atD, maxA = continuity(W, T.tm - 0.5, T.te)
  check("E7: passenger with the mount hold: seat final -> keep chase (logged), no FPP cut, no vehicle camera request",
    count(W, "passenger seat (state 3): utppCamera keeps the vehicle-behind chase pose") == 1 and #W.fppActs == 0 and #W.reqs == 0
    and count(W, "exit epoch=") == 0 and notUtpp == 0, string.format("%d frames not utpp, %d reqs, fpp %d", notUtpp, #W.reqs, #W.fppActs))
  check("E7: the camera sits on the vehicle-behind pose (0,-5.5,2.4) for the ride, continuous (<= 0.8 m, 25 deg per 50 ms)",
    behind and math.abs(behind.x) < 0.05 and math.abs(behind.y + 5.5) < 0.05 and math.abs(behind.z - 2.4) < 0.05 and maxD <= 0.8
    and maxA <= 25, behind and string.format("(%.2f,%.2f,%.2f) %.3f m %.1f deg", behind.x, behind.y, behind.z, maxD, maxA) or "none")
  local sigOff = 0
  for _, r in ipairs(W.rec) do if r.t >= T.tm and r.t <= T.te and r.sig ~= true then sigOff = sigOff + 1 end end
  check("E7: the exit is taken from the shown chase pose (no activation), hold, unmount, release to the orbit; TPP signal on throughout",
    count(W, "exit (exit start (chase)): utppCamera still shown") == 1 and count(W, "vehicle release to orbit over 0.80 s") == 1
    and vhs(cam).held == false and cam.T.state == "tpp" and count(W, "Step error") == 0 and cam.stats.errors == 0 and sigOff == 0,
    string.format("rel %d held %s state %s sig off frames %d", count(W, "vehicle release to orbit over 0.80 s"), tostring(vhs(cam).held),
      tostring(cam.T.state), sigOff))
  -- a real passenger (the driver seat is taken: no hold at the mount, the decision is kept): the hold is taken at the passenger state
  local W2, cam2, _, T2 = sessionX({ driver = false, driverSeatFree = false, rideMove = 0, ride = 3.0, perspExitDelay = 0.4 }, nil)
  local late = firstAt(W2, "passenger seat (state 3): hold taken while seated")
  local notUtpp2, jumps = 0, 0
  for _, r in ipairs(W2.rec) do if r.t >= (late or 0) + DT and r.t <= T2.tx + 1.0 and r.win ~= "utpp" then notUtpp2 = notUtpp2 + 1 end end
  local maxD2, atD2, maxA2 = continuity(W2, (late or 0) - 0.3, T2.te)
  check("E7: a passenger of an occupied driver seat: the hold is taken at the passenger state (pull-out at once), chase for the ride, "
    .. "no FPP cut", late and late >= T2.td - 1e-6 and late <= T2.td + 3 * DT and #W2.fppActs == 0 and notUtpp2 == 0 and #W2.reqs == 0
    and count(W2, "pull-out start at once (hold taken while seated, state 3)") == 1 and maxD2 <= 0.8 and maxA2 <= 25
    and count(W2, "exit (exit start (chase)): utppCamera still shown") == 1, string.format("late %s, %d frames not utpp, %.3f m %.1f deg",
      tostring(late), notUtpp2, maxD2, maxA2))
  local Wf = select(1, sessionX({ driver = false, driverSeatFree = false, rideMove = 0, ride = 3.0, perspExitDelay = 0.4,
    before = function(_, _, v) ab(v, "SetPassengerChase", false) end }, nil))
  check("E7: SetPassengerChase(false) = the request path only (no chase, the camera cuts to FPP as before)",
    count(Wf, "keepTpp off -> chase") == 0 and #Wf.fppActs >= 1)
  local Wu = select(1, sessionX({ driver = false, driverSeatFree = false, rideMove = 0, ride = 3.0, perspExitDelay = 0.4, isaFails = true }, nil))
  check("E7: an unreadable vehicle class gets no chase and no request (unchanged)", count(Wu, "keepTpp off -> chase") == 0 and #Wu.reqs == 0
    and #Wu.fppActs >= 1)
end)

runBlock(function() -- E8: the exit pitch: the game's look pitch is -60 at the end of the exit animation; the on-foot camera starts at a sane pitch
  for _, v in ipairs({ { name = "own-camera passenger (chase hold, release to the orbit)", opts = {} },
                       { name = "passenger without a hold (camera re-enters from the game camera)", opts = { chaseOff = true } } }) do
    local T0 = timeline({ ride = 3.0 })
    local W, cam, vc, T = sessionX({ driver = false, driverSeatFree = false, rideMove = 0, ride = 3.0, perspExitDelay = 0.4,
      before = function(_, _, m) if v.opts.chaseOff then ab(m, "SetPassengerChase", false) end end }, lookPitchScript(T0))
    local lo, hi = maxPitchDev(W, T.tu - 0.3, T.tu + 3.0)
    local maxD, atD, maxA, atA = continuity(W, T.tu - 0.3, T.tu + 3.0)
    check("E8: " .. v.name .. ": the camera pitch stays within [-30, 30] deg through the unmount while the game look reads -60, "
      .. "no pitch jump above 10 deg per 50 ms", lo >= -30 and hi <= 30 and maxA <= 10 + 8,
      string.format("pitch %.1f .. %.1f, max %.1f deg per 50 ms at %.2f", lo, hi, maxA, atA or -1))
  end
end)


-- ================================================================ 2026-11-03 jump-out landing (game log 16-28-47, JO1): the release blend (0.8 s from the
-- unmount + 0.5 s) ended the hold 0.15 s before the touchdown of a 1.3 s flight (launch 10.4 m/s, g -16) and the capsule pivot snapped
-- 1 m on the ascent and 1 - 3 times before the touchdown. Now the pivot follows V without a snap (camera_math pivotStep fast mode)
-- and the hold stays on while V is in flight (utpp_camera VehicleCompose landWait): it ends vehLandSettle s after the landing.
runBlock(function()
  local function stats(W, T)
    local land
    for i = 2, #W.track do
      if W.track[i].t > T.tu + 0.2 and W.track[i].pos.z <= 1e-9 and W.track[i - 1].pos.z > 1e-9 then land = W.track[i].t; break end
    end
    local byT = {}
    for _, k in ipairs(W.track) do byT[string.format("%.4f", k.t)] = k.pos end
    local relAt, offAt = firstAt(W, "vehicle release to orbit over"), firstAt(W, "vehicle hold OFF (release to orbit done)")
    -- per frame change of (camera z - V z) in the release and the 0.5 s after the hold, view angle change per frame
    local maxZ, atZ, maxA, atA = 0, nil, 0, nil
    for i = 2, #W.rec do
      local r, q = W.rec[i], W.rec[i - 1]
      local pr, pq = byT[string.format("%.4f", r.t)], byT[string.format("%.4f", q.t)]
      if pr and pq and relAt and offAt and r.t >= relAt and r.t <= offAt + 0.5 then
        local dz = math.abs((r.pos.z - pr.z) - (q.pos.z - pq.z))
        if dz > maxZ then maxZ, atZ = dz, r.t end
        local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(r.fwd), CM.norm(q.fwd))))))
        if ang > maxA then maxA, atA = ang, r.t end
      end
    end
    return land, relAt, offAt, maxZ, atZ, maxA, atA
  end
  local W, cam, vc, T = jumpOut(true, nil, 10.4, 16)
  local land, relAt, offAt, maxZ, atZ, maxA, atA = stats(W, T)
  check("jump-out flight 1.3 s: the release still starts at the unmount + 0.5 s", relAt and math.abs(relAt - (T.tu + 0.5)) <= 2 * DT, tostring(relAt))
  check("jump-out flight: the hold does not end within 0.15 s of the touchdown (ends 0.2 s after the landing)",
        land and offAt and offAt >= land + 0.15 and offAt <= land + 0.35 and count(W, "hold kept until") == 1
        and count(W, "vehicle release: hold ends") == 1, string.format("land %s hold OFF %s", tostring(land), tostring(offAt)))
  check("jump-out flight: camera relative to V (z) moves at most 0.15 m per frame through the release and 0.5 s after the hold",
        maxZ <= 0.15, string.format("%.3f m at %s", maxZ, tostring(atZ)))
  check("jump-out flight: no view jump through the release end / hold end (<= 3 deg per frame), no FPP, back in the orbit",
        maxA <= 3 and #W.fppActs == 0 and vhs(cam).held == false and cam.T.state == "tpp" and count(W, "FAILED") == 0,
        string.format("%.2f deg at %s", maxA, tostring(atA)))
  -- r13e (r13d jump-out game log 1468.69 - 1470.38: launch 9.75 m/s, g about 15.6 m/s^2, touchdown at 11.6 m/s): the rendered camera through the
  -- touchdown, the release end, the hold end and 0.5 s after: no frame moves more than 0.10 m (V stands still after the touchdown, so this is all
  -- camera motion) and no frame's step grows by more than 0.03 m over the one before (no kick at a mode switch), at 60 / 144 fps
  -- (+ the game log's order: there the release ended 0.10 s before the touchdown (1469.99 / 1470.09), so the landing ran on the orbit and the
  -- capsule pivot alone - replayed with the hold ended before the touchdown: vehLandHold = false and a longer flight, 12 m/s)
  local LANDC = { { fps = 60 }, { fps = 144 }, { fps = 60, cc = { vehLandHold = false }, vz = 12, pre = true }, { fps = 144, cc = { vehLandHold = false }, vz = 12, pre = true } }
  for _, lc in ipairs(LANDC) do
    local fps = lc.fps
    FPS, DT = fps, 1 / fps
    local Wl, _, _, Tl = jumpOut(true, nil, lc.vz or 9.75, 15.6, lc.cc)
    FPS, DT = 60, 1 / 60
    local landL, _, offL = stats(Wl, Tl)
    local w0, w1 = landL, offL and offL + 0.5
    -- (the pre case ends before the own pitch hands over to the game's look, offL + 0.5 s: a glide of the look state, about 0 deg in the logged
    -- run - pitch -14.3 through 1470.94 - but 6.5 deg onto the replay's look pitch; and its step growth is the camera's own (V stops dead at the
    -- touchdown, the step beyond hers grows without any camera kick)
    if lc.pre then w0, w1 = offL, (landL and offL) and math.min(landL + 0.5, offL + 0.5 - 1 / fps) end
    local maxS, atS, maxK, atK, prevS, maxR, atR = 0, nil, 0, nil, nil, 0, nil
    for i = 2, #Wl.rec do
      local a, b = Wl.rec[i - 1], Wl.rec[i]
      if w0 and w1 and b.t >= w0 - 1e-6 and b.t <= w1 then
        local s = math.max(0, CM.dist(a.pos, b.pos) - CM.dist(a.ppos, b.ppos)) * (1 / 60) / (b.t - a.t)
        if s > maxS then maxS, atS = s, b.t end
        local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd)))))) * (1 / 60) / (b.t - a.t)
        if ang > maxR then maxR, atR = ang, b.t end
        local g = lc.pre and CM.dist(a.pos, b.pos) * (1 / 60) / (b.t - a.t) or s
        if prevS and math.abs(g - prevS) > maxK and (lc.pre or g - prevS > 0) then maxK, atK = math.abs(g - prevS), b.t end
        prevS = g
      end
      if os.getenv("UTPP_R13E_DEBUG") and landL and b.t >= landL - 0.25 and b.t <= landL + 0.4 then
        print(string.format("jdbg %d %.3f cam=(%.3f,%.3f,%.3f) step=%.3f V=(%.3f,%.3f,%.3f) held=%s piv=%s base=%s jo=%s pitch=%.2f arm=%.3f", fps, b.t, b.pos.x, b.pos.y, b.pos.z, CM.dist(a.pos, b.pos),
          b.ppos.x, b.ppos.y, b.ppos.z, tostring(b.held), b.piv and string.format("%.3f,%.3f,%.3f", b.piv.x, b.piv.y, b.piv.z) or "-", tostring(b.pbase), tostring(b.joOn), select(2, yawPitch(b.fwd)), b.piv and CM.dist(b.pos, b.piv) or -1))
      end
    end
    check(string.format("r13e jump-out landing %d fps%s: step beyond V's own <= 0.10 m per 60 fps frame (%.3f at %s), no step growth > 0.03 m (%.3f at %s), rotation <= 1.0 deg (%.2f at %s)",
      fps, lc.pre and ", hold ended before the touchdown: the hold end -> the own pitch hand-over (0.5 s later; the camera's own step change)" or ": touchdown -> 0.5 s after the hold", maxS, tostring(atS), maxK, tostring(atK), maxR, tostring(atR)),
      landL ~= nil and offL ~= nil and (not lc.pre or offL < landL) and maxS <= 0.10 and maxK <= 0.03 and maxR <= 1.0,
      string.format("land %s off %s", tostring(landL), tostring(offL)))
  end
  -- negative control: the fixed end (vehLandHold = false) ends the hold at the touchdown (the reported coincidence)
  local W2, _, _, T2 = jumpOut(true, nil, 10.4, 16, { vehLandHold = false })
  local land2, _, off2 = stats(W2, T2)
  check("jump-out flight negative control: vehLandHold = false -> the hold ends within 0.1 s of the touchdown (as reported)",
        land2 and off2 and math.abs(off2 - land2) <= 0.1 and count(W2, "hold kept until") == 0, string.format("land %s hold OFF %s", tostring(land2), tostring(off2)))
  -- a low hop keeps the old timing (no wait)
  local W3, _, _, T3 = jumpOut(true)
  local _, rel3, off3 = stats(W3, T3)
  check("jump-out low hop (1.5 m/s): hold timing unchanged (release 0.8 s after its start, no wait)",
        rel3 and off3 and math.abs(off3 - (rel3 + 0.8)) <= 2 * DT and count(W3, "hold kept until") == 0, string.format("%s / %s", tostring(rel3), tostring(off3)))
end)

-- 2026-11-05 r13g jump-out landing framing through the real camera path (cfg.jumpOutFrame, CM.jumpOutFrameStep; replay of the game logs: test_utpp_jumpout_replay.lua).
-- The tilt is in res.shownPitch of the orbit frames and comes into the release blend with its weight (VehicleCompose): the release end while V still falls
-- (the logged order: blend over 0.07 s before the touchdown) and the hold end must not step the rendered pitch, and the tilt never reaches the look state
-- (the orbit pitch after the hold is the one without it). Measured on the rendered camera: the framing error of V's chest (V + 1.3 m; its elevation from the
-- camera against the same point over the pivot base, minus the written tilt) and the pitch rate from the release start to 0.6 s after the touchdown.
-- ground = true: the spatial queries hit the ground plane z = 0 (the game: the landing brake and the look ahead have a ground estimate); without it (the harness
-- default, no ray hit) the base falls on the no-ground spring and the tilt has no look ahead: there it must at least halve the framing error.
runBlock(function()
  local function pitchOfF(f) f = CM.norm(f); return math.deg(math.asin(math.max(-1, math.min(1, f.z)))) end
  local CASES = { { fps = 60, ground = true }, { fps = 144, ground = true }, { fps = 60, ground = true, cc = { vehLandHold = false }, vz = 12, pre = true },
                  { fps = 144, ground = true, cc = { vehLandHold = false }, vz = 12, pre = true }, { fps = 60 }, { fps = 144 }, { fps = 60, cc = { vehLandHold = false }, vz = 12, pre = true } }
  local function one(lc, extra)
    FPS, DT = lc.fps, 1 / lc.fps
    local cc = {}
    for k, v in pairs(lc.cc or {}) do cc[k] = v end
    for k, v in pairs(extra or {}) do cc[k] = v end
    local ex = nil
    if lc.ground then
      ex = { before = function(w)
        local sqs = {}
        function sqs:SyncRaycastByCollisionGroup(from, to)
          if from.z >= 0 and to.z < 0 then
            local k = from.z / (from.z - to.z)
            return true, { position = { x = from.x + (to.x - from.x) * k, y = from.y + (to.y - from.y) * k, z = 0 } }
          end
          return false, { position = { x = 0, y = 0, z = 0 } }
        end
        w.env.Game.GetSpatialQueriesSystem = function() return sqs end
      end }
    end
    local res = { pcall(jumpOut, true, nil, lc.vz or 9.75, 15.6, cc, ex) }
    FPS, DT = 60, 1 / 60
    if not res[1] then error(res[2]) end
    return res[2], res[5]
  end
  local function stats(W, T, pre, fps)
    local land
    for i = 2, #W.track do
      if W.track[i].t > T.tu + 0.2 and W.track[i].pos.z <= 1e-9 and W.track[i - 1].pos.z > 1e-9 then land = W.track[i].t; break end
    end
    local relAt, offAt = firstAt(W, "vehicle release to orbit over"), firstAt(W, "vehicle hold OFF (release to orbit done)")
    -- (as r13e: with the hold ended before the touchdown the window ends before the own pitch hands over to the game's look, 0.5 s after the hold - a glide of
    -- the replay's look state, about 0 deg in the logged run)
    local w1 = land and land + 0.6
    if pre and offAt and w1 then w1 = math.min(w1, offAt + 0.5 - 1 / fps) end
    local m = { dev = 0, devAt = nil, rot = 0, rotAt = nil, rotL = 0, rotLAt = nil, jp = 0, land = land, relAt = relAt, offAt = offAt }
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if land and relAt and b.t >= relAt and b.t <= w1 and b.pbase and b.ppos then
        local hd = math.sqrt((b.ppos.x - b.pos.x) ^ 2 + (b.ppos.y - b.pos.y) ^ 2)
        local e = math.deg(math.atan2(b.ppos.z + 1.3 - b.pos.z, hd) - math.atan2(b.pbase + 1.3 - b.pos.z, hd))
        if math.abs(e - (b.jp or 0)) > m.dev then m.dev, m.devAt = math.abs(e - (b.jp or 0)), b.t end
        m.jp = math.max(m.jp, math.abs(b.jp or 0))
        local r = math.abs(pitchOfF(b.fwd) - pitchOfF(a.fwd)) / (b.t - a.t)
        if r > m.rot then m.rot, m.rotAt = r, b.t end
        if b.t >= land - 1e-6 and r > m.rotL then m.rotL, m.rotLAt = r, b.t end -- (from the touchdown on: the r13e window)
      end
    end
    m.after = offAt and pitchOfF(recAt(W, offAt + 0.3).fwd) or nil
    m.afterJp = offAt and (recAt(W, offAt + 0.3).jp or 0) or nil
    return m
  end
  for _, lc in ipairs(CASES) do
    local W, T = one(lc)
    local Wo, To = one(lc, { jumpOutFrame = false })
    local m, o = stats(W, T, lc.pre, lc.fps), stats(Wo, To, lc.pre, lc.fps)
    local name = string.format("%d fps, %s%s", lc.fps, lc.ground and "ground ray" or "no ground ray", lc.pre and ", hold ended before the touchdown" or "")
    print(string.format("  jump-out landing framing (camera path) %s: framing error %.2f deg at %s (jumpOutFrame = false %.2f), tilt peak %.2f, pitch rate %.0f deg/s at %s (off: %.0f), release %s land %s hold OFF %s, pitch 0.3 s after the hold %.2f (off %.2f)",
      name, m.dev, tostring(m.devAt), o.dev, m.jp, m.rot, tostring(m.rotAt), o.rot, tostring(m.relAt), tostring(m.land), tostring(m.offAt), m.after or -99, o.after or -99))
    local lim = lc.ground and 6 or 0.5 * o.dev
    check(string.format("jump-out landing framing (camera path) %s: the tilt acts (%.2f deg); the framing error %.2f deg <= %s (without the tilt %.2f)", name, m.jp, m.dev,
          lc.ground and "6 deg" or string.format("half the one without the tilt, %.2f", lim), o.dev),
          m.land ~= nil and m.jp > 1 and m.dev <= lim and o.jp == 0)
    check(string.format("jump-out landing framing (camera path) %s: no pitch step through the release, its end and the hold end (%.0f deg/s <= 96, at %s); from the touchdown on <= 60 deg/s (r13e: 1.0 deg per 60 fps frame; %.0f at %s)",
          name, m.rot, tostring(m.rotAt), m.rotL, tostring(m.rotLAt)),
          m.rot <= 96 and m.rotL <= 60 and count(W, "FAILED") == 0 and #W.fppActs == 0 and vhs(W.mods.cam).held == false)
    check(string.format("jump-out landing framing (camera path) %s: the tilt stays out of the look state - 0.3 s after the hold the pitch minus the written tilt is the one without it (%.2f - %.2f / %.2f)", name, m.after or -99, m.afterJp or -99, o.after or -99),
          m.after ~= nil and o.after ~= nil and math.abs(m.after - m.afterJp - o.after) <= 0.05)
  end
end)

-- 2026-11-03 JO2 (game log 16-28-47 :5467-5486): a re-mount 0.7 s into the release to the orbit of the previous exit. HoldForVehicle(true)
-- answered "already held", then the release_end of the old release switched the hold off 0.08 s later: the new ride had no hold
-- (SetPoseOverride=FAILED not held) and the next exit ran without one. The running release is cancelled by the new mount.
runBlock(function()
  local W = makeWorld({})
  local cam = load(W)
  run(W, 7.0)
  local ok1, ok2, why2
  withEnv(W, function() ok1 = cam.HoldForVehicle(true, "test mount 1") end)
  run(W, W.t + 0.5)
  withEnv(W, function() ok2 = cam.ReleaseToOrbit(0.8, "test exit", true) end)
  run(W, W.t + 0.7)
  local relRunning = cam.vh.release ~= nil
  withEnv(W, function() ok2, why2 = cam.HoldForVehicle(true, "test mount 2") end)
  run(W, W.t + 1.5)
  check("JO2: a mount 0.7 s into a running release to the orbit cancels the release and keeps the hold (no hold OFF after the old release_end)",
        ok1 == true and relRunning and ok2 == true and why2 == "already held" and cam.vh.held == true and cam.vh.release == nil
        and count(W, "vehicle hold OFF (release to orbit done)") == 0 and count(W, "is cancelled by the new mount") == 1,
        string.format("held=%s release=%s hold OFF lines %d", tostring(cam.vh.held), tostring(cam.vh.release), count(W, "vehicle hold OFF (release to orbit done)")))
  -- control: without a second mount the release ends the hold as before
  local W2 = makeWorld({})
  local cam2 = load(W2)
  run(W2, 7.0)
  withEnv(W2, function() cam2.HoldForVehicle(true, "test mount 1") end)
  run(W2, W2.t + 0.5)
  withEnv(W2, function() cam2.ReleaseToOrbit(0.8, "test exit", true) end)
  run(W2, W2.t + 2.2)
  check("JO2 control: without the second mount the release ends the hold (hold OFF once)", cam2.vh.held == false
        and count(W2, "vehicle hold OFF (release to orbit done)") == 1)
end)

-- 2026-11-03 review r2 #11: the re-mount cancel of a running release must not step the shown pose from the half-blended pose to the release
-- target. The game's look pitch is -60 (the exit animation) while the release target is clamped to -25 (vehReleasePitchMin), so the blend has a
-- real pitch / position gap; the second mount comes 0.3 s into the 0.8 s release (smoothstep w ~ 0.2). The pose written the frame before and the
-- frames after differ only by the blend's own motion (<= 0.4 m and <= 4 deg per frame); on the old code the first frame after the mount jumped by
-- (1 - w) of the gap (about 28 deg).
runBlock(function()
  local W = makeWorld({})
  local cam = load(W)
  local function look(w) w.p.fppPitch = -60 end
  run(W, 7.0)
  withEnv(W, function() cam.HoldForVehicle(true, "test mount 1") end)
  run(W, W.t + 0.2, look)
  cam.S.look.pitch = -60 -- the orbit looks steeply down (the exit animation's look)
  run(W, W.t + 0.3, look)
  local pitchRel
  withEnv(W, function() cam.ReleaseToOrbit(0.8, "test exit", true); pitchRel = cam.vh.release and cam.vh.release.from.pitch end)
  run(W, W.t + 0.3, look)
  local tMount = W.t
  withEnv(W, function() cam.HoldForVehicle(true, "test mount 2") end)
  run(W, W.t + 1.2, look)
  local maxD, maxA, atD, atA = 0, 0, nil, nil
  for i = 2, #W.rec do
    local a, b = W.rec[i - 1], W.rec[i]
    if a.t >= tMount - 0.1 and b.t <= tMount + 1.0 then
      local d = CM.dist(a.pos, b.pos)
      if d > maxD then maxD, atD = d, b.t end
      local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd))))))
      if ang > maxA then maxA, atA = ang, b.t end
    end
  end
  check("re-mount cancel (r2 #11): no view step through the cancel (<= 0.4 m and <= 4 deg per frame), the blend finishes, the hold stays on",
        pitchRel ~= nil and pitchRel < -40 and maxD <= 0.4 and maxA <= 4 and cam.vh.held == true and cam.vh.release == nil
        and count(W, "vehicle hold OFF (release to orbit done)") == 0 and count(W, "is cancelled by the new mount") == 1,
        string.format("release from pitch %s, max step %.3f m at %s, %.2f deg at %s, held=%s release=%s", tostring(pitchRel), maxD, tostring(atD), maxA,
          tostring(atA), tostring(cam.vh.held), tostring(cam.vh.release)))
end)

-- 2026-11-03 review r2 #12: orbit frames of the hold report the SHOWN pitch (res.shownPitch: look pitch + the ragdoll boom bias), so vh.last, the
-- release_end write and OrbitPose do not differ from res.q by the bias
runBlock(function()
  local W = makeWorld({})
  local cam = load(W)
  local out = {}
  withEnv(W, function()
    cam.vh.held = true
    local res = { world = CM.v3(1, 2, 3), yaw = 20, pitch = -5, shownPitch = -15 }
    local _, y, p, kind = cam.VehicleCompose(res, 1.0)
    out.y, out.p, out.kind = y, p, kind
    cam.vh.landWait = { since = 0, airAt = 0 }
    local _, _, p2, kind2 = cam.VehicleCompose(res, 2.0)
    out.p2, out.kind2 = p2, kind2
    local _, _, p3 = cam.VehicleCompose({ world = CM.v3(1, 2, 3), yaw = 20, pitch = -5 }, 3.0)
    out.p3 = p3
  end)
  check("orbit frame while held (r2 #12): the shown pitch, not the look pitch", out.kind == "orbit" and out.y == 20 and out.p == -15, tostring(out.p))
  check("landWait end (r2 #12): release_end carries the shown pitch", out.kind2 == "release_end" and out.p2 == -15, tostring(out.p2) .. " " .. tostring(out.kind2))
  check("orbit frame without a ragdoll bias (r2 #12): pitch as before", out.p3 == -5, tostring(out.p3))
end)

-- ================================================================ 2026-11-04 jump-out landing mode: the arming comes from the jump-out classification only
-- (review finding 9 and the missing pass-through test). utpp_vehiclecam classifies the exit by the vehicle speed (exitFollowSpeed) and tells utpp_camera
-- (VehicleJumpOut); utpp_camera passes it to CM.solve as the input jumpOut (the pivot's jump-out mode, CM.jumpOutStep). A normal exit from a standing
-- vehicle must never arm it and must be bit for bit the path without the mode (cfg.jumpOutGov = false).
runBlock(function()
  -- spy on CM.solve (init.lua calls it through the shared module table): the jumpOut input and the pivot mode state of every frame
  local function spy(W, cam, store)
    local orig = cam.CM.solve
    cam.CM.solve = function(S, inp)
      local r = orig(S, inp)
      store[#store + 1] = { t = W.t, jo = inp.jumpOut, on = S.pivot.joOn == true }
      return r
    end
  end
  local function sameRec(A, B)
    if #A.rec ~= #B.rec then return false, "frame count " .. #A.rec .. " vs " .. #B.rec end
    for i = 1, #A.rec do
      local a, b = A.rec[i], B.rec[i]
      if a.pos.x ~= b.pos.x or a.pos.y ~= b.pos.y or a.pos.z ~= b.pos.z or a.fwd.x ~= b.fwd.x or a.fwd.y ~= b.fwd.y or a.fwd.z ~= b.fwd.z then
        return false, string.format("frame %d t=%.3f differs", i, a.t)
      end
    end
    return true
  end
  local function nTrue(store, key, t0, t1)
    local n = 0
    for _, s in ipairs(store) do if s[key] == true and (t0 == nil or s.t >= t0) and (t1 == nil or s.t <= t1) then n = n + 1 end end
    return n
  end
  -- (1) pass-through: the exit hold flag of a jump-out becomes the jumpOut input of CM.solve, through the TPP ride (TakeFromVehicle path) and the exit
  -- from the game camera (TakeForVehicleExit path)
  local variants = { { name = "TPP ride", world = {}, tl = {}, takes = 0 },
                     { name = "exit from the game camera", world = { perspExitDelay = 0.4 }, tl = {}, fpp = 2.0, takes = 1 } }
  for _, vr in ipairs(variants) do
    local st = {}
    local W, cam, vc, T = jumpOut(true, nil, 10.4, 16, nil, { world = vr.world, tl = vr.tl, userFppAt = vr.fpp, before = function(w, c) spy(w, c, st) end })
    local offAt = 1e9 -- the first hold OFF after the exit start (the hand-over of the ride ends an earlier hold)
    for _, l in ipairs(W.lines) do
      local lt = tonumber(l:sub(1, 7)) or 0
      if lt >= T.tx and string.find(l, "vehicle hold OFF", 1, true) then offAt = lt; break end
    end
    local firstOn
    for _, s in ipairs(st) do if s.jo == true then firstOn = s.t; break end end
    check(string.format("jump-out pass-through (%s): the jump-out classification arms the mode once, at the exit start (first jumpOut input %.2f, exit start %.2f)",
          vr.name, firstOn or -1, T.tx), count(W, "vehicle jump-out landing mode armed") == 1 and count(W, "vehicle exit take-over (") == vr.takes and firstOn ~= nil and firstOn >= T.tx - 0.05 and firstOn <= T.tx + 0.3)
    check(string.format("jump-out pass-through (%s): jumpOut is never true before the exit, true through the hold, false after the hold ended", vr.name),
          nTrue(st, "jo", nil, T.tx - 0.05) == 0 and nTrue(st, "jo", firstOn or 0, offAt - 0.05) > 20 and nTrue(st, "jo", offAt + 0.02, nil) == 0,
          string.format("before %d, in hold %d, after %d, hold off at %.2f", nTrue(st, "jo", nil, T.tx - 0.05), nTrue(st, "jo", firstOn or 0, offAt - 0.05),
            nTrue(st, "jo", offAt + 0.02, nil), offAt))
    check(string.format("jump-out pass-through (%s): the pivot's jump-out mode ran (%d frames) and is over at the end", vr.name, nTrue(st, "on")),
          nTrue(st, "on") > 20 and cam.S.pivot.joOn == nil)
  end
  -- (2) a normal exit (vehicle standing): the mode is never armed and the whole camera path is bit for bit the path with the mode disabled. A jump-out
  -- differs between the two (positive control: the comparison can see the mode)
  local normals = { { name = "TPP ride", opts = {} }, { name = "exit from the game camera", opts = { userFppAt = 2.0, perspExitDelay = 0.4 } } }
  for _, nv in ipairs(normals) do
    local runs = {}
    for _, gov in ipairs({ true, false }) do
      local st = {}
      local o = {}
      for k, v in pairs(nv.opts) do o[k] = v end
      o.before = function(w, c) c.cfg.jumpOutGov = gov; spy(w, c, st) end
      local W = session(o)
      runs[gov and "on" or "off"] = { W = W, st = st }
    end
    local same, why = sameRec(runs.on.W, runs.off.W)
    check(string.format("normal exit (%s): jumpOut is never true, the mode never runs, no jump-out arming line", nv.name),
          nTrue(runs.on.st, "jo") == 0 and nTrue(runs.on.st, "on") == 0 and count(runs.on.W, "vehicle jump-out landing mode") == 0 and #runs.on.st > 100)
    check(string.format("normal exit (%s): the rendered camera is bit for bit the one with jumpOutGov = false (%d frames)", nv.name, #runs.on.W.rec), same, why)
  end
  do
    local recs = {}
    for _, gov in ipairs({ true, false }) do
      recs[#recs + 1] = jumpOut(true, nil, 10.4, 16, { jumpOutGov = gov })
    end
    local same = sameRec(recs[1], recs[2])
    check("jump-out positive control: with the mode on the rendered camera differs from jumpOutGov = false (the bit-identical check above can fail)", not same)
  end
  -- 2026-11-04 r13 pivot xy follower (jumpOutXY) through the real camera path: the pivot's offset to V per frame (spy on CM.solve), the hand-overs (release end,
  -- hold off, own pitch on, hold -> free) and the end of the mode. A normal exit never arms it (bit for bit); jumpOutXY = false is the pivot copying V.
  do
    local function xyRun(fps, over)
      FPS, DT = fps, 1 / fps
      local rows = {}
      local origSolve, camMod -- (CM is one table shared by all worlds: the spy must be taken off again, or it records the next world's solves as well)
      local res = { pcall(function()
        return jumpOut(true, nil, 10.4, 16, over, { tl = { tm = math.max(7, 330 / fps + 0.5) }, before = function(w, c)
          local orig = c.CM.solve
          origSolve, camMod = orig, c
          c.CM.solve = function(S, inp)
            local r = orig(S, inp)
            local pv = r and r.pivot
            local pp = inp.playerPos
            local X = S.pivot.xyJ
            if pv and pp then rows[#rows + 1] = { t = w.t, ox = pv.x - pp.x, oy = pv.y - pp.y, st = X ~= nil, jo = S.pivot.joOn == true, held = inp.jumpOut ~= nil, dt = inp.dt,
              ux = X and X.ux, uy = X and X.uy, fx = X and X.fx, fy = X and X.fy, px = pp.x, py = pp.y } end
            return r
          end
        end })
      end) }
      FPS, DT = 60, 1 / 60
      if camMod then camMod.CM.solve = origSolve end
      if not res[1] then error(res[2]) end
      return res[2], rows, res[3]
    end
    for _, fps in ipairs({ 30, 60, 144 }) do
      local W, rows, cam = xyRun(fps)
      local Wo, rowsO = xyRun(fps, { jumpOutXY = false })
      local lag, lagO, maxStep, atT, onFrames, lastSt = 0, 0, 0, nil, 0, nil
      for i, r in ipairs(rows) do
        lag = math.max(lag, math.sqrt(r.ox ^ 2 + r.oy ^ 2))
        if r.st then onFrames = onFrames + 1 end
        if i > 1 then
          local st = math.sqrt((r.ox - rows[i - 1].ox) ^ 2 + (r.oy - rows[i - 1].oy) ^ 2)
          if st > maxStep then maxStep, atT = st, r.t end
        end
      end
      for _, r in ipairs(rowsO) do lagO = math.max(lagO, math.sqrt(r.ox ^ 2 + r.oy ^ 2)) end
      local last = rows[#rows]
      local sameAsOff = #W.rec == #Wo.rec
      for i = 1, sameAsOff and #W.rec or 0 do
        local a, b2 = W.rec[i], Wo.rec[i]
        if a.pos.x ~= b2.pos.x or a.pos.y ~= b2.pos.y or a.pos.z ~= b2.pos.z then sameAsOff = false; break end
      end
      print(string.format("  jump-out pivot xy (full camera path), %d fps: offset to V max %.3f m (jumpOutXY = false %.3f), largest per-frame offset change %.3f m at %.2f s, follower frames %d, end offset %.5f m",
        fps, lag, lagO, maxStep, atT or -1, onFrames, last and math.sqrt(last.ox ^ 2 + last.oy ^ 2) or -1))
      check(string.format("jump-out pivot xy, %d fps: the follower ran (%d frames) and the offset to V stayed inside jumpOutXYLagHard (%.3f <= 1.0 m); jumpOutXY = false keeps the pivot on V (%.3f m)", fps, onFrames, lag, lagO),
            onFrames > 10 and lag > 0.02 and lag <= cam.CM.config().jumpOutXYLagHard + 1e-6 and lagO < 1e-9)
      local X, pvS = cam.S.pivot.xyJ, cam.S.pivot
      local endOff = X and math.sqrt((X.cx - pvS.xyLx) ^ 2 + (X.cy - pvS.xyLy) ^ 2) or 0
      check(string.format("jump-out pivot xy, %d fps: the mode is over at the end and the follower (if V still creeps) sits on V within 2 cm (%.4f m)", fps, endOff),
            pvS.joOn == nil and endOff < 0.02)
      check(string.format("jump-out pivot xy, %d fps: the rendered camera differs from the pivot-on-V one (positive control)", fps), not sameAsOff)
    end
    local recs = {}
    for _, xy in ipairs({ true, false }) do
      local o = {}
      o.before = function(w, c) c.cfg.jumpOutXY = xy end
      recs[#recs + 1] = session(o)
    end
    local sameN, whyN = sameRec(recs[1], recs[2])
    check("normal exit (standing vehicle): the rendered camera is bit for bit the one with jumpOutXY = false", sameN and #recs[1].rec > 100, whyN)
  end
  -- (3) an older utpp_camera without VehicleJumpOut: the command is logged as unavailable, the exit runs on as before (no error)
  do
    local W, cam = jumpOut(true, nil, 10.4, 16, nil, { before = function(w, c) c.VehicleJumpOut = nil end })
    check("jump-out with a camera without VehicleJumpOut: logged as unavailable, no mode, no update error", count(W, "VehicleJumpOut(true)=FAILED VehicleJumpOut unavailable") == 1
      and count(W, "vehicle jump-out landing mode") == 0 and cam.S.pivot.joOn == nil)
  end
end)

-- ================================================================ 2026-11-04 jump-out from a standing vehicle (double F): the native launch arms the mode
-- The exit of a standing or slow vehicle is no jump-out by the speed classification, but the double F launches V all the same. The native counts the
-- launch (Game.UTPPVehicleJumpOutEpoch, MMDriver jump-out edge); utpp_vehiclecam polls it while the exit hold is up and sends the same jumpout command.
runBlock(function()
  local function spy(W, cam, store)
    local orig = cam.CM.solve
    cam.CM.solve = function(S, inp)
      local r = orig(S, inp)
      store[#store + 1] = { t = W.t, jo = inp.jumpOut, on = S.pivot.joOn == true, base = type(S.pivot.base) == "number" and S.pivot.base or nil }
      return r
    end
  end
  local function nTrue(store, key, t0, t1)
    local n = 0
    for _, s in ipairs(store) do if s[key] == true and (t0 == nil or s.t >= t0) and (t1 == nil or s.t <= t1) then n = n + 1 end end
    return n
  end
  local function sameRec(A, B)
    if #A.rec ~= #B.rec then return false, "frame count " .. #A.rec .. " vs " .. #B.rec end
    for i = 1, #A.rec do
      local a, c = A.rec[i], B.rec[i]
      if a.pos.x ~= c.pos.x or a.pos.y ~= c.pos.y or a.pos.z ~= c.pos.z or a.fwd.x ~= c.fwd.x or a.fwd.y ~= c.fwd.y or a.fwd.z ~= c.fwd.z then
        return false, string.format("frame %d t=%.3f differs", i, a.t)
      end
    end
    return true
  end
  -- max vertical acceleration of the pivot base (pv.base is its height; second difference of the recorded base) over [t0, t1]
  local function maxAccel(store, t0, t1)
    local m, at = 0, nil
    for i = 3, #store do
      local a, c, d = store[i - 2], store[i - 1], store[i]
      if d.t >= t0 and d.t <= t1 and a.base and c.base and d.base then
        local acc = math.abs(d.base - 2 * c.base + a.base) / (DT * DT)
        if acc > m then m, at = acc, d.t end
      end
    end
    return m, at
  end
  local function setCfg(vc, cfg) for k, val in pairs(cfg or {}) do vc.ctrl.cfg[k] = val end end
  local function run1(opts) -- opts: native (false = no native), launch (the native counts a launch), cfg (utpp_vehiclecam config override)
    local st = {}
    local ex = { speed = 0, launchAfter = opts.launch and 0.06 or nil, before = function(w, c, v)
      spy(w, c, st)
      setCfg(v, opts.cfg)
      if opts.native ~= false then w.env.Game.UTPPVehicleJumpOutEpoch = function() return w.joEpoch or 0 end end
    end }
    local W, cam, vc, T = jumpOut(true, 1.0, 9.8, 16, nil, ex)
    return W, cam, vc, T, st
  end
  -- (1) standing vehicle, double F: the mock native reports the launch 0.06 s after the unmount (log: 0.058 s after the workspot drop)
  local W, cam, vc, T, st = run1({ launch = true })
  local launchAt = T.tu + 0.06
  local firstOn
  for _, s in ipairs(st) do if s.jo == true then firstOn = s.t; break end end
  check("standing double-F exit: the speed classification does not arm it (no 'exit while the vehicle moves' line)", count(W, "exit while the vehicle moves") == 0)
  check("standing double-F exit: the native launch arms the jump-out mode once, within 3 frames of the launch, with the native reason",
        count(W, "vehicle jump-out landing mode armed") == 1 and count(W, "jump-out (native launch)") >= 1
        and count(W, "jump-out launch reported by the native (epoch 1") == 1 and firstOn ~= nil and firstOn >= launchAt - 1e-6 and firstOn <= launchAt + 3 * DT,
        string.format("first jumpOut input %s, launch %.3f", tostring(firstOn), launchAt) .. "\n" .. dump(W, { "jump-out", "jumpout" }))
  check("standing double-F exit: jumpOut is false until the launch and the pivot's mode runs after it", nTrue(st, "jo", nil, launchAt - 0.02) == 0
        and nTrue(st, "jo", launchAt, nil) > 10 and nTrue(st, "on") > 10 and cam.S.pivot.joOn == nil, string.format("pre %d, post %d, on %d",
        nTrue(st, "jo", nil, launchAt - 0.02), nTrue(st, "jo", launchAt, nil), nTrue(st, "on")))
  -- the landing follows the governor limits: the pivot base never accelerates faster than jumpOutAup (+ the smoothing margin) through the flight and landing
  local lim = (cam.cfg.jumpOutAup or 80) * 1.25
  local aOn, atOn = maxAccel(st, launchAt, T.te)
  check(string.format("standing double-F exit: the pivot base acceleration stays within the governor limit (%.0f <= %.0f m/s^2)", aOn, lim), aOn <= lim,
        string.format("%.1f m/s^2 at %s", aOn, tostring(atOn)))
  check("standing double-F exit: no error, hold released to the orbit, no FPP", count(W, "FAILED") == 0 and vhs(cam).held == false and cam.T.state == "tpp" and #W.fppActs == 0)
  -- (2) mutation: the native ignored (exitJumpNative = false) -> not armed, the rigid fast-mode follow breaks the limit at the landing
  local Wm, _, _, Tm, stm = run1({ launch = true, cfg = { exitJumpNative = false } })
  local aOff, atOff = maxAccel(stm, Tm.tu + 0.06, Tm.te)
  check("standing double-F exit mutation: native ignored -> the mode is never armed", nTrue(stm, "jo") == 0 and count(Wm, "vehicle jump-out landing mode") == 0)
  check(string.format("standing double-F exit mutation: the unarmed rigid follow exceeds the governor limit (%.0f > %.0f m/s^2), so the limit check above can fail", aOff, lim),
        aOff > lim, string.format("%.1f m/s^2 at %s", aOff, tostring(atOff)))
  -- (3) a normal standing exit (no launch): bit for bit the path without the native (a mock that always answers 0, no native at all, exitJumpNative = false)
  local normal = { { name = "mock native, no launch", o = { native = true } }, { name = "no native (older DLL)", o = { native = false } },
                   { name = "native disabled", o = { native = true, cfg = { exitJumpNative = false } } } }
  local runs = {}
  for i, nv in ipairs(normal) do
    local st2 = {}
    local Wn = session({ before = function(w, c, v)
      spy(w, c, st2)
      if nv.o.native then w.env.Game.UTPPVehicleJumpOutEpoch = function() return 0 end end
      setCfg(v, nv.o.cfg)
    end })
    runs[i] = { W = Wn, st = st2 }
    check(string.format("normal standing exit (%s): jumpOut is never true, no jump-out arming line", nv.name),
          nTrue(st2, "jo") == 0 and nTrue(st2, "on") == 0 and count(Wn, "vehicle jump-out landing mode") == 0 and count(Wn, "launch reported by the native") == 0 and #st2 > 100)
  end
  for i = 2, #runs do
    local same, why = sameRec(runs[1].W, runs[i].W)
    check(string.format("normal standing exit: the rendered camera is bit for bit the same with '%s' (%d frames)", normal[i].name, #runs[1].W.rec), same, why)
  end
  check("older DLL (no native): logged once, no error", count(runs[2].W, "Game.UTPPVehicleJumpOutEpoch unavailable") == 1 and count(runs[2].W, "FAILED") == 0)
  -- (4) a native that raises: pcall, logged once, the exit runs on; no native in the jump-out scenario: not armed
  do
    local Wr = run1({ launch = true, native = false })
    local st3 = {}
    local Wf = jumpOut(true, 1.0, 9.8, 16, nil, { speed = 0, before = function(w, c)
      spy(w, c, st3); w.env.Game.UTPPVehicleJumpOutEpoch = function() error("boom") end end })
    check("native that raises: logged once, no mode, no error", count(Wf, "Game.UTPPVehicleJumpOutEpoch failed") == 1 and nTrue(st3, "jo") == 0 and count(Wf, "FAILED") == 0)
    check("no native (older DLL) in the jump-out scenario: not armed, no error", count(Wr, "Game.UTPPVehicleJumpOutEpoch unavailable") == 1
      and count(Wr, "vehicle jump-out landing mode armed") == 0 and count(Wr, "FAILED") == 0)
  end
  -- (5) a launch counted BEFORE the exit (stale epoch) is no new launch: the baseline is taken at the exit start
  do
    local st4 = {}
    local Ws = jumpOut(true, 1.0, 9.8, 16, nil, { speed = 0, before = function(w, c)
      spy(w, c, st4); w.joEpoch = 7; w.env.Game.UTPPVehicleJumpOutEpoch = function() return w.joEpoch or 0 end end })
    check("stale launch counter before the exit: not armed", nTrue(st4, "jo") == 0 and count(Ws, "vehicle jump-out landing mode") == 0)
  end
  -- (6) a moving vehicle keeps the speed classification (armed at the exit start, once); the later native launch adds nothing
  do
    local st5 = {}
    local Wj = jumpOut(true, nil, 10.4, 16, nil, { launchAfter = 0.06, before = function(w, c)
      spy(w, c, st5); w.env.Game.UTPPVehicleJumpOutEpoch = function() return w.joEpoch or 0 end end })
    check("moving vehicle jump-out with the native: still armed once, by the speed classification", count(Wj, "vehicle jump-out landing mode armed") == 1
      and count(Wj, "exit while the vehicle moves 12.0 m/s") == 1 and count(Wj, "launch reported by the native") == 0)
  end
end)

-- ================================================================ 2026-11-04 jump-out launch camera (game log 02-01-16, jumpout_move and jumpout_stand)
-- The shown camera rose at 15 - 17 m/s while V rose at 9.7 m/s (+92 / -120 m/s2) and pitched -8 -> +5 -> -14 deg within 0.35 s with a dead stop at the end:
-- (1) the pivot base overshoot of CM.jumpOutStep (own test in test_utpp_camera.lua, jumpOutCatch), (2) the collapse of the follow offset on top of V's climb
-- (OwnPose exitFollowMaxVz / exitFollowMaxAz), (3) the framing correction (VehicleCompose) went 0 -> 6 deg in 50 ms and stopped dead (vehFrameAccel).
-- The logged game ran at about 18 fps, so 20 fps is part of the sweep. Metrics are taken from a 20 Hz sampling of the rendered camera (as the game log).
runBlock(function()
  local function pitchOf(f) f = CM.norm(f); return math.deg(math.asin(f.z)) end
  local LAUNCH = 9.73
  -- one jump-out at fps: speed 8.2 = the moving vehicle (jump-out by the speed), 0 + native = the standing vehicle (launch edge of the native)
  local function launch(fps, speed, native, vcCfg, camCfg)
    FPS, DT = fps, 1 / fps
    local res = { pcall(function()
      return jumpOut(true, 1.0, LAUNCH, 16, camCfg, { rise = 2.4, speed = speed, tl = { tm = math.max(7, 330 / fps + 0.5) }, launchAfter = native and 0.05 or nil,
        before = function(w, c, v)
          w.env.Game.UTPPVehicleJumpOutEpoch = function() return w.joEpoch or 0 end
          for k, val in pairs(vcCfg or {}) do v.ctrl.cfg[k] = val end
        end })
    end) }
    FPS, DT = 60, 1 / 60
    if not res[1] then error(res[2]) end
    return res[2], res[3], res[4], res[5]
  end
  local function metrics(W, T)
    local S, R, j, t = {}, W.rec, 1, T.tu - 0.1
    while t <= T.tu + 1.0 + 1e-9 do
      while j < #R and R[j].t < t - 1e-9 do j = j + 1 end
      S[#S + 1] = R[j]
      t = t + 0.05
    end
    local m = { vz = 0, azUp = 0, azDown = 0, pr = 0, pa = 0, n = #S, bobUp = 0, bobDn = 0 }
    -- (2026-11-05: the pitch without the landing framing tilt of the descent, r.jp (cfg.jumpOutFrame; its own test: test_utpp_jumpout_replay.lua): these limits
    -- are the launch swing's)
    local function pitchL(r) return pitchOf(r.fwd) - (r.jp or 0) end
    local p0 = S[1] and pitchL(S[1]) or 0 -- the pitch before the unmount
    local vzp, prp, dtp
    for i = 2, #S do
      local a, b = S[i - 1], S[i]
      local dt = b.t - a.t
      local vz = (b.pos.z - a.pos.z) / dt
      local pr = (pitchL(b) - pitchL(a)) / dt
      if b.t >= T.tu then
        m.vz = math.max(m.vz, vz)
        m.pr = math.max(m.pr, math.abs(pr))
        m.bobUp, m.bobDn = math.max(m.bobUp, pitchL(b) - p0), math.min(m.bobDn, pitchL(b) - p0)
        if vzp then
          local dm = (dt + dtp) / 2 -- the change of the two interval speeds happens over the mean of the two intervals (a 30 fps frame grid is uneven at 20 Hz)
          m.azUp = math.max(m.azUp, (vz - vzp) / dm); m.azDown = math.min(m.azDown, (vz - vzp) / dm)
          m.pa = math.max(m.pa, math.abs((pr - prp) / dm))
        end
      end
      vzp, prp, dtp = vz, pr, dt
    end
    return m
  end
  local function fmt(m) return string.format("camera vz max %.1f, az %.0f .. %.0f, pitch rate %.0f deg/s, pitch accel %.0f deg/s2, pitch %+.1f .. %+.1f deg", m.vz, m.azDown, m.azUp, m.pr, m.pa, m.bobDn, m.bobUp) end
  -- limits: the camera never rises faster than V's launch speed (+1.5 m/s: the base is still catching up with a decelerating V), the acceleration stays at the
  -- order of the landing mode's aup (80, +25 % for the 20 Hz sampling), the pitch changes at most 70 deg/s with at most 1300 deg/s2 at the 20 Hz sampling
  -- The pitch excursion is guarded as well (up-swing <= 20 deg, no down-swing below the pre-launch pitch by more than 3 deg: the log had +13 / -19 deg). The harness
  -- reproduces the mechanism, not the amplitude: it shows +9 .. +12 deg (moving) and +14 .. +17 deg (standing) up, no down-swing, camera vz 8 .. 11 m/s (log: 15 - 17).
  local function ok(m) return m.vz <= LAUNCH + 1.5 and m.azUp <= 100 and m.azDown >= -110 and m.pr <= 70 and m.pa <= 1300 and m.bobUp <= 20 and m.bobDn >= -3 end
  local cases = { { "moving vehicle", 8.2, false }, { "standing vehicle", 0, true } }
  local mut = {
    { name = "no framing ease (vehFrameAccel = 0)", cam = { vehFrameAccel = 0, vehJumpFrame = false }, vc = nil, bad = function(m) return m.pr > 70 end }, -- (in the plain band: the widened jump-out band swings so little that the ease hardly matters)
    { name = "no vertical follow limiter (exitFollowMaxAz = 0)", cam = nil, vc = { exitFollowMaxAz = 0 }, bad = function(m) return m.azUp > 100 or m.vz > LAUNCH + 1.5 end },
  }
  local seen = {}
  for _, fps in ipairs({ 20, 30, 60, 144 }) do
    for _, cs in ipairs(cases) do
      local W, _, _, T = launch(fps, cs[2], cs[3])
      local m = metrics(W, T)
      check(string.format("jump-out launch camera, %s, %d fps: shown camera <= V's launch speed + 1.5 m/s, accel within 100 / -110 m/s2, pitch <= 70 deg/s and 1300 deg/s2, excursion +20 / -3 deg (%s)",
            cs[1], fps, fmt(m)), ok(m) and m.n > 15 and count(W, "exit follow (") == 1 and count(W, "FAILED") == 0, fmt(m))
      for _, mu in ipairs(mut) do
        local Wm, _, _, Tm = launch(fps, cs[2], cs[3], mu.vc, mu.cam)
        local mm = metrics(Wm, Tm)
        seen[mu.name] = (seen[mu.name] or 0) + (mu.bad(mm) and 1 or 0)
        print(string.format("  mutation '%s', %s, %d fps: %s", mu.name, cs[1], fps, fmt(mm)))
      end
    end
  end
  for _, mu in ipairs(mut) do
    check(string.format("jump-out launch camera mutation '%s' breaks the limits in at least 5 of the 8 cases (%d)", mu.name, seen[mu.name] or 0), (seen[mu.name] or 0) >= 5)
  end
  -- every mutation together = the previous behaviour: fails the limits in all cases
  for _, fps in ipairs({ 20, 60, 144 }) do
    for _, cs in ipairs(cases) do
      local Wm, _, _, Tm = launch(fps, cs[2], cs[3], { exitFollowMaxVz = 0, exitFollowMaxAz = 0 }, { vehFrameAccel = 0, jumpOutCatch = 0 })
      local mm = metrics(Wm, Tm)
      check(string.format("jump-out launch camera mutation (all three off = before), %s, %d fps: the limits are broken (%s)", cs[1], fps, fmt(mm)), not ok(mm), fmt(mm))
    end
  end
  -- a normal exit (standing vehicle, no launch) is bit for bit the same with every new limiter off
  do
    local recs = {}
    for _, off in ipairs({ false, true }) do
      recs[#recs + 1] = session({ before = function(_, c, v)
        if off then c.cfg.vehFrameAccel, c.cfg.jumpOutCatch = 0, 0; v.ctrl.cfg.exitFollowMaxVz, v.ctrl.cfg.exitFollowMaxAz = 0, 0 end
      end })
    end
    local same = #recs[1].rec == #recs[2].rec
    for i = 1, same and #recs[1].rec or 0 do
      local a, b = recs[1].rec[i], recs[2].rec[i]
      if a.pos.x ~= b.pos.x or a.pos.y ~= b.pos.y or a.pos.z ~= b.pos.z or a.fwd.x ~= b.fwd.x or a.fwd.y ~= b.fwd.y or a.fwd.z ~= b.fwd.z then same = false; break end
    end
    check("normal exit (standing vehicle): the rendered camera is bit for bit the one with the launch limiters off", same and #recs[1].rec > 100)
  end
  -- 2026-11-04 r13: the framing change of the launch follow relative to V is capped per second (exitJumpMaxSpeed, frame time aware) and the follow's yaw / pitch rate
  -- (exitJumpMaxRate). Measured per rendered frame between the start of the follow and the start of the release: the camera minus V's position. 0 = the common caps (before).
  do
    local function relSteps(fps, speed, native, vcCfg)
      local W, _, _, T = launch(fps, speed, native, vcCfg)
      local byT = {}
      for _, k in ipairs(W.track) do byT[string.format("%.4f", k.t)] = k.pos end
      local t0, t1 = firstAt(W, "exit follow ("), firstAt(W, "vehicle release to orbit over")
      local maxStep, maxDt, prev, prevPos = 0, 0, nil, nil
      for _, r in ipairs(W.rec) do
        local pp = byT[string.format("%.4f", r.t)]
        if pp and t0 and t1 and r.t >= t0 and r.t <= t1 then
          local rel = CM.sub(r.pos, pp)
          if prev then
            local st = CM.dist(rel, prev)
            local dt = r.t - prevPos
            if dt > 0 and st / dt > maxStep then maxStep, maxDt = st / dt, dt end -- m/s of the framing change
          end
          prev, prevPos = rel, r.t
        end
      end
      return maxStep, t0 ~= nil and t1 ~= nil
    end
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _, cs in ipairs({ { "moving vehicle", 8.2, false }, { "standing vehicle", 0, true } }) do
        local sp, okW = relSteps(fps, cs[2], cs[3])
        local spOff = relSteps(fps, cs[2], cs[3], { exitJumpMaxSpeed = 0, exitJumpMaxRate = 0 })
        local spLow = relSteps(fps, cs[2], cs[3], { exitJumpMaxSpeed = 4 })
        print(string.format("  launch follow, %s, %d fps: framing change relative to V max %.1f m/s (explicit 0 %.1f, cap 4 m/s %.1f)", cs[1], fps, sp, spOff, spLow))
        check(string.format("r13 launch follow, %s, %d fps: the caps are off by default (framing change relative to V %.2f = %.2f m/s with exitJumpMaxSpeed / Rate = 0)", cs[1], fps, sp, spOff),
              okW and sp == spOff)
        check(string.format("r13 launch follow, %s, %d fps: the cap bites - exitJumpMaxSpeed = 4 lowers the framing change relative to V by more than 3 m/s (%.1f -> %.1f)", cs[1], fps, spOff, spLow),
              spLow <= spOff - 3)
      end
    end
  end
end)

-- ================================================================ 2026-11-04 door pop in the mount hold (game log 02-01-16, ride 7 'behind' approach, 02:13:28.40 - 28.79)
-- V walks up to the rear of a standing car; 0.45 s after the mount start the front-left door swings open and crosses the camera boom for about 0.25 s
-- (hits=1, blocked=0): the boom went 2.93 -> 2.16 -> 1.27 m within 0.15 s and back (a V close-up for 0.3 s). The mount hold pulled the boom in at 12 m/s
-- (vehHoldColRate); a surface that is gone after 0.25 s is no occluder, so a pull-in now starts after vehHoldColDelay s of pressure and ramps up.
runBlock(function()
  -- door = { from, to } s after the mount start: every ray hits a surface 1.3 m from its start; cam0 = utpp_camera config override;
  -- rot = { from, to, deg/s } s after the mount start: mouse-look of that rate (CameraMouseX through the camera's input table) in the hold
  local function run(fps, door, cam0, tlo, rot)
    local tm = math.max(7, 330 / fps + 0.5)
    FPS, DT = fps, 1 / fps
    local lens = {}
    local ok, W, cam, vc, T = pcall(function()
      local w = makeWorld({})
      local c, v = load(w)
      for k, val in pairs(cam0 or {}) do c.cfg[k] = val end
      local sqs = {}
      function sqs:SyncRaycastByCollisionGroup(from, to)
        local mt = w.t - tm
        if door and mt >= door[1] and mt <= door[2] then
          local dx, dy, dz = to.x - from.x, to.y - from.y, to.z - from.z
          local l = math.sqrt(dx * dx + dy * dy + dz * dz)
          if l > 1.3 then return true, { position = { x = from.x + dx / l * 1.3, y = from.y + dy / l * 1.3, z = from.z + dz / l * 1.3 } } end
        end
        return false, { position = { x = 0, y = 0, z = 0 } }
      end
      w.env.Game.GetSpatialQueriesSystem = function() return sqs end
      local tlOpts = { tm = tm }
      for k, val in pairs(tlo or {}) do tlOpts[k] = val end
      local Tl = timeline(tlOpts)
      local base = Tl.script
      local script = function(ww, t)
        base(ww, t)
        local g = c.G
        if rot and t - tm >= rot[1] and t - tm <= rot[2] then
          c.input.mouseX = c.input.mouseX + rot[3] * DT / CM.mouseUnit(c.cal, c.cfg, c.rt.mouseScale)
        end
        if g and g.last and t >= tm - 0.2 and t <= tm + 1.0 then
          lens[#lens + 1] = { t = t - tm, arm = g.last.arm, full = g.last.fullLen, hits = g.last.hits, yaw = g.last.yaw, pitch = g.last.pitch }
        end
      end
      while w.t < tm + 1.2 do frame(w, script) end
      return w, c, v, Tl
    end)
    FPS, DT = 60, 1 / 60
    if not ok then error(W) end
    return W, cam, vc, T, lens
  end
  local function stat(lens)
    local top, drop, speed, atDrop = 0, 0, 0, nil
    for _, r in ipairs(lens) do if r.arm and r.t >= 0 then top = math.max(top, r.arm) end end
    for i, r in ipairs(lens) do
      if r.arm and r.t >= 0 then
        drop = math.max(drop, top - r.arm)
        if i > 1 and lens[i - 1].arm and r.t > lens[i - 1].t then speed = math.max(speed, (lens[i - 1].arm - r.arm) / (r.t - lens[i - 1].t)) end
      end
    end
    return drop, speed
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local W, cam, vc, T, lens = run(fps, { 0.45, 0.70 })
    local drop, speed = stat(lens)
    local hitsSeen = 0
    for _, r in ipairs(lens) do if (r.hits or 0) > 0 then hitsSeen = hitsSeen + 1 end end
    local Wm, _, _, _, lensM = run(fps, { 0.45, 0.70 }, { vehHoldColDelay = 0 })
    local dropM, speedM = stat(lensM)
    check(string.format("door pop in the mount hold, %d fps: the boom does not move for a 0.25 s door pulse (drop %.2f m, %d frames with hits); mutation (vehHoldColDelay = 0): drop %.2f m "
          .. "at %.1f m/s", fps, drop, hitsSeen, dropM, speedM), hitsSeen > 2 and drop <= 0.05 and dropM >= 0.5, string.format("%.2f / %.2f, hits %d", drop, dropM, hitsSeen))
    -- the camera at rest: its own yaw / pitch rate stays far below the quiet threshold (vehHoldColQuietRot), so the gate does not see the door as a camera turn
    local rate = 0
    for i = 2, #lens do
      local a, b = lens[i - 1], lens[i]
      if a.yaw and b.yaw and b.t > a.t then rate = math.max(rate, math.abs(CM.wrapDeg(b.yaw - a.yaw)) / (b.t - a.t), math.abs(b.pitch - a.pitch) / (b.t - a.t)) end
    end
    check(string.format("door pop in the mount hold, %d fps: the camera's turn rate in the door run is %.2f deg/s, below the quiet threshold %.1f deg/s", fps, rate, cam.cfg.vehHoldColQuietRot),
          rate < 0.25 * cam.cfg.vehHoldColQuietRot, tostring(rate))
  end
  -- a real occluder (the surface stays) is still honoured: the boom comes in after the delay, never faster than the rate limit (the rate path ends with the
  -- pull-out override of the mount, 0.77 s after the mount start in this harness (0.9 s in the game log); the solver's boom is hidden behind the override from then on)
  for _, fps in ipairs({ 30, 60, 144 }) do
    local W, cam, vc, T, lens = run(fps, { 0.10, 5 })
    local first, drop, speed = nil, 0, 0
    for i, r in ipairs(lens) do
      if r.arm and r.t >= 0 and r.t <= 0.75 then
        if first == nil and r.arm < lens[1].arm - 0.02 then first = r.t end
        drop = math.max(drop, lens[1].arm - r.arm)
        if i > 1 and lens[i - 1].arm and r.t > lens[i - 1].t then speed = math.max(speed, (lens[i - 1].arm - r.arm) / (r.t - lens[i - 1].t)) end
      end
    end
    check(string.format("door pop in the mount hold, %d fps: a surface that stays pulls the boom in %.2f s after it appeared (>= the delay), %.2f m by 0.75 s at <= 12 m/s (%.1f)", fps,
          (first or 99) - 0.10, drop, speed), first ~= nil and first - 0.10 >= 0.35 - 2 / fps and drop >= 0.3 and speed <= 12.01, string.format("%s %.2f %.1f", tostring(first), drop, speed))
  end
  -- a hit that comes with a moving pivot (V sliding into the seat next to a big body, the 1.15 m Columbus / Emperor case of 2026-11-01) is an occluder: the
  -- rate limited pull-in starts at once, exactly as without the delay
  for _, fps in ipairs({ 30, 60, 144 }) do
    local o = { 0.30, 5 }
    local _, _, _, _, lens = run(fps, o, nil, { mountDur = 0.9 })
    local _, _, _, _, lensM = run(fps, o, { vehHoldColDelay = 0 }, { mountDur = 0.9 })
    local function firstIn(l)
      for _, r in ipairs(l) do if r.arm and r.t >= 0.30 and l[1].arm and r.arm < l[1].arm - 0.02 then return r.t - 0.30 end end
      return nil
    end
    local a, b = firstIn(lens), firstIn(lensM)
    local same = #lens == #lensM
    for i = 1, same and #lens or 0 do if lens[i].arm ~= lensM[i].arm then same = false; break end end
    check(string.format("door pop in the mount hold, %d fps: a hit with the pivot moving pulls in after %.2f s (without the delay %.2f s), the same trajectory", fps, a or -1, b or -1),
          a ~= nil and b ~= nil and a <= 0.12 and same, string.format("%s %s same=%s", tostring(a), tostring(b), tostring(same)))
  end
-- review r4 (medium): the pivot is at rest (V in a standing car) but the camera turns (mouse-look swings the boom into the car body): that is an occluder, the
-- rate limited pull-in starts at once exactly as without the delay; the same surface with the camera at rest (the door) still waits the delay
do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local o = { 0.10, 5 } -- the surface appears 0.10 s after the mount start and stays (as in the 'surface that stays' check above)
    local rot = { 0.0, 5, 40 } -- 40 deg/s from the mount start, longer than the hold
    local _, camA, _, _, lens = run(fps, o, nil, nil, rot)
    local _, _, _, _, lensM = run(fps, o, { vehHoldColDelay = 0 }, nil, rot)
    local _, _, _, _, lensR = run(fps, o, { vehHoldColQuietRot = 0 }, nil, rot) -- mutation: the camera turn is not measured (the first version)
    local function firstIn(l)
      for _, r in ipairs(l) do if r.arm and r.t >= 0.10 and l[1].arm and r.arm < l[1].arm - 0.02 then return r.t - 0.10 end end
      return nil
    end
    local a, b, m = firstIn(lens), firstIn(lensM), firstIn(lensR)
    local same = #lens == #lensM
    for i = 1, same and #lens or 0 do if lens[i].arm ~= lensM[i].arm then same = false; break end end
    local turn = 0
    for i = 2, #lens do
      if lens[i].t >= 0.2 and lens[i].yaw and lens[i - 1].yaw and lens[i].t > lens[i - 1].t then
        turn = math.max(turn, math.abs(CM.wrapDeg(lens[i].yaw - lens[i - 1].yaw)) / (lens[i].t - lens[i - 1].t))
      end
    end
    check(string.format("rotation with an occluder in the mount hold, %d fps (camera turn %.0f deg/s): the pull-in starts after %.2f s (without the delay %.2f s), the same trajectory; "
          .. "mutation (turn not measured): %.2f s", fps, turn, a or -1, b or -1, m or -1),
          turn > 20 and a ~= nil and b ~= nil and a <= 0.12 and same and m ~= nil and m >= 0.35 - 2 / fps,
          string.format("%s %s same=%s mut=%s turn=%.1f", tostring(a), tostring(b), tostring(same), tostring(m), turn))
  end
end
end)

-- ================================================================ 2026-11-04 jump-out camera: release pitch, launch swing, start velocity, frame stall
-- Game logs 06-04-45 (jump_stand / jump_move) and the normal exit of normal_cycle_door: (1) the release to the orbit took the shown pitch of the launch swing (-4.6 against
-- -14.3 before the exit) as the orbit pitch for the rest of the game; the swing itself was 14 deg because the framing band tilted the camera by the whole lag of the
-- rising V; (2) a frame of 83 - 120 ms moved the release / pull-out blends by 10 - 15 % at once (catch-up steps of 0.535 m / 0.602 m); (3) the release started from the
-- shown pose at rest (the camera stopped dead at 4.6 m/s in one frame). Metrics are taken from the rendered camera of the offline replay (V rises at 9.8 m/s, g 16).
runBlock(function()
  local function pitchOfR(r) return math.deg(math.asin(math.max(-1, math.min(1, CM.norm(r.fwd).z)))) end
  -- one jump-out: speed 0 = the standing vehicle (native launch), 7 = moving; cc = utpp_camera config override, vv = utpp_vehiclecam config override
  local function jo(speed, cc, vv, stall)
    local ex = { speed = speed, launchAfter = speed == 0 and 0.06 or nil, before = function(w, c, v)
      w.env.Game.UTPPVehicleJumpOutEpoch = function() return w.joEpoch or 0 end
      for k, val in pairs(vv or {}) do v.ctrl.cfg[k] = val end
      w.stall = stall
    end }
    local W, cam, vc, T = jumpOut(true, 1.0, 9.8, 16, cc, ex)
    local byT = {}
    for _, k in ipairs(W.track) do byT[string.format("%.4f", k.t)] = k.pos end
    return W, cam, vc, T, byT
  end
  local function pitchStats(W, T, byT)
    local offAt = firstAt(W, "vehicle hold OFF (release to orbit done)")
    local p0 = pitchOfR(recAt(W, T.tu - 0.05))
    local up, dn = -99, 99
    for _, r in ipairs(W.rec) do
      -- (2026-11-05: without the landing framing tilt of the descent, r.jp (cfg.jumpOutFrame): this is the launch swing)
      if r.t >= T.tu and r.t <= offAt then up = math.max(up, pitchOfR(r) - (r.jp or 0) - p0); dn = math.min(dn, pitchOfR(r) - (r.jp or 0) - p0) end
    end
    local pAfter = pitchOfR(recAt(W, offAt + 0.3)) -- before the own pitch hands over to the game's look (0.5 s after the hold)
    return p0, up, dn, pAfter, offAt
  end
  local base = {}
  for _, sp in ipairs({ 0, 7 }) do
    local W, cam, _, T, byT = jo(sp)
    local p0, up, dn, pAfter = pitchStats(W, T, byT)
    local Wo, camo, _, To, byTo = jo(sp, { vehJumpReleasePitch = false, vehJumpFrame = false })
    local _, _, _, pAfterO = pitchStats(Wo, To, byTo)
    local Wf, _, _, Tf, byTf = jo(sp, { vehJumpFrame = false })
    local _, upF = pitchStats(Wf, Tf, byTf)
    print(string.format("  jump-out pitch (%s): before %.1f, swing +%.1f / %.1f, after the hold %.1f (mutation release pitch: %.1f, mutation band: swing +%.1f)",
      sp == 0 and "standing" or "moving", p0, up, dn, pAfter, pAfterO, upF))
    check(string.format("jump-out (%s): the orbit pitch after the hold is the pitch before the exit (%.1f -> %.1f, within 2 deg)", sp == 0 and "standing" or "moving", p0, pAfter),
          math.abs(pAfter - p0) <= 2 and count(W, "FAILED") == 0)
    check(string.format("jump-out (%s) mutation: the shown launch pitch as the release target (plain band) keeps the orbit further off than the fix (%.1f -> %.1f)", sp == 0 and "standing" or "moving", p0, pAfterO),
          math.abs(pAfterO - p0) > math.abs(pAfter - p0) + 0.5)
    check(string.format("jump-out (%s): the launch pitch swing stays within +5 / -5 deg of the pitch before the exit (+%.1f / %.1f)", sp == 0 and "standing" or "moving", up, dn),
          up <= 5 and dn >= -5)
    check(string.format("jump-out (%s) mutation: the plain framing band swings more than 5 deg (+%.1f)", sp == 0 and "standing" or "moving", upF), upF > 5)
  end

  -- (3) the release starts with the camera's speed: the rendered speed changes by at most 1.5 m/s from one frame to the next through the release start
  local function vzStep(W, T, byT)
    local relAt = firstAt(W, "vehicle release to orbit over")
    local m, at = 0, nil
    for i = 3, #W.rec do
      local a, b, c = W.rec[i - 2], W.rec[i - 1], W.rec[i]
      if c.t >= relAt - 0.03 and c.t <= relAt + 0.2 then
        local d = CM.len(CM.sub(CM.sub(c.pos, b.pos), CM.sub(b.pos, a.pos))) / DT
        if d > m then m, at = d, c.t - relAt end
      end
    end
    return m, at
  end
  for _, sp in ipairs({ 0, 7 }) do
    local W, _, _, T, byT = jo(sp)
    local Wm, _, _, Tm, byTm = jo(sp, { vehReleaseCarryVel = false })
    local d, dAt = vzStep(W, T, byT)
    local dm = vzStep(Wm, Tm, byTm)
    print(string.format("  jump-out release start (%s): largest frame-to-frame speed change %.2f m/s at %+.3f s (mutation: %.2f m/s)", sp == 0 and "standing" or "moving", d, dAt or 0, dm))
    check(string.format("jump-out (%s): the release starts with the camera's speed (frame-to-frame speed change %.2f <= 1.2 m/s)", sp == 0 and "standing" or "moving", d), d <= 1.2)
    check(string.format("jump-out (%s) mutation: the release from rest stops the camera dead (%.2f m/s in one frame > 3)", sp == 0 and "standing" or "moving", dm), dm > 3)
  end

  -- (2) one frame of 120 ms in the middle of a running release to the orbit (the shown pose and the target differ by a pitch of about 35 deg and the position that
  -- goes with it): the camera does not catch up with the blend in one frame (time beyond vehBlendDtCap of a frame is not counted)
  for _, carry in ipairs({ false, true }) do
    local function stallRun(cfgv)
      local W = makeWorld({})
      local cam = load(W)
      for k, val in pairs(cfgv or {}) do cam.cfg[k] = val end
      local function look(w) w.p.fppPitch = -60 end
      run(W, 7.0)
      withEnv(W, function() cam.HoldForVehicle(true, "test mount") end)
      run(W, W.t + 0.2, look)
      cam.S.look.pitch = -60
      run(W, W.t + 0.3, look)
      withEnv(W, function() cam.ReleaseToOrbit(0.8, "test exit", carry) end)
      local t0 = W.t
      W.stall = { at = t0 + 0.25, dt = 0.12 }
      run(W, W.t + 1.2, look)
      local m, mAt = 0, nil
      for i = 2, #W.rec do
        local a, b = W.rec[i - 1], W.rec[i]
        if a.t >= t0 + 0.1 and b.t <= t0 + 0.9 then
          local d = CM.dist(a.pos, b.pos)
          if d > m then m, mAt = d, b.t - t0 end
        end
      end
      return m, mAt, W
    end
    local m, mAt, W = stallRun(nil)
    local mm = stallRun({ vehBlendDtCap = 0 })
    print(string.format("  release to the orbit (carry %s) with a 120 ms frame: largest camera step %.3f m at +%.2f s (mutation without the cap %.3f m)", tostring(carry), m, mAt or 0, mm))
    check(string.format("release to the orbit (carry %s), a 120 ms frame at +0.25 s: no catch-up step (largest camera step %.3f m <= 0.25 m)", tostring(carry), m), m <= 0.25 and W.stall.done and count(W, "FAILED") == 0)
    check(string.format("release to the orbit (carry %s) mutation: without the stall cap the same frame steps %.3f m (> 0.25 m)", tostring(carry), mm), mm > 0.25)
  end

  -- (5) framing: V's waist (0.9 m over her feet, where a tucked or crouched V is) and head (1.6 m) stay inside the vertical field of view from the unmount until
  -- 0.5 s after the hold (the shown pitch of the old release kept the camera 10 deg too high: V sank out of the bottom of the frame)
  local function frameStats(W, T, byT)
    local offAt = firstAt(W, "vehicle hold OFF (release to orbit done)")
    local lo, hi, half = 99, -99, 27.5
    for _, r in ipairs(W.rec) do
      local pp = byT[string.format("%.4f", r.t)]
      if pp and r.t >= T.tu and r.t <= offAt + 0.5 then
        half = 0.5 * (r.fov or 55)
        local function ang(pt) local d, f = CM.norm(CM.sub(pt, r.pos)), CM.norm(r.fwd); return math.deg(math.asin(d.z)) - math.deg(math.asin(f.z)) end
        lo = math.min(lo, ang(CM.add(pp, CM.v3(0, 0, 0.9)))); hi = math.max(hi, ang(CM.add(pp, CM.v3(0, 0, 1.6))))
      end
    end
    return lo, hi, half
  end
  for _, sp in ipairs({ 0, 7 }) do
    local W, _, _, T, byT = jo(sp)
    local Wm, _, _, Tm, byTm = jo(sp, { vehJumpReleasePitch = false, vehJumpFrame = false })
    local lo, hi, half = frameStats(W, T, byT)
    local lom, him = frameStats(Wm, Tm, byTm)
    print(string.format("  jump-out framing (%s): waist %.1f deg, head %.1f deg from the view axis (half FOV %.1f); mutation waist %.1f, head %.1f", sp == 0 and "standing" or "moving", lo, hi, half, lom, him))
    check(string.format("jump-out (%s): V's waist and head stay inside the field of view through the flight, the landing and 0.5 s after (waist %.1f > -%.1f, head %.1f < %.1f)",
          sp == 0 and "standing" or "moving", lo, half, hi, half), lo > -half and hi < half)
  end

  -- a normal mount + ride + exit + release (standing vehicle, no jump-out; the pull-out of the mount included) is bit for bit the same with every new limiter / feature of this
  -- block off: the new paths act only in a jump-out hold (release pitch, band, start velocity) or after a frame longer than the cap (blends)
  do
    local recs = {}
    for _, off in ipairs({ false, true }) do
      recs[#recs + 1] = session({ before = function(_, c, v)
        if off then
          c.cfg.vehBlendDtCap, c.cfg.vehJumpReleasePitch, c.cfg.vehReleaseCarryVel, c.cfg.vehJumpFrame = 0, false, false, false
          v.ctrl.cfg.blendDtCap = 0
        end
      end })
    end
    local same = #recs[1].rec == #recs[2].rec and #recs[1].rec > 300
    for i = 1, same and #recs[1].rec or 0 do
      local a, b = recs[1].rec[i], recs[2].rec[i]
      if a.pos.x ~= b.pos.x or a.pos.y ~= b.pos.y or a.pos.z ~= b.pos.z or a.fwd.x ~= b.fwd.x or a.fwd.y ~= b.fwd.y or a.fwd.z ~= b.fwd.z then same = false; break end
    end
    check(string.format("normal mount / ride / exit (%d frames): the rendered camera is bit for bit the same with the jump-out release / band / start velocity / stall caps off", #recs[1].rec), same)
  end

  -- (2b) the mount pull-out: one frame of 120 ms in the middle of the pull-out (game log normal_cycle_door: 0.602 m / 11.6 deg after a 0.12 s stall)
  do
    local W0 = session({})
    local pullAt = firstAt(W0, "pull-out start (")
    local function stallPull(capOff)
      local at = pullAt + 0.35
      local W = session({ before = function(w, c, v)
        w.stall = { at = at, dt = 0.12 }
        if capOff then v.ctrl.cfg.blendDtCap = 0 end
      end })
      local m, mAt, ma = 0, nil, 0
      for i = 2, #W.rec do
        local a, b = W.rec[i - 1], W.rec[i]
        if b.t >= at - 0.02 and b.t <= at + 0.3 then
          local d = CM.dist(a.pos, b.pos)
          if d > m then m, mAt = d, b.t - at end
          ma = math.max(ma, math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd)))))))
        end
      end
      return m, mAt, ma, W
    end
    local m, mAt, ma, W = stallPull(false)
    local mm, _, mma = stallPull(true)
    print(string.format("  mount pull-out with a 120 ms frame: largest camera step %.3f m / %.1f deg at +%.2f s (mutation without the cap %.3f m / %.1f deg)", m, ma, mAt or 0, mm, mma))
    check(string.format("mount pull-out, a 120 ms frame in the middle: no catch-up step (%.3f m, %.1f deg)", m, ma), pullAt ~= nil and W.stall.done and m <= 0.3 and ma <= 6 and count(W, "FAILED") == 0)
    check(string.format("mount pull-out mutation: without the stall cap the same frame steps %.3f m / %.1f deg", mm, mma), mm > m * 1.5 + 0.1)
  end

  -- the same at a steady 20 fps (every frame is longer than the base cap of 0.04 s; the cap follows the running frame time): the stall cap is not a slow-motion switch
  do
    local recs = {}
    for _, off in ipairs({ false, true }) do
      FPS, DT = 20, 1 / 20
      local ok, W = pcall(session, { before = function(_, c, v)
        if off then c.cfg.vehBlendDtCap = 0; v.ctrl.cfg.blendDtCap = 0 end
      end })
      FPS, DT = 60, 1 / 60
      if not ok then error(W) end
      recs[#recs + 1] = W
    end
    local same = #recs[1].rec == #recs[2].rec and #recs[1].rec > 100
    local firstDiff
    for i = 1, same and #recs[1].rec or 0 do
      local a, b = recs[1].rec[i], recs[2].rec[i]
      if a.pos.x ~= b.pos.x or a.pos.y ~= b.pos.y or a.pos.z ~= b.pos.z or a.fwd.x ~= b.fwd.x or a.fwd.y ~= b.fwd.y or a.fwd.z ~= b.fwd.z then same = false; firstDiff = a.t; break end
    end
    check(string.format("normal mount / ride / exit at a steady 20 fps (%d frames): the rendered camera is bit for bit the same with the stall caps on and off", #recs[1].rec), same, tostring(firstDiff))
  end
end)

-- ================================================================ 2026-11-04 r9: 'behind' mount, the camera at V's head (game log 09-09-29 .. 09-09-31, build r5a)
-- The standing car's body between V (next to the left rear door) and the held orbit (the camera on the far side of the cab) pulled the boom in to 0.88 m from V's
-- head within 0.2 s (camTrace dist 2.93 -> 1.68 -> 0.89, hits=1, mode hold, the pivot still decelerating from the walk = not quiet = the rate limited pull-in at once,
-- 12 m/s) and held it there for 0.9 s (V's face filled the frame); the pull-out started from that pose ("start frozen at -172 deg, 0.83 m from the vehicle") and its
-- radius grew along the angle of V: it passed 0.23 m from V's position (hair filled the frame). The mount-pivot line said "largest step 0.04 m ... pivot unchanged".
runBlock(function()
  -- vehicle frame of the log: V at (-1.70,-0.41,-0.41), the left rear door; shown camera 'cam veh=' at 20 Hz (t = log t=918.xx), mount start 918.51
  local V0 = { x = -1.70, y = -0.41, z = -0.41 }
  local PIV = { x = -1.70, y = -0.41, z = 1.14 } -- pivotZ +1.55 over her feet
  local LOG = { { 918.57, 1.04, 0.68, 1.49 }, { 918.62, 1.05, 0.63, 1.49 }, { 918.68, 1.06, 0.59, 1.49 }, { 918.73, 0.85, 0.48, 1.46 }, { 918.78, 0.27, 0.26, 1.38 },
                { 918.83, -0.31, 0.06, 1.31 }, { 918.88, -0.86, -0.13, 1.24 }, { 918.93, -0.87, -0.13, 1.24 }, { 919.00, -0.87, -0.13, 1.24 },
                { 919.06, -0.87, -0.13, 1.24 }, { 919.11, -0.86, -0.13, 1.25 }, { 919.16, -0.85, -0.13, 1.25 }, { 919.21, -0.85, -0.12, 1.25 },
                { 919.26, -0.84, -0.12, 1.25 }, { 919.32, -0.84, -0.12, 1.25 }, { 919.37, -0.83, -0.12, 1.25 } }
  local W0 = makeWorld({})
  local cam, vc = load(W0)
  -- (1) the detector on the logged camera: the pivot-only line of the game ("largest step 0.04 m", pivot unchanged) and the new camera line
  local function replay(withCam)
    local st, lines, camLines = {}, {}, {}
    local n = 0
    local function step(t, camp, mounted)
      n = n + 1
      local c = nil
      if camp and withCam then
        local d = math.sqrt((camp[2] - PIV.x) ^ 2 + (camp[3] - PIV.y) ^ 2 + (camp[4] - PIV.z) ^ 2)
        c = { x = camp[2], y = camp[3], z = camp[4], arm = d, fullLen = 2.95 }
      end
      local _, _, line, camLine = cam.MountShiftGate(st, cam.cfg, t, n, "tpp", mounted, { held = true }, V0, c)
      if line then lines[#lines + 1] = line end
      if camLine then camLines[#camLines + 1] = camLine end
    end
    for t = 918.0, 918.5, 1 / 60 do step(t, nil, false) end -- on foot
    -- the game runs the gate every frame (about 18 ms): the 20 Hz lines of the log are interpolated to 60 fps (a per-line step would read 3 x the per-frame one)
    local i = 1
    for t = 918.52, 919.43, 1 / 60 do
      while LOG[i + 1] and LOG[i + 1][1] <= t do i = i + 1 end
      local a, b = LOG[i], LOG[i + 1]
      local w = b and math.max(0, math.min(1, (t - a[1]) / (b[1] - a[1]))) or 0
      local p = { t, a[2], a[3], a[4] }
      if b then p = { t, a[2] + (b[2] - a[2]) * w, a[3] + (b[3] - a[3]) * w, a[4] + (b[4] - a[4]) * w } end
      if t < LOG[1][1] then p = { t, LOG[1][2], LOG[1][3], LOG[1][4] } end
      step(t, p, true)
    end
    step(919.43, nil, true) -- the pull-out override takes the view (no orbit camera any more)
    for t = 919.5, 920.2, 1 / 60 do step(t, nil, true) end
    return lines, camLines
  end
  local lines, camLines = replay(true)
  local _, oldCamLines = replay(false)
  check("r9 detector: the pivot-only line of the game stays 'pivot unchanged' (the replay of the logged mount: pivot at rest)", #lines == 1 and string.find(lines[1], "pivot unchanged", 1, true) ~= nil, tostring(lines[1]))
  local nearest = camLines[1] and tonumber(string.match(camLines[1], "nearest to V ([%d%.]+) m"))
  local stepM = camLines[1] and tonumber(string.match(camLines[1], "camera move relative to V ([%d%.]+) m per"))
  local boom = camLines[1] and tonumber(string.match(camLines[1], "shortest boom ([%d%.]+) m"))
  check(string.format("r9 detector: the camera line of the logged mount is a WARNING (nearest to V %s m, move %s m per 0.1 s (12 m/s), shortest boom %s m)", tostring(nearest), tostring(stepM), tostring(boom)),
        #camLines == 1 and string.find(camLines[1], "mount camera WARNING", 1, true) ~= nil and nearest ~= nil and nearest < 1.0 and nearest > 0.8 and stepM ~= nil and stepM > 0.9
        and boom ~= nil and boom < 1.0, tostring(camLines[1]))
  check("r9 detector mutation: without the camera the game's line sees nothing (no camera line)", #oldCamLines == 0)
  -- a camera that stays far from V and moves smoothly is no warning
  do
    local st, n, out = {}, 0, nil
    local function step(t, mounted, c)
      n = n + 1
      local _, _, _, cl = cam.MountShiftGate(st, cam.cfg, t, n, "tpp", mounted, { held = true }, V0, c)
      out = out or cl
    end
    for t = 0, 0.5, 1 / 60 do step(t, false, nil) end
    for t = 0.51, 2.4, 1 / 60 do step(t, true, { x = 1.0 + 0.3 * (t - 0.5), y = 0.7, z = 1.5, arm = 2.93, fullLen = 2.95 }) end
    check("r9 detector: a far, slowly moving mount camera gives a plain camera line without WARNING", out ~= nil and string.find(out, "mount camera:", 1, true) ~= nil, tostring(out))
  end

  -- 2026-11-04 r11: the camera of the previous solve was measured against THIS frame's V, so V's own snap onto the seat (the game's mount puts V on the seat slot
  -- in one frame) or a stale solve counted as camera motion and printed a false WARNING. The sample is the previous solve's camera against V's position of that
  -- solve (cam.px/py/pz, cam.t), only when that solve is the previous update (cam.skip otherwise: no sample, the window goes on).
  do
    local function watch(pairing, skipAt)
      local st, n, out = {}, 0, nil
      local vx, vy, vz, prev = 0, 0, 0, nil
      local function step(t, mounted)
        n = n + 1
        if t >= 1.0 then vx = 1.0 end -- V snaps 1 m onto the seat at t = 1.0 (one frame); the camera follows V rigidly at a constant offset (2.5, 0.7, 1.5)
        local c = nil
        if prev and mounted then
          c = { x = prev.x + 2.5, y = prev.y + 0.7, z = prev.z + 1.5, arm = 2.93, fullLen = 2.95 }
          if pairing then c.px, c.py, c.pz, c.t = prev.x, prev.y, prev.z, prev.t end
          if skipAt and t >= skipAt and t < skipAt + 0.05 then c = { x = c.x, y = c.y, z = c.z, skip = true } end
        end
        local _, _, _, cl = cam.MountShiftGate(st, cam.cfg, t, n, "tpp", mounted, { held = true }, { x = vx, y = vy, z = vz }, c)
        out = out or cl
        prev = { x = vx, y = vy, z = vz, t = t }
      end
      for t = 0, 0.5, 1 / 60 do step(t, false) end
      for t = 0.51, 2.4, 1 / 60 do step(t, true) end
      return out
    end
    local good, bad, skipped = watch(true), watch(false), watch(true, 1.4)
    check("r11 mount camera watch: V snaps 1 m onto the seat while the camera follows rigidly: no WARNING (previous camera against V of that solve)",
          good ~= nil and string.find(good, "mount camera:", 1, true) ~= nil and string.find(good, "WARNING", 1, true) == nil, tostring(good))
    check("r11 mount camera watch mutation (this frame's V against the previous camera = the old pairing): the seat snap is a WARNING", bad ~= nil and string.find(bad, "mount camera WARNING", 1, true) ~= nil,
          tostring(bad))
    check("r11 mount camera watch: a skipped sample (the previous solve is not the previous update) does not end the window early and gives no WARNING",
          skipped ~= nil and string.find(skipped, "mount camera:", 1, true) ~= nil and string.find(skipped, "WARNING", 1, true) == nil and string.find(skipped, "orbit over 1.50 s", 1, true) ~= nil, tostring(skipped))
  end
  -- (2) the boom floor of the mount hold (CM.collisionSmooth): the logged pull-in against a surface 0.95 m from the pivot, the rate limited path (12 m/s, not quiet)
  for _, fps in ipairs({ 30, 60, 144 }) do
    local function boomLow(minLen)
      local cs = CM.newCollision()
      local lo, dt = 99, 1 / fps
      CM.collisionSmooth(cs, cam.cfg, dt, 2.93, 0, 12, nil, minLen)
      for i = 1, math.floor(1.0 * fps) do
        local t = i * dt
        local allowed = t >= 0.2 and 0.89 or 2.93
        CM.collisionSmooth(cs, cam.cfg, dt, allowed, allowed < 2 and 1 or 0, 12, nil, minLen)
        lo = math.min(lo, cs.len)
      end
      return lo
    end
    local with, without = boomLow(cam.cfg.vehHoldColMin), boomLow(nil)
    check(string.format("r9 boom floor, %d fps: the rate limited pull-in of the mount hold stops at %.2f m (floor %.2f m); mutation (no floor): %.2f m", fps, with, cam.cfg.vehHoldColMin, without),
          cam.cfg.vehHoldColMin >= 1.2 and with >= cam.cfg.vehHoldColMin - 1e-6 and with <= cam.cfg.vehHoldColMin + 0.2 and without < 1.0)
  end
  -- a boom that was already shorter than the floor (V next to a wall before the mount) is not pulled in further and not lengthened by the floor
  do
    local cs = CM.newCollision()
    CM.collisionSmooth(cs, cam.cfg, 1 / 60, 1.1, 0, 12, nil, 1.6)
    for _ = 1, 30 do CM.collisionSmooth(cs, cam.cfg, 1 / 60, 0.6, 1, 12, nil, 1.6) end
    check(string.format("r9 boom floor: a boom that was already shorter (1.1 m) keeps its length (%.2f m)", cs.len), math.abs(cs.len - 1.1) < 1e-9)
  end
  -- 2026-11-04 r11: the gate of the rate limited pull-in drops while the orbit is still shown and the occluder remains (an aborted / cancelled mount,
  -- HoldForVehicle(false)): the boom at the 1.6 m floor used to take the instant branch and snapped up to ~0.7 m in one frame. It now keeps the rate limited
  -- pull-in until it has reached the allowed length, then the instant pull-in is back (vehHoldColCarry = false: the previous behaviour).
  for _, fps in ipairs({ 30, 60, 144 }) do
    local function handOver(c)
      local cs, dt = CM.newCollision(), 1 / fps
      CM.collisionSmooth(cs, c, dt, 2.93, 0, 12, nil, 1.6)
      for _ = 1, math.floor(0.5 * fps) do CM.collisionSmooth(cs, c, dt, 0.7, 1, 12, nil, 1.6) end -- the mount hold: the boom stops at the floor
      local atFloor, maxStep, prev, reachedAt, stepsAfter = cs.len, 0, cs.len, nil, nil
      for i = 1, math.floor(2.0 * fps) do -- the gate is gone (inRate / inMin nil), the car body is still there
        CM.collisionSmooth(cs, c, dt, 0.7, 1, nil, nil, nil)
        maxStep = math.max(maxStep, prev - cs.len)
        prev = cs.len
        if reachedAt == nil and cs.len <= 0.7 + 1e-9 then reachedAt = i * dt end
      end
      local reached = cs.len
      CM.collisionSmooth(cs, c, dt, 0.4, 1, nil, nil, nil) -- a new, closer hit afterwards: instant again
      return { floor = atFloor, maxStep = maxStep, reached = reached, reachedAt = reachedAt, instant = cs.len }
    end
    local a = handOver(cam.cfg)
    local b = handOver(setmetatable({ vehHoldColCarry = false }, { __index = cam.cfg }))
    local rate = cam.cfg.vehHoldColRate or 12
    check(string.format("r11 boom hand-over, %d fps: the gate drops with the boom at the %.2f m floor: every step <= %.2f m (rate %.0f m/s; %.3f m), the boom reaches the allowed 0.70 m after %.2f s, "
          .. "a new closer hit is instant again (%.2f m)", fps, a.floor, rate / fps, rate, a.maxStep, a.reachedAt or -1, a.instant),
          math.abs(a.floor - 1.6) < 1e-6 and a.maxStep <= rate / fps + 1e-6 and math.abs(a.reached - 0.7) < 1e-9 and a.reachedAt ~= nil and a.reachedAt <= 0.6
          and math.abs(a.instant - 0.4) < 1e-9, string.format("%.3f %.3f %s %.3f", a.floor, a.maxStep, tostring(a.reachedAt), a.instant))
    check(string.format("r11 boom hand-over mutation (vehHoldColCarry = false: the previous behaviour), %d fps: the boom snaps %.2f m in one frame", fps, b.maxStep), b.maxStep > 0.8)
  end
  -- the floor is only on the rate limited path: without inRate (on foot) the pull-in is instant, as always
  do
    local cs = CM.newCollision()
    CM.collisionSmooth(cs, cam.cfg, 1 / 60, 2.93, 0, nil, nil, 1.6)
    CM.collisionSmooth(cs, cam.cfg, 1 / 60, 0.5, 1, nil, nil, 1.6)
    check(string.format("r9 boom floor: on foot (no rate limit) the pull-in is instant and not floored (%.2f m)", cs.len), math.abs(cs.len - 0.5) < 1e-9)
  end

  -- (3) the whole camera: the mount hold of the harness with a surface 1.0 m from every ray start that appears 0.10 s after the mount start and stays (V moving into the seat; the rate limited boom only counts while the orbit is shown, before the pull-out override)
  for _, fps in ipairs({ 30, 60, 144 }) do
    local function mount(cc, vcc)
      local tm = math.max(7, 330 / fps + 0.5)
      FPS, DT = fps, 1 / fps
      local lens = {}
      local ok, W = pcall(function()
        local w = makeWorld({})
        local c, v = load(w)
        for k, val in pairs(cc or {}) do c.cfg[k] = val end
        for k, val in pairs(vcc or {}) do v.ctrl.cfg[k] = val end
        local sqs = {}
        function sqs:SyncRaycastByCollisionGroup(from, to)
          local mt = w.t - tm
          if mt >= 0.10 then
            local dx, dy, dz = to.x - from.x, to.y - from.y, to.z - from.z
            local l = math.sqrt(dx * dx + dy * dy + dz * dz)
            if l > 1.0 then return true, { position = { x = from.x + dx / l * 1.0, y = from.y + dy / l * 1.0, z = from.z + dz / l * 1.0 } } end
          end
          return false, { position = { x = 0, y = 0, z = 0 } }
        end
        w.env.Game.GetSpatialQueriesSystem = function() return sqs end
        local Tl = timeline({ tm = tm, mountDur = 0.9 }) -- V moves into the seat: the pivot is not at rest = the rate limited pull-in at once
        local base = Tl.script
        local script = function(ww, t)
          base(ww, t)
          local g = c.G
          if g and g.last and t >= tm and t <= tm + 1.2 then lens[#lens + 1] = { t = t - tm, arm = g.last.arm, hits = g.last.hits, ovr = c.vh and c.vh.override ~= nil } end
        end
        while w.t < tm + 1.2 do frame(w, script) end
        return w
      end)
      FPS, DT = 60, 1 / 60
      if not ok then error(W) end
      return lens
    end
    -- (2026-11-04 r12: the mutation is the r11 pull-in too - at the capped rate the boom has not reached the surface within the orbit window)
    -- (2026-11-05: the early pull-out of a camera that approaches the vehicle box (pullEscapeLead) sees the collapsing boom of the mutation and starts the
    -- override one frame earlier at 30 fps (0.33 instead of 0.40 s after the mount, the product's own frame), before the boom is under 1.0 m: the mutation
    -- runs with pullEscapeLead = 0 so its orbit window is the product's and only the boom floor differs)
    local lensF, lensM = mount(nil), mount({ vehHoldColMin = 0, vehHoldColMountRate = 0, vehHoldColLift = 0 }, { pullEscapeLead = 0 })
    local function lo(l, t1)
      local m, hits, ovrAt = 99, 0, nil
      for _, r in ipairs(l) do
        if r.arm and r.t <= t1 and not r.ovr then m = math.min(m, r.arm) end
        if (r.hits or 0) > 0 and not r.ovr then hits = hits + 1 end
        if r.ovr and ovrAt == nil then ovrAt = r.t end
      end
      return m, hits, ovrAt
    end
    local a, hitsA, ovrA = lo(lensF, 0.75)
    local b, _, ovrB = lo(lensM, 0.75)
    check(string.format("r9 mount hold, %d fps: the boom against a standing surface stays >= %.2f m (%.2f m, %d frames with hits); mutation (vehHoldColMin = 0): %.2f m "
          .. "in the same orbit window (override from %.3f / %.3f s)", fps, cam.cfg.vehHoldColMin, a, hitsA, b, ovrA or -1, ovrB or -1),
          hitsA > 3 and a >= cam.cfg.vehHoldColMin - 1e-6 and b < 1.0 and ovrA ~= nil and ovrB ~= nil and math.abs(ovrA - ovrB) < 1e-6,
          string.format("%.2f %.2f %d", a, b, hitsA))
  end

  -- (4) the pull-out path of the log: start frozen at -172 deg, 0.83 m from the vehicle centre, 1.25 m up; V's position in the same frame. The path must keep
  -- pullClearR from V's axis; mutation pullClearR = 0: it passes within 0.5 m (the log: 0.23 m)
  do
    local f = { px = V0.x, py = V0.y, pz = V0.z, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 } -- vehicle at the origin; pose results are player-local = relative to V
    local rad = math.pi / 180
    local starts = {
      { "logged start (-172 deg, 0.83 m, 1.25 m up)", { a = -172 * rad, r = 0.83, h = 1.25 }, true },
      { "start at the boom floor (1.6 m from the pivot, over the cab)", { a = math.atan2(0.19, -0.27), r = math.sqrt(0.27 ^ 2 + 0.19 ^ 2), h = 1.54 } },
    }
    for _, st in ipairs(starts) do
      local function minD(clr)
        local cfg = setmetatable({ pullClearR = clr }, { __index = vc.DEFAULTS })
        local m, at = 99, 0
        for i = 0, 200 do
          local w = i / 200
          local p = vc.PullPathPose(st[2], w, f, cfg)
          if p.z > -0.3 and p.z < 2.1 then
            local d = math.sqrt(p.x * p.x + p.y * p.y)
            if d < m then m, at = d, w end
          end
        end
        return m, at
      end
      local withD, withAt = minD(vc.DEFAULTS.pullClearR)
      local offD = minD(0)
      -- 2026-11-04 r12: a start inside the clearance (the logged one is 0.93 m from V, pullClearR is 1.0 m) keeps its own distance at w = 0 and fades out to the clearance
      local p0 = vc.PullPathPose(st[2], 0, f, setmetatable({ pullClearR = 0 }, { __index = vc.DEFAULTS }))
      local startD = math.sqrt(p0.x * p0.x + p0.y * p0.y)
      check(string.format("r9 pull-out path, %s: closest to V's axis %.2f m at w=%.2f (clearance %.2f m); without the clearance: %.2f m", st[1], withD, withAt, vc.DEFAULTS.pullClearR, offD),
            withD >= math.min(vc.DEFAULTS.pullClearR, startD) - 1e-6 and (st[3] ~= true or offD < 0.5)) -- (the logged start is the mutation: without the clearance it passes within 0.5 m)
    end
    -- a path that never comes near V (the parallel rides: start 3 m away on the far side) is unchanged by the clearance, bit for bit
    local far = { a = 0.3, r = 3.2, h = 1.9 }
    local same = true
    for i = 0, 100 do
      local w = i / 100
      local c1 = setmetatable({ pullClearR = vc.DEFAULTS.pullClearR }, { __index = vc.DEFAULTS })
      local c0 = setmetatable({ pullClearR = 0 }, { __index = vc.DEFAULTS })
      local p1, y1, q1 = vc.PullPathPose(far, w, f, c1)
      local p0, y0, q0 = vc.PullPathPose(far, w, f, c0)
      if p1.x ~= p0.x or p1.y ~= p0.y or p1.z ~= p0.z or y1 ~= y0 or q1 ~= q0 then same = false end
    end
    check("r9 pull-out path: a start on the far side of the car (3.2 m) is bit for bit the same with the clearance on and off", same)
    -- a start that is already inside the clearance (a boom that was shorter before the hold, 0.25 m from V): w = 0 is the frozen pose bit for bit, no step, clear after the fade
    do
      local near = { a = math.atan2(-0.45, -1.45), r = math.sqrt(1.45 ^ 2 + 0.45 ^ 2), h = 1.3 } -- (a raw path through V's axis itself: r11 keeps one side for the pull, see the sweep below; this one passes 0.21 m)
      local c1 = setmetatable({ pullClearR = vc.DEFAULTS.pullClearR }, { __index = vc.DEFAULTS })
      local c0 = setmetatable({ pullClearR = 0 }, { __index = vc.DEFAULTS })
      local p10, p00 = vc.PullPathPose(near, 0, f, c1), vc.PullPathPose(near, 0, f, c0)
      local at0 = p10.x == p00.x and p10.y == p00.y and p10.z == p00.z
      local maxStep, maxStep0, clearAfter = 0, 0, 99
      local prev, prev0 = p10, p00
      for i = 1, 400 do
        local w = i / 400
        local a, b = vc.PullPathPose(near, w, f, c1), vc.PullPathPose(near, w, f, c0)
        maxStep = math.max(maxStep, math.sqrt((a.x - prev.x) ^ 2 + (a.y - prev.y) ^ 2 + (a.z - prev.z) ^ 2))
        maxStep0 = math.max(maxStep0, math.sqrt((b.x - prev0.x) ^ 2 + (b.y - prev0.y) ^ 2 + (b.z - prev0.z) ^ 2))
        if w >= vc.DEFAULTS.pullClearFade and a.z > -0.3 and a.z < 2.1 then clearAfter = math.min(clearAfter, math.sqrt(a.x * a.x + a.y * a.y)) end
        prev, prev0 = a, b
      end
      check(string.format("r9 pull-out path: a start inside the clearance (%.2f m from V): w = 0 is the frozen pose, the largest step per 1/400 of the path %.3f m (without the clearance %.3f m), clear (%.2f m) after the fade",
            math.sqrt(p00.x ^ 2 + p00.y ^ 2), maxStep, maxStep0, clearAfter), at0 and maxStep <= maxStep0 * 1.5 + 0.01 and clearAfter >= vc.DEFAULTS.pullClearR - 1e-6)
    end
    -- motorcycles keep their path (bikePullClearR = 0): the path config of a bike has no clearance
    check("r9 pull-out path: a motorcycle's path config has no clearance (bikePullClearR 0), a car's has", (vc.PullCfg(vc.DEFAULTS, true).pullClearR or -1) == 0 and (vc.PullCfg(vc.DEFAULTS, false).pullClearR or 0) > 0)

    -- 2026-11-04 r11: the pushed path is continuous for every start. The r9 push was radial from V's axis, so a raw path that passes near the axis swung the pushed
    -- point around V at the clearance radius, and on an exact pass it flipped side within one path step (ported replay: start on the 1.6 m boom floor, orbit
    -- yaw ~17.5 deg, pitch -10: a 1.57 m camera step over 1/400 of the path against 0.02 m raw). The height band also popped sideways 0.45 - 0.62 m where a path
    -- crossed z = 2.1 m within the clearance radius. Sweep: boom 1.2 / 1.6 m x pitch -20..+10 x the full yaw circle (5 deg) plus a fine yaw scan (0.25 deg) around
    -- the start whose raw path passes closest to V's axis; the largest 3D step per 1/400 of the path against the raw path's, and the clearance from V's axis inside the
    -- band. Mutations: pullClearSide = 0 + pullClearRamp = 0 (the r9 push and the hard band).
    do
      local deg = math.pi / 180
      local function cfgWith(over) return setmetatable(over, { __index = vc.DEFAULTS }) end
      local function startAt(boom, yawDeg, pitchDeg)
        local cy, sy, cp, sp = math.cos(yawDeg * deg), math.sin(yawDeg * deg), math.cos(pitchDeg * deg), math.sin(pitchDeg * deg)
        local fx, fy, fz = -sy * cp, cy * cp, sp -- look direction (yaw CCW from +y, pitch up positive); the boom is behind it, the pivot 1.55 m over V's feet
        return vc.VehCyl(-boom * fx, -boom * fy, 1.55 - boom * fz, f)
      end
      local function dist3(a, b) return math.sqrt((a.x - b.x) ^ 2 + (a.y - b.y) ^ 2 + (a.z - b.z) ^ 2) end
      local D = vc.DEFAULTS
      -- the height factor of the band as the path computes it: 1 inside [z0, z1], the ramp outside both edges
      local function zfOf(z, ramp)
        if z >= D.pullClearZ0 and z <= D.pullClearZ1 then return 1 end
        if ramp <= 0 then return 0 end
        local k = (z < D.pullClearZ0) and (z - (D.pullClearZ0 - ramp)) / ramp or 1 - (z - D.pullClearZ1) / ramp
        if k <= 0 then return 0 end
        if k >= 1 then return 1 end
        return k * k * (3 - 2 * k)
      end
      local cfg0 = cfgWith({ pullClearR = 0 })
      -- one start: largest step of the pushed path and of the raw path per 1/N of the path, the clearance margin (path distance from V's axis minus the faded radius,
      -- where the faded radius is > 0), whether the raw path enters the clearance, and its closest approach to V's axis inside the band
      local function probe(cfgv, st, N)
        local ramp = tonumber(cfgv.pullClearRamp) or 0.25
        local prev, prev0 = vc.PullPathPose(st, 0, f, cfgv), vc.PullPathPose(st, 0, f, cfg0)
        local mx, mx0, clr, touched, near = 0, 0, 99, false, 99
        for i = 1, N do
          local w = i / N
          local a, b = vc.PullPathPose(st, w, f, cfgv), vc.PullPathPose(st, w, f, cfg0)
          mx, mx0 = math.max(mx, dist3(a, prev)), math.max(mx0, dist3(b, prev0))
          local zf = zfOf(a.z, ramp)
          if zf > 0 then
            local db = math.sqrt(b.x * b.x + b.y * b.y)
            if db < D.pullClearR * zf - 1e-9 then touched = true end -- the raw path is inside the (faded) clearance here: the push is active
            if zf == 1 then near = math.min(near, db) end
            clr = math.min(clr, math.sqrt(a.x * a.x + a.y * a.y) - D.pullClearR * zf)
          end
          prev, prev0 = a, b
        end
        return mx, mx0, clr, touched, near
      end
      local function sweep(over)
        local cfgv = cfgWith(over or {})
        local R = { ex = -1, at = "", step = 0, clr = 99, runs = 0, crossing = 0 }
        local function note(boom, pitch, yaw, near)
          local st = startAt(boom, yaw, pitch)
          local mx, mx0, clr, touched, nr = probe(cfgv, st, 400)
          R.runs = R.runs + 1
          if touched then R.crossing = R.crossing + 1 end
          local ex = mx - (3 * mx0 + 0.03) -- excess over "a small multiple of the raw step"
          if ex > R.ex then R.ex, R.at, R.step = ex, string.format("boom %.1f pitch %d yaw %.2f (step %.3f m, raw %.3f m)", boom, pitch, yaw, mx, mx0), mx end
          R.clr = math.min(R.clr, clr)
          return nr
        end
        for _, boom in ipairs({ 1.2, 1.6 }) do
          for _, pitch in ipairs({ -20, -10, 0, 10 }) do
            local bestNear, bestYaw = 99, 0
            for yaw = 0, 355, 5 do
              local nr = note(boom, pitch, yaw)
              if nr < bestNear then bestNear, bestYaw = nr, yaw end
            end
            for k = -20, 20 do note(boom, pitch, bestYaw + k * 0.25) end -- the axis-crossing band, fine
          end
        end
        return R
      end
      local good = sweep(nil)
      local old = sweep({ pullClearSide = 0, pullClearRamp = 0 })
      check(string.format("r11 pull-out path: %d starts (boom 1.2 / 1.6 m, pitch -20..+10, the full yaw circle + a fine scan of the axis-crossing band; %d of them cross the clearance): the largest step per "
            .. "1/400 of the path stays within 3 x the raw step + 3 cm (worst: %s), the clearance holds (worst margin %.4f m)", good.runs, good.crossing, good.at, good.clr),
            good.ex <= 0 and good.clr >= -1e-6 and good.crossing >= 20, string.format("excess %.3f %s margin %.4f", good.ex, good.at, good.clr))
      check(string.format("r11 pull-out path mutation (pullClearSide = 0, pullClearRamp = 0: the r9 push): the same sweep steps (worst %s)", old.at), old.ex > 0.5, string.format("excess %.3f %s", old.ex, old.at))
      -- the height band: a path that crosses z = 2.1 m inside the clearance radius pops sideways with the hard band and glides with the ramp. The worst start of a yaw scan
      -- (boom 1.2 / 1.6, pitch -20..-10; those starts begin around z = 2 m) with the hard band and the one-sided push, then the finely sampled path of that start
      do
        local hard = cfgWith({ pullClearRamp = 0 })
        local worstPop, worstAt = 0, nil
        for _, boom in ipairs({ 1.2, 1.6 }) do
          for _, pitch in ipairs({ -20, -15, -10 }) do
            for yaw = 0, 359 do
              local mx, mx0 = probe(hard, startAt(boom, yaw, pitch), 400)
              if mx - mx0 > worstPop then worstPop, worstAt = mx - mx0, { boom = boom, pitch = pitch, yaw = yaw } end
            end
          end
        end
        local st = startAt(worstAt.boom, worstAt.yaw, worstAt.pitch)
        local crossed = false
        do
          local prev0
          for i = 0, 4000 do
            local b = vc.PullPathPose(st, i / 4000, f, cfg0)
            if prev0 and b.z >= D.pullClearZ1 and prev0.z < D.pullClearZ1 and math.sqrt(b.x ^ 2 + b.y ^ 2) < D.pullClearR then crossed = true end
            prev0 = b
          end
        end
        local mNew, m0 = probe(cfgWith({}), st, 4000)
        local mHard, m0h = probe(hard, st, 4000)
        check(string.format("r11 pull-out path, height band edge (boom %.1f, pitch %d, yaw %d; the raw path crosses z = %.1f m inside the clearance radius: %s): largest step per 1/4000 of the path %.4f m "
              .. "with the ramp (raw %.4f m), %.3f m with the hard band", worstAt.boom, worstAt.pitch, worstAt.yaw, D.pullClearZ1, tostring(crossed), mNew, m0, mHard),
              crossed and mNew <= 3 * m0 + 0.01 and mHard > 0.3, string.format("ramp %.4f raw %.4f hard %.4f", mNew, m0, mHard))
      end
    end
  end
end)

-- ================================================================ 2026-11-04 r12: the camera around a car mount (game logs 12-57-36 / 13-13-25, build r6a, camTrace on)
-- The 'mount camera' watch (M.MountShiftGate) printed WARNINGs on 4 of 14 car mounts: (W1/W2/W3) the boom came in 2.93 -> 1.60 m within 0.3 s (0.69 / 0.80 / 1.19 m per 0.1 s relative
-- to V) because the immediate gate path of the rate limited pull-in ran at 12 m/s whenever V had moved a moment before or was still sliding into the seat; (W4) the boom was 0.73 m
-- when the hold began (V had walked up to the car body) and the pull-out path - frozen in the vehicle frame while V slid toward its start - passed 0.28 m from V's head.
-- The tests replay the sequences in the offline world: V walks up, the mount slides her into the seat (mountDur), a surface (the car body) appears at `from` s after the mount start and stays.
-- Metrics from the rendered camera against V's position (same frame): the move over 0.1 s, and the horizontal distance to V.
runBlock(function()
  local cam, vc = load(makeWorld({}))
  local function mountRun(fps, cc, o)
    o = o or {}
    local tm = math.max(7, 330 / fps + 0.5)
    FPS, DT = fps, 1 / fps
    local smp, W = {}, nil
    local ok, err = pcall(function()
      local w = makeWorld({})
      W = w
      local c, v = load(w)
      for k, val in pairs(cc or {}) do c.cfg[k] = val end
      for k, val in pairs(o.vc or {}) do v.ctrl.cfg[k] = val end
      local sqs = {}
      function sqs:SyncRaycastByCollisionGroup(from, to)
        local mt = w.t - tm
        if mt >= (o.from or 0.10) then
          local dx, dy, dz = to.x - from.x, to.y - from.y, to.z - from.z
          local l = math.sqrt(dx * dx + dy * dy + dz * dz)
          local d = o.dist or 1.0
          if l > d then return true, { position = { x = from.x + dx / l * d, y = from.y + dy / l * d, z = from.z + dz / l * d } } end
        end
        return false, { position = { x = 0, y = 0, z = 0 } }
      end
      w.env.Game.GetSpatialQueriesSystem = function() return sqs end
      local Tl = timeline({ tm = tm, mountDur = o.mountDur or 0.9 })
      while w.t < tm + 1.7 do
        frame(w, Tl.script)
        local r, p = w.rec[#w.rec], w.p.pos
        if w.t >= tm - 0.3 then smp[#smp + 1] = { t = w.t - tm, c = r.pos, v = { x = p.x, y = p.y, z = p.z } } end
      end
    end)
    FPS, DT = 60, 1 / 60
    if not ok then error(err) end
    return smp, W, tm
  end
  -- largest move of the camera relative to V over `span` s and the nearest horizontal distance, over [t0, t1] (s after the mount start)
  local function metrics(smp, t0, t1, span)
    span = span or 0.1
    local maxStep, atStep, minD, atD = 0, nil, 99, nil
    for i, s in ipairs(smp) do
      if s.t >= t0 and s.t <= t1 then
        local rx, ry, rz = s.c.x - s.v.x, s.c.y - s.v.y, s.c.z - s.v.z
        local d = math.sqrt(rx * rx + ry * ry)
        if d < minD then minD, atD = d, s.t end
        local j = nil
        for k = i - 1, 1, -1 do
          if smp[k].t < t0 - 1e-9 then break end
          if smp[k].t <= s.t - span + 1e-9 then j = k; break end
        end
        if j then
          local b = smp[j]
          local sx, sy, sz = rx - (b.c.x - b.v.x), ry - (b.c.y - b.v.y), rz - (b.c.z - b.v.z)
          local st = math.sqrt(sx * sx + sy * sy + sz * sz) * span / math.max(s.t - b.t, 1e-6)
          if st > maxStep then maxStep, atStep = st, s.t end
        end
      end
    end
    return maxStep, atStep, minD, atD
  end
  local function lineOf(W, needle)
    for _, l in ipairs(W.lines) do if string.find(l, needle, 1, true) then return l end end
    return nil
  end
  local OLD = { vehHoldColMountRate = 0, vehHoldColLift = 0 } -- utpp_camera: the r11 pull-in rates
  local OLDV = { pullClearFreeze = false, pullClearR = 0.8 } -- utpp_vehiclecam: the r11 clearance
  for _, fps in ipairs({ 30, 60, 144 }) do
    -- (W1 / W2 / W3) the car body shows up in the boom 0.10 s after the mount start while V slides into the seat: the boom comes in smoothly to the floor, no move
    -- above 0.3 m per 0.1 s relative to V, never closer than 1.0 m, through the pull-out start and the first 0.6 s of the path
    local smp, W, tm = mountRun(fps, nil, {})
    local pullAt = (firstAt(W, "pull-out start") or (tm + 0.8)) - tm
    local sOrbit, aOrbit, dOrbit = metrics(smp, 0, pullAt)
    local sHand, aHand, dHand = metrics(smp, pullAt - 0.1, pullAt + 0.1) -- (the path itself moves faster later: designed, up to 0.7 m per 0.1 s; the watch has its own limit for it)
    local _, _, dPull, aPull = metrics(smp, pullAt, pullAt + 0.7)
    local mline = lineOf(W, "mount camera")
    local smpO, WO = mountRun(fps, OLD, { vc = OLDV })
    local pullO = (firstAt(WO, "pull-out start") or (tm + 0.8)) - tm
    local sOld, aOld = metrics(smpO, 0, pullO)
    local sHandOld = metrics(smpO, pullO - 0.1, pullO + 0.1)
    check(string.format("r12 car mount, %d fps: orbit hold - camera move relative to V <= 0.3 m per 0.1 s (%.2f m at +%.2f s), nearest to V %.2f m; mutation (r11 rates): %.2f m per 0.1 s", fps, sOrbit,
          aOrbit or -1, dOrbit, sOld), sOrbit <= 0.30 and dOrbit >= 1.0 and sOld > 0.5, string.format("%.2f %.2f %.2f", sOrbit, dOrbit, sOld))
    check(string.format("r12 car mount, %d fps: hand-over to the pull-out - move %.2f m per 0.1 s around its start (+%.2f s, +-0.1 s), nearest to V %.2f m through the first 0.7 s of the path (at +%.2f s); r11 rates: %.2f m (the same - the pull-in is over before this replay's pull-out starts; the mutation checks are above and below)", fps,
          sHand, pullAt, dPull, aPull or -1, sHandOld), pullAt > 0.3 and sHand <= 0.30 and dPull >= 1.0 - 0.02, string.format("%.2f %.2f", sHand, dPull))
    check(string.format("r12 car mount, %d fps: the mount camera watch reports no WARNING through the pull-out", fps), mline ~= nil and string.find(mline, "WARNING", 1, true) == nil
          and string.find(mline, "pull-out override", 1, true) ~= nil, tostring(mline))
  end
  -- (W2) the occluder arrives 0.30 s after the mount start, 0.07 s before the pull-out starts in this replay (the log of 13:15:38: hits=1 at +0.87 s, pull-out start at +0.92 s): the boom
  -- is mid pull-in when the path takes over. The camera steps around the hand-over stay <= 0.3 m per 0.1 s; with the r11 rates the pull-in alone moves the camera 1.2 m per 0.1 s there.
  for _, fps in ipairs({ 30, 60, 144 }) do
    local o = { from = 0.30 }
    local smp, W, tm = mountRun(fps, nil, o)
    local pullAt = (firstAt(W, "pull-out start") or (tm + 0.8)) - tm
    local sNew, aNew, dNew = metrics(smp, pullAt - 0.1, pullAt + 0.1)
    local smpO, WO = mountRun(fps, OLD, { from = 0.30, vc = OLDV })
    local pullO = (firstAt(WO, "pull-out start") or (tm + 0.8)) - tm
    local sOld = metrics(smpO, pullO - 0.1, pullO + 0.1)
    check(string.format("r12 car mount with the occluder just before the pull-out (W2), %d fps: pull-out at +%.2f s, move %.2f m per 0.1 s around it, nearest to V %.2f m; mutation (r11 rates): %.2f m per 0.1 s", fps,
          pullAt, sNew, dNew, sOld), pullAt > o.from and sNew <= 0.30 and dNew >= 1.0 and sOld > 0.5, string.format("%.2f %.2f %.2f", sNew, dNew, sOld))
  end
  -- (W4) the boom is already short when the mount hold begins (V walked up to the car body: surface 0.85 m from the pivot since before the mount): the boom rises to the floor at vehHoldColLift m/s
  -- (the first frames inherit the short boom - reported by the watch, see the log), and the pull-out starts from a camera 1.0 m and more from V
  for _, fps in ipairs({ 30, 60, 144 }) do
    local o = { from = -5, dist = 0.85 }
    local smp, W, tm = mountRun(fps, nil, o)
    local pullAt = (firstAt(W, "pull-out start") or (tm + 0.8)) - tm
    local sUp, aUp, dUp = metrics(smp, 0, pullAt)
    local _, _, dLate, aLate = metrics(smp, 0.5, pullAt + 0.7)
    local first = smp[1] and smp[1].t >= 0 and smp[1] or nil
    for _, s in ipairs(smp) do if s.t >= 0 then first = s; break end end
    local d0 = math.sqrt((first.c.x - first.v.x) ^ 2 + (first.c.y - first.v.y) ^ 2)
    local smpO = mountRun(fps, OLD, { from = -5, dist = 0.85, vc = OLDV })
    local _, _, dOld = metrics(smpO, 0.3, ((firstAt(W, "pull-out start") or (tm + 0.8)) - tm) + 0.7)
    check(string.format("r12 short boom at the mount, %d fps: the boom starts at %.2f m, the camera rises with <= 0.3 m per 0.1 s (%.2f m), and is >= 1.0 m from V from +0.5 s through the pull-out (%.2f m at +%.2f s); "
          .. "mutation (r11: the short boom keeps its length): %.2f m", fps, d0, sUp, dLate, aLate or -1, dOld), d0 < 1.0 and sUp <= 0.30 and dLate >= 1.0 - 0.02 and dOld < 0.95,
          string.format("%.2f %.2f %.2f %.2f", d0, sUp, dLate, dOld))
  end
  -- (W4) the pull-out path frozen in the vehicle frame while V slides toward its start: the log of 13:20:25 (start -170 deg, 0.93 m from the vehicle axis, 1.27 m up; V slid about 0.6 m
  -- toward the start within 0.3 s). Offline replay: V moves along a line toward the start point; the shown point (player-local, V = origin) must not come closer to V than its start distance
  -- (the fade only moves it out); the live distance of the start (r11) shrank to 0.1 m and so did the fade's starting radius.
  do
    local deg = math.pi / 180
    local start = { a = -170 * deg, r = 0.93, h = 1.27 }
    local px0, py0, px1, py1 = -1.55, -0.50, -0.95, -0.18 -- V (vehicle frame, vehicle at the origin, yaw 0): from 0.72 m to 0.04 m from the start point (-0.916, -0.161)
    local function replay(c)
      local st = { a = start.a, r = start.r, h = start.h }
      local minD, d0, steps, prev = 99, nil, 0, nil
      for i = 0, 100 do
        local t = i * 0.01 -- s after the start of the pull
        local s = math.min(1, t / 0.5); s = s * s * (3 - 2 * s)
        local f = { px = px0 + (px1 - px0) * s, py = py0 + (py1 - py0) * s, pz = -0.4, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
        local k = math.min(1, t / 1.39)
        local w = k * k * (3 - 2 * k)
        local p = vc.PullPathPose(st, w, f, c)
        local d = math.sqrt(p.x * p.x + p.y * p.y)
        if d0 == nil then d0 = d end
        if p.z > -0.3 and p.z < 2.1 then minD = math.min(minD, d) end
        if i % 10 == 0 then -- 0.1 s: the step of the point relative to V (player-local = V is the origin)
          if prev then steps = math.max(steps, math.sqrt((p.x - prev.x) ^ 2 + (p.y - prev.y) ^ 2 + (p.z - prev.z) ^ 2)) end
          prev = p
        end
      end
      return minD, d0, steps
    end
    local cNew = setmetatable({}, { __index = vc.DEFAULTS })
    local cOld = setmetatable({ pullClearFreeze = false, pullClearR = 0.8 }, { __index = vc.DEFAULTS })
    local mNew, d0, stepNew = replay(cNew)
    local mOld, _, stepOld = replay(cOld)
    -- (the step is the motion of the shown point relative to V over 0.1 s, reported: V's slide itself is 0.6 m in 0.5 s here and the point moves with the push; see the report)
    check(string.format("r12 pull-out path while V slides toward its start: nearest to V %.2f m (start %.2f m, clearance %.2f m), largest step relative to V %.2f m per 0.1 s; mutation (r11: live start "
          .. "distance, 0.8 m): %.2f m, step %.2f m", mNew, d0, cNew.pullClearR, stepNew, mOld, stepOld), mNew >= d0 - 0.02 and mOld < d0 - 0.2, string.format("%.2f %.2f %.2f", mNew, d0, mOld))
  end
  -- the unit: the immediate gate path (no delay) and the delayed path are both capped by vehHoldColMountRate; lengthening is limited by vehHoldColLift
  for _, fps in ipairs({ 30, 60, 144 }) do
    local cfg = cam.cfg
    local dt = 1 / fps
    local function drop(rate, delay, lift)
      local cs = CM.newCollision()
      CM.collisionSmooth(cs, cfg, dt, 2.93, 0, rate, delay, 1.6, lift)
      local prev, worst = cs.len, 0
      for i = 1, math.floor(1.0 * fps) do
        CM.collisionSmooth(cs, cfg, dt, 0.89, 1, rate, delay, 1.6, lift)
        worst = math.max(worst, (prev - cs.len) / dt)
        prev = cs.len
      end
      return worst
    end
    local capped = math.min(cfg.vehHoldColRate, cfg.vehHoldColMountRate)
    local wI, wD, wOld = drop(capped, nil, cfg.vehHoldColLift), drop(capped, cfg.vehHoldColDelay, cfg.vehHoldColLift), drop(cfg.vehHoldColRate, nil, nil)
    check(string.format("r12 pull-in rate, %d fps: immediate path %.2f m/s, delayed path %.2f m/s (cap %.1f m/s); mutation (vehHoldColRate %.0f): %.1f m/s", fps, wI, wD, capped, cfg.vehHoldColRate, wOld),
          wI <= capped + 1e-6 and wD <= capped + 1e-6 and wOld >= 11.9 and capped * 0.1 <= 0.3 - 0.04)
    -- lengthening: a boom at the floor and a surface that goes away rises at <= vehHoldColLift m/s (the r11 return was exponential, about 3 m/s per m of gap)
    local function rise(lift)
      local cs = CM.newCollision()
      CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 2.4, nil, 1.6, lift)
      for _ = 1, math.floor(0.6 * fps) do CM.collisionSmooth(cs, cfg, dt, 0.89, 1, 2.4, nil, 1.6, lift) end
      local prev, worst = cs.len, 0
      for _ = 1, math.floor(1.0 * fps) do
        CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 2.4, nil, 1.6, lift)
        worst = math.max(worst, (cs.len - prev) / dt)
        prev = cs.len
      end
      return worst
    end
    local rL, rO = rise(cfg.vehHoldColLift), rise(nil)
    check(string.format("r12 boom return, %d fps: lengthening %.2f m/s with vehHoldColLift %.1f; mutation (no lift limit): %.1f m/s", fps, rL, cfg.vehHoldColLift, rO), rL <= cfg.vehHoldColLift + 1e-6 and rO > 5)
    -- a short boom with an occluder rises to the floor at the lift rate; without the lift it keeps its length
    local function shortBoom(lift)
      local cs = CM.newCollision()
      CM.collisionSmooth(cs, cfg, dt, 0.73, 0, 2.4, nil, 1.6, lift)
      local worst, prev = 0, cs.len
      for _ = 1, math.floor(1.5 * fps) do
        CM.collisionSmooth(cs, cfg, dt, 0.5, 1, 2.4, nil, 1.6, lift)
        worst = math.max(worst, (cs.len - prev) / dt)
        prev = cs.len
      end
      return cs.len, worst
    end
    local lenL, wL = shortBoom(cfg.vehHoldColLift)
    local lenO = shortBoom(nil)
    check(string.format("r12 short boom, %d fps: rises to the floor (%.2f m) at %.2f m/s; mutation (no lift): stays %.2f m", fps, lenL, wL, lenO),
          math.abs(lenL - 1.6) <= cfg.deadzone + 1e-6 and wL <= cfg.vehHoldColLift + 1e-6 and math.abs(lenO - 0.73) < 1e-9)
  end
  -- the watch through the override: a camera sample with shown = true (the pull-out pose) counts against V
  do
    local function watch(shownFn, withShown)
      local st, n, out = {}, 0, nil
      local V0 = { x = 0, y = 0, z = 0 }
      local function step(t, mounted, c)
        n = n + 1
        local _, _, _, cl = cam.MountShiftGate(st, cam.cfg, t, n, "tpp", mounted, { held = true }, V0, c)
        out = out or cl
      end
      for t = 0, 0.5, 1 / 60 do step(t, false, nil) end
      for t = 0.51, 2.4, 1 / 60 do
        local age = t - 0.51
        local c
        if age < 0.9 then c = { x = 2.5, y = 0.7, z = 1.5, arm = 2.93, fullLen = 2.95 }
        elseif withShown then local p = shownFn(age - 0.9); c = { x = p.x, y = p.y, z = p.z, shown = true } end
        step(t, true, c)
      end
      return out
    end
    -- a designed pull-out: 0.7 m per 0.1 s (7 m/s) from the orbit position, away from V
    local smooth = function(a) return { x = 2.5 + 7 * a, y = 0.7, z = 1.5 } end
    local good = watch(smooth, true)
    -- a jump of 0.6 m at the hand-over, and a path through V
    local jump = watch(function(a) return { x = 3.1 + 7 * a, y = 0.7, z = 1.5 } end, true)
    local near = watch(function(a) return { x = 2.5 - 2.0 * math.min(1, a / 0.3), y = 0.7, z = 1.5 } end, true)
    local ended = watch(smooth, false)
    check("r12 watch: a designed pull-out (7 m/s) is no WARNING and is reported with the override samples", good ~= nil and string.find(good, "WARNING", 1, true) == nil and string.find(good, "pull-out override", 1, true) ~= nil, tostring(good))
    check("r12 watch: a 0.6 m jump at the hand-over from the orbit to the override is a WARNING", jump ~= nil and string.find(jump, "mount camera WARNING", 1, true) ~= nil, tostring(jump))
    check("r12 watch: a pull-out that comes within 1 m of V is a WARNING", near ~= nil and string.find(near, "mount camera WARNING", 1, true) ~= nil, tostring(near))
    check("r12 watch mutation (no override samples = the r11 watch): the same jump ends the window without the pull-out part", ended ~= nil and string.find(ended, "pull-out override", 1, true) == nil, tostring(ended))
  end
end)

-- ================================================================ r13e exit pose governor + release speed
-- r13d game logs (standard3 'behind' 535.24 - 538.58): the exit match crossed 7.7 m (vehicle camera (-7.35,-6.03,4.67) -> the vehicle-behind pose) in
-- 1.00 s, 0.17 - 0.26 m per frame, the release to the orbit 4.2 m in 0.80 s, 0.10 - 0.13 m per frame. Replayed with the logged vehicle camera pose, the
-- logged unmount times (2.04 s normal exit, 1.04 s the jump-out's) at 30 / 60 / 144 fps: per-frame rendered step beyond V's own step (scaled to 60 fps)
-- and per-frame rotation over the exit, the box, the hold end; mutation = vehExitGov / vehReleaseSpeed off (the r13d behaviour).
do
  local LOGFAR = { x = -7.35, y = -6.03, z = 4.67, pitch = -11, yaw = -48 }
  local function exitRun(fps, exitDur, off, far)
    FPS, DT = fps, 1 / fps
    local W, cam, vc, T = session({ far = far or LOGFAR, exitDur = exitDur, after = 3.5, tm = fps < 60 and 14.0 or nil,
      before = off and function(_, c) c.cfg.vehExitGov = false; c.cfg.vehReleaseSpeed = 0 end or nil })
    FPS, DT = 60, 1 / 60
    local maxS, atS, maxR, atR = 0, nil, 0, nil
    local offAt = nil
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if offAt == nil and b.t >= T.tx and a.held and not b.held then offAt = b.t end
    end
    local t1 = (offAt or T.te) + 2.5 / fps -- (through the hand-over to the camera's own rules; the follow swing onto V's walking heading after it is on-foot behaviour)
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if b.t >= T.tx and b.t <= t1 then
        local dtf = b.t - a.t
        local own = CM.dist(a.ppos, b.ppos)
        local s = math.max(0, CM.dist(a.pos, b.pos) - own) * (1 / 60) / dtf -- (the step a 60 fps frame would show)
        if s > maxS then maxS, atS = s, b.t end
        local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd)))))) * (1 / 60) / dtf
        if ang > maxR then maxR, atR = ang, b.t end
      end
    end
    if os.getenv("UTPP_R13E_DEBUG") then
      for i = 2, #W.rec do
        local a, b = W.rec[i - 1], W.rec[i]
        if b.t >= (offAt or 0) - 1.2 and b.t <= (offAt or 0) + 0.5 then
          local ang = math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd))))))
          local yw, pt = yawPitch(b.fwd)
          print(string.format("dbg %d %.3f step=%.3f own=%.3f ang=%.2f yaw=%.1f pitch=%.1f held=%s", fps, b.t, CM.dist(a.pos, b.pos), CM.dist(a.ppos, b.ppos), ang, yw, pt, tostring(b.held)))
        end
      end
    end
    return { W = W, T = T, maxS = maxS, atS = atS, maxR = maxR, atR = atR, offAt = offAt, box = boxFrames(W, T.tx, T.te),
             lines = count(W, "exit match: cam=") }
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    for _, ed in ipairs({ 2.04, 1.04 }) do
      local r = exitRun(fps, ed)
      check(string.format("r13e exit %d fps, unmount %.2f s after the exit start: rendered step beyond V's own <= 0.10 m per 60 fps frame (%.3f at %.2f), rotation <= 1.6 deg (%.2f at %.2f), no box frame, the hold ends (%.2f s after the exit start)",
        fps, ed, r.maxS, r.atS or -1, r.maxR, r.atR or -1, r.offAt and (r.offAt - r.T.tx) or -1),
        r.lines == 1 and r.maxS <= 0.10 and r.maxR <= 1.6 and r.box == 0 and r.offAt ~= nil and r.offAt - r.T.tx <= ed + 2.6,
        dump(r.W, { "exit match", "release", "hold OFF", "unmount" }, r.T.tx))
    end
  end
  -- the other r13d / r13c pose families: utility4 (the high vehicle camera far behind, unmount 2.70 s) and standard3 'parallel' (the vehicle camera
  -- beside the car, yaw -95: utpp_vehiclecam ends it with its follow release, 'exit follow (unmount after a jump-out, vehicle 0.0 m/s)')
  local FAMS = { { name = "utility4", far = { x = -0.08, y = -10.79, z = 6.51, pitch = -10, yaw = 0 }, ed = 2.70 },
                 { name = "standard3 parallel", far = { x = -9.85, y = 1.39, z = 4.42, pitch = -9, yaw = -95 }, ed = 2.05 } }
  for _, fam in ipairs(FAMS) do
    for _, fps in ipairs({ 30, 60, 144 }) do
      local r = exitRun(fps, fam.ed, false, fam.far)
      check(string.format("r13e exit %s %d fps, unmount %.2f s after the exit start: rendered step beyond V's own <= 0.10 m per 60 fps frame (%.3f at %.2f), rotation <= 1.6 deg (%.2f at %.2f), no box frame, the hold ends (%.2f s after the exit start; follow releases %d)",
        fam.name, fps, fam.ed, r.maxS, r.atS or -1, r.maxR, r.atR or -1, r.offAt and (r.offAt - r.T.tx) or -1, count(r.W, "exit follow (")),
        r.lines == 1 and r.maxS <= 0.10 and r.maxR <= 1.6 and r.box == 0 and r.offAt ~= nil and r.offAt - r.T.tx <= fam.ed + 2.6,
        dump(r.W, { "exit match", "exit follow", "release", "hold OFF", "unmount" }, r.T.tx))
    end
  end
  -- the logged 'parallel' exit (r13c 22-23-06.1 t=249.57) was a rolling one: the vehicle at 4.2 m/s at the exit start (utpp_vehiclecam: jump-out,
  -- VehicleJumpOut armed), V on the ground, the unmount 2.05 s later, then 'exit follow (unmount after a jump-out)' 3.85 m from the orbit in 0.25 s
  local function rollRun(fps, mut)
    FPS, DT = fps, 1 / fps
    local W, _, _, T = jumpOut(true, 0.4, 0.2, 9.81, mut and { vehExitGovJumpSpeed = 0 } or nil,
      { speed = 4.2, rise = 0.3 / 2.05, tl = { exitDur = 2.05, after = 3.5, tm = fps < 60 and 14.0 or nil } })
    FPS, DT = 60, 1 / 60
    local offAt
    for i = 2, #W.rec do if offAt == nil and W.rec[i].t >= T.tx and W.rec[i - 1].held and not W.rec[i].held then offAt = W.rec[i].t end end
    local maxS, atS, maxR = 0, nil, 0
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if b.t >= T.tx and b.t <= (offAt or T.te) + 2.5 / fps then
        local dtf = b.t - a.t
        local s = math.max(0, CM.dist(a.pos, b.pos) - CM.dist(a.ppos, b.ppos)) * (1 / 60) / dtf
        if s > maxS then maxS, atS = s, b.t end
        maxR = math.max(maxR, math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd)))))) * (1 / 60) / dtf)
      end
    end
    if os.getenv("UTPP_R13E_DEBUG") then
      for _, l in ipairs(W.lines) do if l:find("exit", 1, true) or l:find("release", 1, true) or l:find("jump", 1, true) or l:find("hold O", 1, true) or l:find("flight", 1, true) or l:find("governor", 1, true) then print("rdbg " .. fps .. " " .. l:sub(1, 200)) end end
      for i = 2, #W.rec do local a, b = W.rec[i - 1], W.rec[i]
        if atS and b.t >= atS - 0.4 and b.t <= atS + 0.1 then print(string.format("rdbg %d %.3f step=%.3f own=%.3f held=%s z=%.3f", fps, b.t, CM.dist(a.pos, b.pos), CM.dist(a.ppos, b.ppos), tostring(b.held), b.ppos.z)) end end
    end
    return { W = W, T = T, maxS = maxS, atS = atS, maxR = maxR, offAt = offAt, jo = count(W, "exit while the vehicle moves"), fol = count(W, "exit follow (unmount after a jump-out") }
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local r = rollRun(fps)
    check(string.format("r13e rolling exit (4.2 m/s, jump-out class, V on the ground) %d fps: step beyond V's own <= 0.10 m per 60 fps frame (%.3f at %.2f), rotation <= 1.6 deg (%.2f), the hold ends",
      fps, r.maxS, r.atS or -1, r.maxR), r.jo == 1 and r.fol == 1 and r.maxS <= 0.10 and r.maxR <= 1.6 and r.offAt ~= nil,
      dump(r.W, { "exit while", "exit follow", "release", "hold OFF", "unmount", "jump-out" }, r.T.tx))
  end
  local rm = rollRun(60, true)
  check(string.format("r13e rolling exit mutation (vehExitGovJumpSpeed = 0: the landing mode ends the governor at once): %.3f m per frame (> 0.15)", rm.maxS), rm.maxS > 0.15, rm.maxS)
  local m = exitRun(60, 2.04, true)
  check(string.format("r13e mutation (vehExitGov = false, vehReleaseSpeed = 0 = r13d): the same exit steps %.3f m per frame (> 0.15)", m.maxS), m.maxS > 0.15, m.maxS)
  -- a slow request is shown as it is: the governor never moves a pose that already moves within its limits (no lag on V's own motion)
  local g = CM.poseGovNew({ x = 0, y = 0, z = 0, yaw = 0, pitch = -10, fov = 60 })
  local cfg = CM.config({})
  local worst = 0
  for i = 1, 240 do
    local t = i / 60
    local req = { x = 1.5 * t, y = 0.2 * t, z = 0, yaw = 20 * t, pitch = -10, fov = 60 }
    local o = CM.poseGovStep(g, req, 1 / 60, cfg, 1.5)
    if t > 0.5 then worst = math.max(worst, math.sqrt((o.x - req.x) ^ 2 + (o.y - req.y) ^ 2), math.abs(CM.wrapDeg(o.yaw - req.yaw)) / 100) end
  end
  check(string.format("r13e governor: a request moving at 1.5 m/s / 20 deg/s is followed exactly after its start (max gap %.4f)", worst), worst <= 1e-6, worst)
end

-- ================================================================ r13f rolling exit of a car that stops before the unmount (standard3 'parallel', t=259.07 - 262.43)
-- The car rolled 4.2 m/s at the exit start (jump-out by the speed) and stood still from about 0.6 s later; the unmount 2.05 s after the exit start took the
-- straight 0.25 s follow ('exit follow (unmount after a jump-out, vehicle 0.0 m/s)'), the exit pose governor had not finished it when the hold ended
-- 0.5 s later and the release ran from behind the car (0.11,-4.56,2.02) to the orbit (-1.17,-1.88,1.22) through the rear corner of the box: 7 frames,
-- 0.14 m at veh (-0.94,-2.36,1.36). Replayed with the logged exit camera, the standard3 record (its box), V standing beside the rear door 0.48 m below the
-- vehicle origin and the orbit boom shortened to 1.3 m after the unmount (the logged dist=1.27/2.95 hits=4). The rendered camera is never inside the box of
-- the record (depth > 0, the product's M.HullDepth), the step / rotation limits of r13e hold; mutation exitFollowAroundAtRest = false (r13e = HEAD 2b318a9).
runBlock(function()
  local REC = "Vehicle.v_standard3_chevalier_emperor"
  local PAR = { x = -9.49, y = -2.09, z = 4.63, pitch = -10, yaw = -75 } -- the logged exit match pose ('exit match: cam=(-9.49,-2.09,4.63) yaw=-75 pitch=-10')
  local PAR2 = { x = -7.61, y = -5.70, z = 4.74, pitch = -11, yaw = -51 } -- the second logged exit (t=291.08; 5 frames, 0.10 m at t=294.13 - 294.19)
  local function rollStop(fps, vcc, moving, far)
    FPS, DT = fps, 1 / fps
    local W = makeWorld({ far = far or PAR, recordId = REC })
    local cam, vc = load(W)
    for k, v in pairs(vcc or {}) do vc.ctrl.cfg[k] = v end
    local T = timeline({ ride = 3.0, exitDur = 2.05, after = 3.5, tm = fps < 60 and 14.0 or nil })
    local base = T.script
    local seat, out = CM.v3(-0.35, 0.25, -0.3), CM.v3(-1.30, -0.7, -0.48)
    local sqs = {} -- after the unmount every camera ray hits a surface 1.3 m from its start (the orbit boom of the log: 1.27 m)
    function sqs:SyncRaycastByCollisionGroup(from, to)
      local dx, dy, dz = to.x - from.x, to.y - from.y, to.z - from.z
      local L = math.sqrt(dx * dx + dy * dy + dz * dz)
      if W.t <= T.tu or L <= 1.3 then return false, { position = { x = 0, y = 0, z = 0 } } end
      local k = 1.3 / L
      return true, { position = { x = from.x + dx * k, y = from.y + dy * k, z = from.z + dz * k } }
    end
    W.env.Game.GetSpatialQueriesSystem = function() return sqs end
    local function script(W, t)
      local v = W.v
      local function rel(p) return W.vehToWorld(p.x, p.y, p.z) end
      if t < T.td then
        base(W, t)
      elseif t < T.tx then
        W.vs, W.mounted = 1, true
        local sp = math.max(0, math.min(4.2, 4.2 * (t - T.td - 0.5)))
        v.pos = CM.v3(v.pos.x, v.pos.y + sp * DT, v.pos.z)
        W.p.pos, W.p.yaw, W.p.vel = rel(seat), v.yaw, CM.v3(0, sp, 0)
      elseif t < T.tu then
        if W.vs ~= 4 then W.exitStart() end
        local k = t - T.tx
        local sp = moving and 4.2 or 4.2 * math.max(0, 1 - k / 0.6) -- (moving: the car still rolls 4.2 m/s at the unmount, the control)
        v.pos = CM.v3(v.pos.x, v.pos.y + sp * DT, v.pos.z)
        W.p.pos, W.p.yaw, W.p.vel = lerp3(rel(seat), rel(out), smooth(k / (T.tu - T.tx))), lerp(0, 4, smooth(k / (T.tu - T.tx))), nil
      else
        W.mounted, W.vs = false, 0
        if moving then v.pos = CM.v3(v.pos.x, v.pos.y + 4.2 * DT, v.pos.z) end
        W.p.pos, W.p.yaw, W.p.vel = rel(out), 4, nil
      end
    end
    run(W, T.te, script)
    FPS, DT = 60, 1 / 60
    local hull = vc.HullFor(vc.ctrl.cfg, REC)
    local inBox, deep, deepAt, maxS, atS, maxR, offAt = 0, -99, nil, 0, nil, 0, nil
    for i = 2, #W.rec do
      local a, b = W.rec[i - 1], W.rec[i]
      if offAt == nil and b.t >= T.tx and a.held and not b.held then offAt = b.t end
      if b.t >= T.tx then
        local l = W.worldToVeh(b.pos)
        local d = vc.HullDepth(hull, l.x, l.y, l.z)
        if d > 0 then inBox = inBox + 1 end
        if d > deep then deep, deepAt = d, b.t end
        local dtf = b.t - a.t
        local s = math.max(0, CM.dist(a.pos, b.pos) - CM.dist(a.ppos, b.ppos)) * (1 / 60) / dtf
        if s > maxS then maxS, atS = s, b.t end
        maxR = math.max(maxR, math.deg(math.acos(math.max(-1, math.min(1, CM.dot(CM.norm(a.fwd), CM.norm(b.fwd)))))) * (1 / 60) / dtf)
      end
    end
    return { W = W, T = T, inBox = inBox, deep = deep, deepAt = deepAt, maxS = maxS, atS = atS, maxR = maxR, offAt = offAt,
             warn = count(W, "camera inside the vehicle box WARNING: "), jo = count(W, "exit while the vehicle moves 4.2"),
             rest = count(W, "the vehicle at rest: around the vehicle"), fol = count(W, "exit follow (unmount after a jump-out") }
  end
  for _, fp in ipairs({ { "exit 1", PAR }, { "exit 2", PAR2 } }) do for _, fps in ipairs({ 30, 60, 144 }) do
    local r = rollStop(fps, nil, false, fp[2])
    check(string.format("r13f rolling exit (%s), car at rest at the unmount, %d fps: jump-out by the speed, follow around the vehicle, rendered camera never inside "
      .. "the standard3 box (%d frames, deepest %.3f m at +%.2f s), step beyond V's own <= 0.10 m per 60 fps frame (%.3f at +%.2f), rotation <= 1.6 deg (%.2f), the hold ends",
      fp[1], fps, r.inBox, r.deep, (r.deepAt or r.T.tx) - r.T.tx, r.maxS, (r.atS or r.T.tx) - r.T.tx, r.maxR),
      r.jo == 1 and r.fol == 1 and r.rest == 1 and r.inBox == 0 and r.warn == 0 and r.maxS <= 0.10 and r.maxR <= 1.6 and r.offAt ~= nil,
      dump(r.W, { "exit while", "exit follow", "release", "hold OFF", "unmount", "vehicle box" }, r.T.tx))
  end end
  local m = rollStop(60, { exitFollowAroundAtRest = false })
  check(string.format("r13f mutation (exitFollowAroundAtRest = false = r13e): the straight follow + governed release enters the box (%d frames, deepest %.3f m; the game's WARNING lines %d)",
    m.inBox, m.deep, m.warn), m.rest == 0 and m.inBox > 0 and m.deep > 0.05 and m.warn >= 1, m.deep)
  -- control: the car still rolls at the unmount -> the straight follow of a jump-out, bit for bit the path with the switch off
  local c1, c0 = rollStop(60, nil, true), rollStop(60, { exitFollowAroundAtRest = false }, true)
  local same = #c1.W.rec == #c0.W.rec
  for i = 1, #c1.W.rec do if not same then break end
    local a, b = c1.W.rec[i], c0.W.rec[i]
    same = b ~= nil and CM.dist(a.pos, b.pos) == 0 and CM.dist(a.fwd, b.fwd) == 0
  end
  check("r13f control: a car still rolling at the unmount keeps the straight jump-out follow, bit for bit the path with exitFollowAroundAtRest = false",
    same and c1.rest == 0 and c1.fol == 1, string.format("same %s, rest %d, follows %d", tostring(same), c1.rest, c1.fol))
end)

print(string.format("utpp vehicle path test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp vehicle path test failed")

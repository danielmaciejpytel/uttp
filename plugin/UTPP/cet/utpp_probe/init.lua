-- UTPP probe: CET test helper (not part of any release).
--
-- Body / camera investigation, step 0: one-shot dumps of the player
-- components (class, name, enabled, appearance, mesh / morph resource hash, mesh appearance, chunk mask), the items in
-- the body slots (record, appearance, their mesh components) and the camera components, before / after the game's
-- TPP representation is switched on and off. Output: UTPP red4ext log (Game.UTPPLogLine) and probe_dump.log in this
-- mod folder. Lines: "D|tag|..." component, "I|tag|..." item, "IC|tag|..." item component, "C|tag|..." camera,
-- "DIFF|a>b|..." change.
--
-- Console (CET):  local p = GetMod("utpp_probe")
--   p.Dump("before")            p.Diff("before", "rep_on")
--   p.Cycle()                   -- dump before -> representation ON -> dump rep_on (+1.5 s) -> diff -> OFF -> dump rep_off
--   p.RepOn()  p.RepOff()       -- representation only (ActivateTPPRepresentationEvent + anim feature TPPRepresentation)
--   p.Cameras()                 -- camera components + active camera (position, forward, FOV)
--   p.CamSpike(true|false)      -- variant A1 spike: activate utppCamera 2.2 m behind V without the controller
--   p.Continuous(true|false)    -- old Phase 2 mode: 15 deg Spine3 test rotation + UTPP status every 5 s (off by default)
--   p.AimProbe(true|false)      -- probe S1: 4 samples/s + one line per player shot, summary on false
--   p.AimProbeSample()          -- one sample line now (cameras, crosshair data, muzzle, PSM UpperBody, item)
--                               -- 2026-10-30 (aim pitch step 0): while AimProbe is on also "aimaxis" (muzzle slot axis in
--                               -- the WeaponRight frame, pitches vs muzzle -> P) and per-weapon "aimaxis summary" lines

local P = { t = 0.0, queue = {}, dumps = {}, continuous = false, frames = 0, frameTime = 0.0, nextLog = 0.0,
            configured = false, LOG_FILE = "probe_dump.log" }

local function fileLog(msg)
  local f = io.open(P.LOG_FILE, "a")
  if f then
    f:write(string.format("[%8.2f] %s\n", P.t, tostring(msg)))
    f:close()
  end
end
function P.Log(msg)
  local line = "[utpp_probe] " .. tostring(msg)
  fileLog(line)
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(line) end) end
end

local function str(fn)
  local ok, v = pcall(fn)
  if not ok or v == nil then return "-" end
  local okN, s = pcall(function() return Game.NameToString(v) end)
  if okN and type(s) == "string" and s ~= "" then return s end
  return tostring(v)
end
local function resHash(ref)
  if ref == nil then return "-" end
  local forms = {
    function() return ResourceAsyncRef.GetHash(ref) end,
    function() return ref.hash end,
    function() return ResourceAsyncRef.GetPath(ref).hash end,
  }
  for _, f in ipairs(forms) do
    local ok, h = pcall(f)
    if ok and h ~= nil then return tostring(h) end
  end
  return tostring(ref)
end
local function field(obj, name)
  local ok, v = pcall(function() return obj[name] end)
  if ok then return v end
  return nil
end
local function vecs(v)
  if v == nil then return "-" end
  return string.format("%.3f,%.3f,%.3f", v.x or 0, v.y or 0, v.z or 0)
end

-- ================================================================ dumps
local SLOTS = { "Head", "Face", "Eyes", "TppHead", "Chest", "Torso", "InnerChest", "OuterChest", "Outfit", "Legs", "Feet",
                "RightArm", "LeftArm", "WeaponRight", "WeaponLeft" }

local function componentRecord(c, idx)
  local cls = str(function() return c:GetClassName() end)
  local rec = {
    idx = idx, cls = cls, name = str(function() return c:GetName() end),
    en = (function() local ok, v = pcall(function() return c:IsEnabled() end); return ok and tostring(v) or "?" end)(),
    app = str(function() return c:GetAppearanceName() end),
  }
  local mesh = field(c, "mesh")
  local morph = field(c, "morphResource")
  rec.mesh = mesh ~= nil and resHash(mesh) or "-"
  rec.morph = morph ~= nil and resHash(morph) or "-"
  rec.meshApp = str(function() return c.meshAppearance end)
  local cm = field(c, "chunkMask")
  rec.chunk = cm ~= nil and tostring(cm) or "-"
  return rec
end

local function recLine(kind, tag, r)
  return string.format("%s|%s|%d|%s|%s|en=%s|app=%s|meshApp=%s|mesh=%s|morph=%s|chunk=%s", kind, tag, r.idx, r.cls,
    r.name, r.en, r.app, r.meshApp, r.mesh, r.morph, r.chunk)
end

function P.Dump(tag)
  tag = tostring(tag or ("t" .. math.floor(P.t)))
  local player = Game.GetPlayer()
  if not player then P.Log("Dump: no player"); return nil end
  local d = { comps = {}, order = {}, items = {} }
  P.Log(string.format("DUMP %s begin: gender=%s template=%s appearance=%s pos=%s", tag,
    str(function() return player:GetResolvedGenderName() end),
    str(function() return player:GetTemplatePath().hash end),
    str(function() return player:GetCurrentAppearanceName() end), vecs(player:GetWorldPosition())))
  local ok, comps = pcall(function() return player:GetComponents() end)
  if not ok or comps == nil then
    P.Log("GetComponents FAILED (Codeware missing?): " .. tostring(comps))
  else
    local seen = {}
    for i, c in ipairs(comps) do
      local r = componentRecord(c, i)
      seen[r.name] = (seen[r.name] or 0) + 1
      local key = r.name .. "#" .. seen[r.name]
      d.comps[key] = r
      d.order[#d.order + 1] = key
      P.Log(recLine("D", tag, r))
    end
    P.Log(string.format("DUMP %s components: %d", tag, #comps))
  end
  local ts = Game.GetTransactionSystem()
  for _, s in ipairs(SLOTS) do
    local okI, item = pcall(function() return ts:GetItemInSlot(player, TweakDBID.new("AttachmentSlots." .. s)) end)
    if okI and item ~= nil then
      local id = item:GetItemID()
      local rec = str(function() return TDBID.ToStringDEBUG(ItemID.GetTDBID(id)) end)
      local app = str(function() return ts:GetItemAppearance(player, id) end)
      local entApp = str(function() return item:GetCurrentAppearanceName() end)
      d.items[s] = { rec = rec, app = app, entApp = entApp }
      P.Log(string.format("I|%s|%s|%s|app=%s|entApp=%s|template=%s", tag, s, rec, app, entApp,
        str(function() return item:GetTemplatePath().hash end)))
      local okC, ic = pcall(function() return item:GetComponents() end)
      if okC and ic then
        for i, c in ipairs(ic) do
          local r = componentRecord(c, i)
          if r.mesh ~= "-" or r.morph ~= "-" then
            d.items[s][r.name .. "#" .. i] = r
            P.Log(recLine("IC", tag .. "|" .. s, r))
          end
        end
      end
    else
      d.items[s] = { rec = "-" }
    end
  end
  P.dumps[tag] = d
  P.Cameras(tag)
  P.Log("DUMP " .. tag .. " end")
  return d
end

function P.Diff(a, b)
  local da, db = P.dumps[a], P.dumps[b]
  if not da or not db then P.Log("Diff: missing dump " .. tostring(da and b or a)); return 0 end
  local n = 0
  local function cmp(key, ra, rb)
    if ra == nil then n = n + 1; P.Log(string.format("DIFF|%s>%s|added|%s", a, b, recLine("D", b, rb))); return end
    if rb == nil then n = n + 1; P.Log(string.format("DIFF|%s>%s|removed|%s", a, b, recLine("D", a, ra))); return end
    local ch = {}
    for _, f in ipairs({ "en", "app", "meshApp", "mesh", "morph", "chunk" }) do
      if ra[f] ~= rb[f] then ch[#ch + 1] = string.format("%s:%s->%s", f, ra[f], rb[f]) end
    end
    if #ch > 0 then
      n = n + 1
      P.Log(string.format("DIFF|%s>%s|%s|%s|%s", a, b, rb.cls, key, table.concat(ch, " ")))
    end
  end
  local keys = {}
  for k in pairs(da.comps) do keys[k] = true end
  for k in pairs(db.comps) do keys[k] = true end
  for k in pairs(keys) do cmp(k, da.comps[k], db.comps[k]) end
  for _, s in ipairs(SLOTS) do
    local ia, ib = da.items[s] or {}, db.items[s] or {}
    if ia.rec ~= ib.rec or ia.app ~= ib.app or ia.entApp ~= ib.entApp then
      n = n + 1
      P.Log(string.format("DIFF|%s>%s|slot %s|%s app=%s entApp=%s -> %s app=%s entApp=%s", a, b, s, tostring(ia.rec),
        tostring(ia.app), tostring(ia.entApp), tostring(ib.rec), tostring(ib.app), tostring(ib.entApp)))
    end
  end
  P.Log(string.format("DIFF %s>%s: %d changes", a, b, n))
  return n
end

-- ================================================================ cameras
local function activeCamera()
  local cs = Game.GetCameraSystem()
  local pos = nil
  local ok, a, b = pcall(function() return cs:GetActiveCameraWorldTransform() end)
  for _, r in ipairs({ b, a }) do
    if pos == nil and ok and r ~= nil and type(r) ~= "boolean" then
      local okP, p = pcall(function() return r.position end)
      if okP and p then pos = p end
    end
  end
  if pos == nil then
    pcall(function()
      local xf = Transform.new()
      cs:GetActiveCameraWorldTransform(xf)
      pos = xf.position
    end)
  end
  local okF, f = pcall(function() return cs:GetActiveCameraForward() end)
  local okV, fov = pcall(function() return cs:GetActiveCameraFOV() end)
  return pos, okF and f or nil, okV and fov or nil
end
P.activeCamera = activeCamera

function P.Cameras(tag)
  tag = tostring(tag or "cams")
  local player = Game.GetPlayer()
  if not player then return end
  local pos, fwd, fov = activeCamera()
  P.Log(string.format("C|%s|active|pos=%s|fwd=%s|fov=%s", tag, vecs(pos), vecs(fwd), tostring(fov)))
  for _, name in ipairs({ "camera", "utppCamera", "tppCamera", "vehicleTPPCamera", "WorldSpaceBlendCamera" }) do
    local okC, c = pcall(function() return player:FindComponentByName(name) end)
    if okC and c ~= nil then
      local wpos = nil
      local okM, m = pcall(function() return c:GetLocalToWorld() end)
      if okM and m then wpos = { x = m.W.x, y = m.W.y, z = m.W.z } end
      local lq = nil
      pcall(function() lq = c:GetLocalOrientation() end)
      P.Log(string.format("C|%s|%s|class=%s|en=%s|fov=%s|local=%s|q=%s|world=%s|dActive=%s|matrixY=%s|heading=%s|pitchMin=%s|pitchMax=%s",
        tag, name, str(function() return c:GetClassName() end), tostring(c:IsEnabled()), str(function() return c:GetFOV() end),
        vecs(c:GetLocalPosition()), lq and string.format("%.3f,%.3f,%.3f,%.3f", lq.i, lq.j, lq.k, lq.r) or "-",
        vecs(wpos), (wpos and pos) and string.format("%.3f", math.sqrt((wpos.x - pos.x) ^ 2 + (wpos.y - pos.y) ^ 2 + (wpos.z - pos.z) ^ 2)) or "-",
        okM and m and vecs({ x = m.Y.x, y = m.Y.y, z = m.Y.z }) or "-",
        tostring(field(c, "headingLocked")), tostring(field(c, "pitchMin")), tostring(field(c, "pitchMax"))))
    else
      P.Log(string.format("C|%s|%s|missing", tag, name))
    end
  end
end

-- ================================================================ representation
local function representation(on)
  local player = Game.GetPlayer()
  if not player then return false end
  local ok, err = pcall(function()
    player:QueueEvent(NewObject(on and "gameActivateTPPRepresentationEvent" or "gameDeactivateTPPRepresentationEvent"))
  end)
  local okF, errF = pcall(function()
    local feat = NewObject("gameAnimFeature_TPPRepresentation")
    feat.IsActive = on and true or false
    AnimationControllerComponent.ApplyFeature(player, "TPPRepresentation", feat, 0)
  end)
  P.Log(string.format("representation %s: event %s, feature %s", on and "ON" or "OFF", ok and "ok" or tostring(err),
    okF and "ok" or tostring(errF)))
  return ok
end
function P.RepOn() return representation(true) end
function P.RepOff() return representation(false) end

local function after(delay, fn) P.queue[#P.queue + 1] = { at = P.t + delay, fn = fn } end
function P.Cycle(delay)
  delay = tonumber(delay) or 1.5
  P.Dump("before")
  P.RepOn()
  after(delay, function() P.Dump("rep_on"); P.Diff("before", "rep_on"); P.RepOff() end)
  after(2 * delay + 0.1, function() P.Dump("rep_off"); P.Diff("before", "rep_off"); P.Log("CYCLE done") end)
  return true
end

-- ================================================================ camera spike (variant A1 takes over the view?)
function P.CamSpike(on)
  local player = Game.GetPlayer()
  if not player then return false end
  local okM, cam = pcall(function() return GetMod("utpp_camera") end)
  if okM and type(cam) == "table" and cam.T and cam.T.state ~= "fpp" then
    P.Log("CamSpike refused: utpp_camera is active (" .. tostring(cam.T.state) .. ")")
    return false
  end
  local c = player:FindComponentByName("utppCamera")
  local fpp = player:FindComponentByName("camera")
  if not c then P.Log("CamSpike: utppCamera missing - ArchiveXL patch not loaded (see red4ext/logs/archive_xl.log)"); return false end
  if on then
    local ok, err = pcall(function()
      c:SetLocalTransform(Vector4.new(0.4, -2.2, 1.7, 1), Quaternion.new(0, 0, 0, 1))
      c:SetFOV(55)
      c:Activate(0, true)
    end)
    P.Log("CamSpike ON: " .. (ok and "Activate ok" or ("FAILED " .. tostring(err))) .. " - takeover check in 0.3 s")
    after(0.3, function()
      local now = activeCamera()
      local wpos = nil
      pcall(function() local m = c:GetLocalToWorld(); wpos = { x = m.W.x, y = m.W.y, z = m.W.z } end)
      local d = (now and wpos) and math.sqrt((now.x - wpos.x) ^ 2 + (now.y - wpos.y) ^ 2 + (now.z - wpos.z) ^ 2) or -1
      P.Log(string.format("CamSpike result: active camera %s, utppCamera world %s, distance %.3f -> %s", vecs(now), vecs(wpos), d,
        (d >= 0 and d < 0.1) and "TAKEOVER OK" or "NO TAKEOVER (check A2 variant)"))
      P.Cameras("spike")
    end)
  else
    pcall(function() fpp:Activate(0, true) end)
    after(0.05, function() pcall(function() c:Deactivate(0, false) end); P.Log("CamSpike OFF") end)
  end
  return true
end

-- ================================================================ aim probe S1 (where do player shots start in 2.31?)
-- Hypothesis W: the shot origin is the ACTIVE camera (utppCamera when it renders), not the FPP camera
-- or the muzzle. Origin sources (script dumps vanilla-2.31):
--   EffectPreAction_PreAttack.Process(ctx)  - pre-action of attack game effects; reads EffectSharedData.position into
--     AttackData.attackPosition [cyberpunk/damage/preAttack.script:12-33]; shared data also has forward / muzzlePosition
--     [core/blackboard/blackboardDefinitions.script:579-591]
--   BaseProjectile.OnShoot / BaseBullet.OnShoot(gameprojectileShootEvent) - startPoint, startVelocity, localToWorld
--     [projectiles/baseProjectile.script:41, baseBullet.script:47 (no super call); orphans.script:27460-27471]
--   DamageSystem.ProcessPipeline(gameHitEvent) - attackData.GetAttackPosition() as a fallback origin, hitPosition as hit
--     [damage/damageSystem.script:18; attackData.script:208]; ProcessMissPipeline(gameMissEvent) [:30; orphans:48042]
--   ShootEvents.OnEnter - player weapon PSM shot state (shot moment, no origin) [player/psm/weaponTransitions.script:1895]
--   BaseBullet.OnCollision(gameprojectileHitEvent) - projectile hit position [baseBullet.script:66]
-- Every hook is registered once in onInit (pcall) and does nothing while the probe is off. Every read is pcall-guarded;
-- a failed value prints "(nan)" (vectors / ints) or "nan" (scalars).
local AP = { on = false, acc = 0.0, period = 0.25, window = 0.15, hitWindow = 0.6, flushAfter = 0.4, samples = 0, shots = 0,
             recs = {}, hooks = {}, hookErr = {}, st = nil }
P.ap = AP
local NAN = "(nan)"

local function grab(fn)
  local ok, a, b = pcall(fn)
  if ok then return a, b end
  return nil, nil
end
local function v3(v)
  if v == nil or type(v) == "boolean" or type(v) == "number" or type(v) == "string" then return nil end
  local ok, x, y, z = pcall(function() return v.x, v.y, v.z end)
  if ok and type(x) == "number" and type(y) == "number" and type(z) == "number" then return { x = x, y = y, z = z } end
  return nil
end
local function vecsOf(...)
  local out = {}
  for i = 1, select("#", ...) do
    local v = v3((select(i, ...)))
    if v then out[#out + 1] = v end
  end
  return out
end
local function vecsOfCall(ok, ...)
  if not ok then return {} end
  return vecsOf(...)
end
local function isZero(v) return v == nil or (math.abs(v.x) + math.abs(v.y) + math.abs(v.z)) < 1e-6 end
local function nz(v) if v ~= nil and not isZero(v) then return v end return nil end
local function vlen(v) return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) end
local function vdist(a, b)
  if a == nil or b == nil then return nil end
  return vlen({ x = a.x - b.x, y = a.y - b.y, z = a.z - b.z })
end
local function vnorm(v)
  if v == nil then return nil end
  local l = vlen(v)
  if l < 1e-6 or l ~= l then return nil end
  return { x = v.x / l, y = v.y / l, z = v.z / l }
end
local function vangle(a, b)
  local na, nb = vnorm(a), vnorm(b)
  if na == nil or nb == nil then return nil end
  local d = math.max(-1, math.min(1, na.x * nb.x + na.y * nb.y + na.z * nb.z))
  return math.deg(math.acos(d))
end
local function fmtV(v)
  if v == nil or v.x ~= v.x or v.y ~= v.y or v.z ~= v.z then return NAN end
  return string.format("(%.3f,%.3f,%.3f)", v.x, v.y, v.z)
end
local function fmtN(n, f)
  if n == nil or n ~= n then return "nan" end
  return string.format(f or "%.3f", n)
end
local function idHash(obj)
  local h = grab(function() return obj:GetEntityID().hash end)
  if h == nil then return nil end
  return tostring(h)
end
local function sameEntity(a, b)
  if a == nil or b == nil then return false end
  local ha, hb = idHash(a), idHash(b)
  if ha ~= nil and hb ~= nil then return ha == hb end
  return rawequal(a, b)
end
local function isPlayerObj(obj, player)
  if obj == nil then return false end
  if player ~= nil and sameEntity(obj, player) then return true end
  local ok, r = pcall(function() return obj:IsPlayer() end)
  return ok and r == true
end

-- active camera: CET out-parameter form first (extra return values), then the argument form (non-zero only)
local function apActiveCam()
  local cs = grab(function() return Game.GetCameraSystem() end)
  if cs == nil then return nil, nil end
  local pos = nil
  local ok, a, b = pcall(function() return cs:GetActiveCameraWorldTransform() end)
  if ok then
    for _, r in ipairs({ b or false, a or false }) do
      if pos == nil and r and r ~= true then pos = nz(v3(grab(function() return r.position end))) end
    end
  end
  if pos == nil then
    pos = nz(v3(grab(function()
      local xf = Transform.new()
      cs:GetActiveCameraWorldTransform(xf)
      return xf.position
    end)))
  end
  return pos, nz(v3(grab(function() return cs:GetActiveCameraForward() end)))
end

local function apFpp(player)
  local c = grab(function() return player:GetFPPCameraComponent() end)
  if c == nil then c = grab(function() return player:FindComponentByName("camera") end) end
  if c == nil then return nil, nil end
  local m = grab(function() return c:GetLocalToWorld() end)
  if m == nil then return nil, nil end
  local pos = nz(v3(grab(function() return m.W end)))
  local fwd = nz(v3(grab(function() return Matrix.GetDirectionVector(m) end))) or nz(v3(grab(function() return m.Y end)))
  return pos, fwd
end

-- TargetingSystem.GetDefaultCrosshairData(instigator, out position, out forward) -> Void [orphans.script:22393]
local function apCrosshair(player)
  local ts = grab(function() return Game.GetTargetingSystem() end)
  if ts == nil then return nil, nil end
  local r = vecsOfCall(pcall(function() return ts:GetDefaultCrosshairData(player) end))
  if #r >= 2 and not isZero(r[1]) then return r[1], nz(r[2]) end
  local p, f = grab(function()
    local pp, ff = Vector4.new(0, 0, 0, 0), Vector4.new(0, 0, 0, 0)
    ts:GetDefaultCrosshairData(player, pp, ff)
    return pp, ff
  end)
  return nz(v3(p)), nz(v3(f))
end

local function apEquipped(player)
  return grab(function()
    return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
  end)
end

-- WeaponObject.GetMuzzleSlotWorldTransform() -> Transform [cyberpunk/items/weapon.script:135]
local function apWeapon(player)
  local okI, item = pcall(apEquipped, player)
  if not okI then return nil, nil, NAN end
  if item == nil then return nil, nil, "none" end
  local rec = grab(function() return TDBID.ToStringDEBUG(ItemID.GetTDBID(item:GetItemID())) end)
  local muz = nz(v3(grab(function() return item:GetMuzzleSlotWorldTransform().position end)))
  return item, muz, (rec ~= nil and tostring(rec) ~= "") and tostring(rec) or NAN
end

local function apUpper(player)
  local u = grab(function()
    local defs = GetAllBlackboardDefs().PlayerStateMachine
    return Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs):GetInt(defs.UpperBody)
  end)
  if type(u) == "number" then return string.format("%d", u) end
  return NAN
end

-- ---------------------------------------------------------------- 2026-10-30 step 0: barrel axis (aim pitch plan 1)
-- Is WeaponRight +Y the real barrel? Per sample (AimProbe on, 4 / s): the muzzle slot ORIENTATION
-- (GetMuzzleSlotWorldTransform().orientation, Transform: lowercase fields) expressed in the WeaponRight frame (the player
-- SlotComponent slot bound to the WeaponRight bone, SlotComponent.GetSlotTransform(slotName, out WorldTransform):
-- .Position / .Orientation, both CET call forms as utpp_reload_probe; fallback the weapon entity root), the muzzle offset
-- in that frame, and the pitches of the muzzle forward (+Y), the frame +Y, the camera and muzzle -> P (P = the crosshair
-- point of utpp_camera AimPoint(); fallback 60 m along the crosshair ray). The shot line adds the angle of each muzzle
-- axis and of the frame +Y to the real shot direction; the summary adds one line per weapon. Plain values only.
local function qOf(q)
  if q == nil then return nil end
  local ok, i, j, k, r = pcall(function() return q.i, q.j, q.k, q.r end)
  if ok and type(i) == "number" and type(j) == "number" and type(k) == "number" and type(r) == "number" then
    local l = math.sqrt(i * i + j * j + k * k + r * r)
    if l > 1e-6 and l == l then return { i = i / l, j = j / l, k = k / l, r = r / l } end
  end
  return nil
end
local function qRot(q, v) -- q v q*
  local x, y, z = v.x, v.y, v.z
  local tx = 2 * (q.j * z - q.k * y)
  local ty = 2 * (q.k * x - q.i * z)
  local tz = 2 * (q.i * y - q.j * x)
  return { x = x + q.r * tx + (q.j * tz - q.k * ty), y = y + q.r * ty + (q.k * tx - q.i * tz),
           z = z + q.r * tz + (q.i * ty - q.j * tx) }
end
local function qInvRot(q, v) return qRot({ i = -q.i, j = -q.j, k = -q.k, r = q.r }, v) end
-- pitch / yaw (deg) of a direction, REDengine conventions (Z up, yaw 0 = +Y, positive yaw = left)
local function pitchOf(v)
  local n = vnorm(v)
  if n == nil then return nil end
  return math.deg(math.asin(math.max(-1, math.min(1, n.z))))
end
local function yawOf(v)
  local n = vnorm(v)
  if n == nil or (math.abs(n.x) + math.abs(n.y)) < 1e-6 then return nil end
  return math.deg(math.atan2(-n.x, n.y))
end
local function wrap(a)
  if a == nil then return nil end
  a = (a + 180) % 360 - 180
  return a
end
local function sub(a, b) return { x = a.x - b.x, y = a.y - b.y, z = a.z - b.z } end
P.AXIS = { X = { x = 1, y = 0, z = 0 }, Y = { x = 0, y = 1, z = 0 }, Z = { x = 0, y = 0, z = 1 } }
-- pure (tested offline): muzzle (mPos, mQ), frame (fPos, fQ; nil = unknown), camera forward, crosshair point aimP ->
-- { mY (world muzzle +Y), mYinF (muzzle +Y in the frame), axisOff (deg, muzzle +Y vs frame +Y), ofsF (muzzle offset in the
-- frame), mPitch / fPitch / camPitch / pPitch (deg), errM / errF (muzzle / frame +Y pitch - muzzle -> P pitch), yawM / yawF
-- (the same for yaw, wrapped) }
function P.AimAxisMath(mPos, mQ, fPos, fQ, camFwd, aimP)
  local o = {}
  if mQ then o.mY = qRot(mQ, P.AXIS.Y) end
  if fQ then o.fY = qRot(fQ, P.AXIS.Y) end
  if mQ and fQ then
    o.mYinF = qInvRot(fQ, o.mY)
    o.axisOff = vangle(o.mY, o.fY)
  end
  if mPos and fPos and fQ then o.ofsF = qInvRot(fQ, sub(mPos, fPos)) end
  o.mPitch, o.fPitch, o.camPitch = o.mY and pitchOf(o.mY), o.fY and pitchOf(o.fY), camFwd and pitchOf(camFwd)
  local toP = (mPos and aimP) and sub(aimP, mPos) or nil
  if toP and vlen(toP) > 1e-3 then
    o.pPitch, o.pYaw = pitchOf(toP), yawOf(toP)
    if o.mPitch then o.errM = o.mPitch - o.pPitch end
    if o.fPitch then o.errF = o.fPitch - o.pPitch end
    if o.mY and o.pYaw and yawOf(o.mY) then o.yawM = wrap(yawOf(o.mY) - o.pYaw) end
    if o.fY and o.pYaw and yawOf(o.fY) then o.yawF = wrap(yawOf(o.fY) - o.pYaw) end
  end
  return o
end
-- muzzle slot world transform: position + orientation (Transform: lowercase fields)
local function apMuzzleXf(item)
  if item == nil then return nil, nil end
  local pos, q = grab(function()
    local t = item:GetMuzzleSlotWorldTransform()
    return t.position, t.orientation
  end)
  return nz(v3(pos)), qOf(q)
end
-- WeaponRight frame: player SlotComponent slot bound to the WeaponRight bone (prefer slotName WeaponRight), its world
-- transform and, if the slot struct exposes them, its relative position / rotation (a slot offset from the bone would
-- contaminate the axis offset: logged). Fallback: the weapon entity root (src "entity").
local function apWeaponFrame(player, item)
  local best = nil
  pcall(function()
    for _, c in ipairs(player:GetComponents() or {}) do
      pcall(function()
        local slots = c.slots
        if slots == nil then return end
        for _, s in ipairs(slots) do
          pcall(function()
            local sn, bn = s.slotName, s.boneName
            sn = type(sn) == "string" and sn or Game.NameToString(sn)
            bn = type(bn) == "string" and bn or Game.NameToString(bn)
            if bn == "WeaponRight" and (best == nil or sn == "WeaponRight") then
              best = { comp = c, slot = s.slotName, slotName = sn, bone = bn, relPos = nz(v3(s.relativePosition)),
                       relQ = qOf(s.relativeRotation) }
            end
          end)
        end
      end)
    end
  end)
  if best then
    local pos, q = grab(function()
      local xf = WorldTransform.new()
      local r1, r2 = best.comp:GetSlotTransform(best.slot, xf)
      if r1 == false then error("slot not found") end
      local t = r2 ~= nil and r2 ~= true and r2 or xf
      return WorldPosition.ToVector4(t.Position), t.Orientation
    end)
    local fq = qOf(q)
    local fp = nz(v3(pos))
    local out = { src = "slot", slotName = best.slotName, bone = best.bone, relPos = best.relPos, relQ = best.relQ }
    best = nil
    if fq and fp then out.pos, out.q = fp, fq; return out end
  end
  if item == nil then return nil end
  local fq = qOf(grab(function() return item:GetWorldOrientation() end))
  local fp = nz(v3(grab(function() return item:GetWorldPosition() end)))
  if fq and fp then return { src = "entity", pos = fp, q = fq } end
  return nil
end
-- crosshair point P: utpp_camera AimPoint() (the point the native targets aim at), else 60 m along the crosshair ray
local function apAimPoint(s)
  local cam = grab(function() return GetMod("utpp_camera") end)
  if type(cam) == "table" and type(cam.AimPoint) == "function" then
    local p, d = grab(function() return cam.AimPoint() end)
    if p ~= nil then return v3(p), "camera", d end
  end
  local o, f = s.xhair or s.cam, vnorm(s.xfwd or s.fwd)
  if o and f then return { x = o.x + f.x * 60, y = o.y + f.y * 60, z = o.z + f.z * 60 }, "far60", 60 end
  return nil, "none", nil
end

local function apSnapshot()
  local s = { t = P.t, item = NAN, upper = NAN }
  local player = grab(function() return Game.GetPlayer() end)
  if player == nil then return s end
  s.cam, s.fwd = grab(function() return apActiveCam() end)
  s.fpp, s.fppfwd = grab(function() return apFpp(player) end)
  s.xhair, s.xfwd = grab(function() return apCrosshair(player) end)
  local okW, item, muz, rec = pcall(apWeapon, player)
  if okW then s.weapon, s.muzzle, s.item = item, muz, rec end
  local okU, up = pcall(apUpper, player)
  if okU then s.upper = up end
  if AP.on and s.weapon ~= nil then -- 2026-10-30 step 0 (only while the probe samples)
    local okM, _, mq = pcall(apMuzzleXf, s.weapon)
    if okM then s.mq = mq end
    local okF, fr = pcall(apWeaponFrame, player, s.weapon)
    if okF then s.frame = fr end
    local okP, ap, src, d = pcall(apAimPoint, s)
    if okP then s.aimP, s.aimPSrc, s.aimPD = ap, src, d end
  end
  s.weapon = nil -- no handle outlives the sample (the shot records keep the snapshot)
  return s
end
local function fmtQ(q)
  if q == nil then return NAN end
  return string.format("(%.4f,%.4f,%.4f,%.4f)", q.i, q.j, q.k, q.r)
end
-- one "aimaxis" line per sample with a weapon (AimProbe on); accumulates the per-weapon summary
local function apAxisLine(s)
  if s.mq == nil and s.frame == nil then return end
  local fr = s.frame or {}
  local m = P.AimAxisMath(s.muzzle, s.mq, fr.pos, fr.q, s.fwd, s.aimP)
  P.Log(string.format("aimaxis t=%.2f item=%s frame=%s slot=%s bone=%s slotRel=%s/%s mq=%s fq=%s mYinF=%s axisOff=%s "
    .. "ofsF=%s camPitch=%s mPitch=%s fPitch=%s pPitch=%s errM=%s errF=%s yawM=%s yawF=%s p=%s d=%s",
    s.t or P.t, tostring(s.item or NAN), tostring(fr.src or "none"), tostring(fr.slotName or "-"), tostring(fr.bone or "-"),
    fmtV(fr.relPos), fmtQ(fr.relQ), fmtQ(s.mq), fmtQ(fr.q), fmtV(m.mYinF), fmtN(m.axisOff, "%.2f"), fmtV(m.ofsF),
    fmtN(m.camPitch, "%.2f"), fmtN(m.mPitch, "%.2f"), fmtN(m.fPitch, "%.2f"), fmtN(m.pPitch, "%.2f"), fmtN(m.errM, "%.2f"),
    fmtN(m.errF, "%.2f"), fmtN(m.yawM, "%.2f"), fmtN(m.yawF, "%.2f"), tostring(s.aimPSrc or "none"), fmtN(s.aimPD, "%.2f")))
  local key = tostring(s.item or NAN)
  AP.axis = AP.axis or {}
  local a = AP.axis[key] or { n = 0, sum = {}, cnt = {} }
  AP.axis[key] = a
  a.n = a.n + 1
  for _, k in ipairs({ "axisOff", "errM", "errF" }) do
    if m[k] ~= nil and m[k] == m[k] then a.sum[k] = (a.sum[k] or 0) + m[k]; a.cnt[k] = (a.cnt[k] or 0) + 1 end
  end
  if m.ofsF then
    for _, c in ipairs({ "x", "y", "z" }) do a.sum["ofs" .. c] = (a.sum["ofs" .. c] or 0) + m.ofsF[c] end
    a.cnt.ofs = (a.cnt.ofs or 0) + 1
  end
end

local function apSampleLine(s)
  return string.format("aimprobe sample t=%.2f cam=%s fwd=%s fpp=%s fppfwd=%s xhair=%s xfwd=%s muzzle=%s upper=%s item=%s",
    s.t or P.t, fmtV(s.cam), fmtV(s.fwd), fmtV(s.fpp), fmtV(s.fppfwd), fmtV(s.xhair), fmtV(s.xfwd), fmtV(s.muzzle),
    tostring(s.upper or NAN), tostring(s.item or NAN))
end

function P.AimProbeSample()
  local ok, s = pcall(apSnapshot)
  if not ok then s = { t = P.t } end
  AP.last = s
  if AP.on then AP.samples = AP.samples + 1 end
  P.Log(apSampleLine(s))
  if AP.on then
    local okX, errX = pcall(apAxisLine, s)
    if not okX then P.Log("aimaxis FAILED: " .. tostring(errX)) end
  end
  return s
end

-- ---------------------------------------------------------------- shot records (one per player shot)
local function apResetStats()
  AP.samples, AP.shots, AP.recs, AP.axis = 0, 0, {}, {}
  AP.st = { votes = { cam = 0, fpp = 0, muzzle = 0 }, sum = { dCam = 0, dFpp = 0, dMuzzle = 0, angCam = 0, angFpp = 0 },
            n = { dCam = 0, dFpp = 0, dMuzzle = 0, angCam = 0, angFpp = 0 } }
end
apResetStats()

local function apAdd(key, v)
  if v ~= nil and v == v then AP.st.sum[key] = AP.st.sum[key] + v; AP.st.n[key] = AP.st.n[key] + 1 end
end

local function apNewRec()
  local ok, s = pcall(apSnapshot)
  local r = { t = P.t, snap = ok and s or { t = P.t } }
  AP.recs[#AP.recs + 1] = r
  return r
end
-- newest open record of a kind: "origin" (no origin yet), "shoot" (no ShootEvents yet), "hit" (no hit yet)
local function apFind(kind)
  for i = #AP.recs, 1, -1 do
    local r = AP.recs[i]
    local age = P.t - r.t
    if kind == "origin" and r.origin == nil and age <= AP.window then return r end
    if kind == "shoot" and not r.shootEvt and age <= AP.window then return r end
    if kind == "hit" and r.hit == nil and age <= AP.hitWindow then return r end
  end
  return nil
end
local function apDuplicate(origin)
  for i = #AP.recs, 1, -1 do
    local r = AP.recs[i]
    if P.t - r.t <= AP.window and r.origin ~= nil and (vdist(r.origin, origin) or 1) < 1e-3 then return true end
  end
  return false
end
local function apOrigin(src, origin, dir)
  if origin == nil or apDuplicate(origin) then return end
  local r = apFind("origin") or apNewRec()
  r.origin, r.dir, r.src = origin, dir, src
end
local function apHit(hit, player, weapon, attackPos, hitDir, src)
  local r = apFind("hit")
  if r == nil then
    -- no shot record open: only a hit by the equipped weapon starts one (origin = AttackData.attackPosition)
    local eq = apEquipped(player)
    if weapon == nil or eq == nil or not sameEntity(weapon, eq) then return end
    r = apNewRec()
  end
  r.hit = hit
  if r.origin == nil and attackPos ~= nil then
    r.origin, r.src = attackPos, src
    r.dir = vnorm(hitDir) or (hit and vnorm({ x = hit.x - attackPos.x, y = hit.y - attackPos.y, z = hit.z - attackPos.z }))
  end
end

local function apFinalize(r)
  AP.shots = AP.shots + 1
  local s = r.snap or {}
  local o, d = r.origin, vnorm(r.dir)
  local dCam, dFpp, dMuz = vdist(o, s.cam), vdist(o, s.fpp), vdist(o, s.muzzle)
  local aCam, aFpp = vangle(d, s.fwd), vangle(d, s.fppfwd)
  apAdd("dCam", dCam); apAdd("dFpp", dFpp); apAdd("dMuzzle", dMuz); apAdd("angCam", aCam); apAdd("angFpp", aFpp)
  local best, bestD = nil, nil
  for _, c in ipairs({ { "cam", dCam }, { "fpp", dFpp }, { "muzzle", dMuz } }) do
    if c[2] ~= nil and (bestD == nil or c[2] < bestD) then best, bestD = c[1], c[2] end
  end
  if best then AP.st.votes[best] = AP.st.votes[best] + 1 end
  -- 2026-10-30 step 0 (appended): angle of the muzzle slot X / Y / Z axes and of the WeaponRight frame +Y to the shot
  -- direction (deg; 180 = the opposite axis), the shot pitch and the muzzle +Y pitch at the shot record
  local angM, angF, mYP = { x = nil, y = nil, z = nil }, nil, nil
  if d and s.mq then
    for _, c in ipairs({ "X", "Y", "Z" }) do angM[c:lower()] = vangle(qRot(s.mq, P.AXIS[c]), d) end
    mYP = pitchOf(qRot(s.mq, P.AXIS.Y))
  end
  if d and s.frame and s.frame.q then angF = vangle(qRot(s.frame.q, P.AXIS.Y), d) end
  P.Log(string.format("aimprobe shot n=%d origin=%s dir=%s dCam=%s dFpp=%s dMuzzle=%s angCam=%s angFpp=%s hit=%s src=%s",
    AP.shots, fmtV(o), fmtV(d), fmtN(dCam), fmtN(dFpp), fmtN(dMuz), fmtN(aCam, "%.2f"), fmtN(aFpp, "%.2f"),
    r.hit ~= nil and fmtV(r.hit) or "none", r.src or (r.shootEvt and "ShootEvents.OnEnter" or "unknown"))
    .. string.format(" angMX=%s angMY=%s angMZ=%s angFY=%s shotPitch=%s mPitch=%s item=%s", fmtN(angM.x, "%.2f"),
    fmtN(angM.y, "%.2f"), fmtN(angM.z, "%.2f"), fmtN(angF, "%.2f"), fmtN(d and pitchOf(d), "%.2f"), fmtN(mYP, "%.2f"),
    tostring(s.item or NAN)))
end
local function apFlush(all)
  local keep = {}
  for _, r in ipairs(AP.recs) do
    if all or P.t - r.t >= AP.flushAfter then
      local ok, err = pcall(apFinalize, r)
      if not ok then P.Log("aimprobe shot log FAILED: " .. tostring(err)) end
    else
      keep[#keep + 1] = r
    end
  end
  AP.recs = keep
end

local function apSummary()
  local st = AP.st
  local function mean(k, f) return st.n[k] > 0 and string.format(f or "%.3f", st.sum[k] / st.n[k]) or "nan" end
  local meanOf = { cam = "dCam", fpp = "dFpp", muzzle = "dMuzzle" }
  local best, bestV, bestM = "unknown", 0, nil
  for _, k in ipairs({ "cam", "fpp", "muzzle" }) do
    local v = st.votes[k]
    local m = st.n[meanOf[k]] > 0 and st.sum[meanOf[k]] / st.n[meanOf[k]] or nil
    if v > bestV or (v == bestV and v > 0 and m ~= nil and (bestM == nil or m < bestM)) then best, bestV, bestM = k, v, m end
  end
  P.Log(string.format("aimprobe summary samples=%d shots=%d originNearest=%s meanDCam=%s meanDFpp=%s meanDMuzzle=%s meanAngCam=%s meanAngFpp=%s",
    AP.samples, AP.shots, best, mean("dCam"), mean("dFpp"), mean("dMuzzle"), mean("angCam", "%.2f"), mean("angFpp", "%.2f")))
  -- 2026-10-30 step 0: one line per weapon (decision rule: axisOff <= 1 deg -> WeaponRight +Y is the barrel)
  local keys = {}
  for k in pairs(AP.axis or {}) do keys[#keys + 1] = k end
  table.sort(keys)
  for _, k in ipairs(keys) do
    local a = AP.axis[k]
    local function am(key, f) return a.cnt[key] and a.cnt[key] > 0 and string.format(f or "%.2f", a.sum[key] / a.cnt[key]) or "nan" end
    local ofs = NAN
    if a.cnt.ofs and a.cnt.ofs > 0 then
      ofs = fmtV({ x = a.sum.ofsx / a.cnt.ofs, y = a.sum.ofsy / a.cnt.ofs, z = a.sum.ofsz / a.cnt.ofs })
    end
    P.Log(string.format("aimaxis summary item=%s samples=%d meanAxisOff=%s meanOfsF=%s meanErrM=%s meanErrF=%s", k, a.n,
      am("axisOff"), ofs, am("errM"), am("errF")))
  end
end

function P.AimProbe(on)
  if on == nil then on = true end
  if on then
    apResetStats()
    AP.on, AP.acc = true, AP.period
    P.Log(string.format("aimprobe start period=%.2f hooks=%s", AP.period, table.concat(AP.hooks, ",")))
  else
    apFlush(true)
    AP.on = false
    apSummary()
  end
  return AP.on
end

-- ---------------------------------------------------------------- hooks (registered in onInit, inert while off)
local function effVec(shared, id)
  local r = vecsOfCall(pcall(function() return EffectData.GetVector(shared, id) end))
  if r[1] ~= nil and not isZero(r[1]) then return r[1] end
  return nz(v3(grab(function()
    local o = Vector4.new(0, 0, 0, 0)
    EffectData.GetVector(shared, id, o)
    return o
  end)))
end
local function effHasVariant(shared, id)
  local ok, a = pcall(function() return EffectData.GetVariant(shared, id) end)
  return ok and a == true
end

local function onPreAttack(_, ctx)
  local player = Game.GetPlayer()
  if not isPlayerObj(grab(function() return EffectScriptContext.GetInstigator(ctx) end), player) then return end
  local w = grab(function() return EffectScriptContext.GetWeapon(ctx) end)
  local eq = apEquipped(player)
  if w == nil or eq == nil or not sameEntity(w, eq) then return end
  local shared = EffectScriptContext.GetSharedData(ctx)
  local defs = GetAllBlackboardDefs().EffectSharedData
  -- projectile impact effects carry the hit event; their position is the impact, not the shot origin
  if effHasVariant(shared, defs.projectileHitEvent) then return end
  apOrigin("EffectPreAction_PreAttack.Process", effVec(shared, defs.position), effVec(shared, defs.forward))
end

local function projectileShot(src)
  return function(_, ev)
    if not isPlayerObj(grab(function() return ev.owner end), Game.GetPlayer()) then return end
    local m = grab(function() return ev.localToWorld end)
    local origin = nz(v3(grab(function() return ev.startPoint end))) or (m and nz(v3(grab(function() return m.W end))))
    local dir = vnorm(nz(v3(grab(function() return ev.startVelocity end))))
      or (m and nz(v3(grab(function() return Matrix.GetDirectionVector(m) end))))
    apOrigin(src, origin, dir)
  end
end

local function onShootState()
  local r = apFind("shoot") or apNewRec()
  r.shootEvt = true
end

local function damageEvent(src)
  return function(_, evt)
    local player = Game.GetPlayer()
    local ad = grab(function() return evt.attackData end)
    if ad == nil or not isPlayerObj(grab(function() return ad:GetInstigator() end), player) then return end
    local weapon = grab(function() return ad:GetWeapon() end)
    local eq = apEquipped(player)
    if weapon ~= nil and eq ~= nil and not sameEntity(weapon, eq) then return end
    apHit(nz(v3(grab(function() return evt.hitPosition end))), player, weapon, nz(v3(grab(function() return ad:GetAttackPosition() end))),
      nz(v3(grab(function() return evt.hitDirection end))), src)
  end
end

local function onBulletCollision(self, ev)
  local player = Game.GetPlayer()
  if not isPlayerObj(grab(function() return self.m_user end), player) then return end
  local hit = nz(v3(grab(function() return ev.hitInstances[1].position end)))
  if hit == nil then return end
  local r = apFind("hit")
  if r ~= nil then r.hit = hit end
end

local function apRegister(cls, method, fn)
  local name = cls .. "." .. method
  local wrapped = function(...)
    if not AP.on then return end
    local ok, err = pcall(fn, ...)
    if not ok and not AP.hookErr[name] then
      AP.hookErr[name] = true
      P.Log("aimprobe hook " .. name .. " FAILED (logged once): " .. tostring(err))
    end
  end
  local ok, err = pcall(function() Observe(cls, method, wrapped) end)
  AP.hooks[#AP.hooks + 1] = ok and name or (name .. "!FAILED")
  if not ok then P.Log("aimprobe Observe " .. name .. " FAILED: " .. tostring(err)) end
end

local function apRegisterAll()
  apRegister("EffectPreAction_PreAttack", "Process", onPreAttack)
  apRegister("BaseProjectile", "OnShoot", projectileShot("BaseProjectile.OnShoot"))
  apRegister("BaseBullet", "OnShoot", projectileShot("BaseBullet.OnShoot"))
  apRegister("ShootEvents", "OnEnter", onShootState)
  apRegister("DamageSystem", "ProcessPipeline", damageEvent("DamageSystem.ProcessPipeline"))
  apRegister("DamageSystem", "ProcessMissPipeline", damageEvent("DamageSystem.ProcessMissPipeline"))
  apRegister("BaseBullet", "OnCollision", onBulletCollision)
end

local function apUpdate(dt)
  if not AP.on then return end
  AP.acc = AP.acc + (dt or 0)
  if AP.acc >= AP.period then
    AP.acc = AP.acc - AP.period
    if AP.acc >= AP.period then AP.acc = 0 end
    if Game.GetPlayer() ~= nil then P.AimProbeSample() end
  end
  if #AP.recs > 0 then apFlush(false) end
end

-- ================================================================ old continuous mode (Phase 2 spike, opt-in)
local function status(tag)
  local fps = P.frameTime > 0 and P.frames / P.frameTime or 0
  local ok, s = pcall(function() return Game.UTPPGetStatus() end)
  fileLog(string.format("%s fps=%.1f frames=%d %s", tag, fps, P.frames, ok and tostring(s) or ("ERROR " .. tostring(s))))
  P.frames, P.frameTime = 0, 0.0
end
function P.Continuous(on)
  P.continuous = on and true or false
  P.configured = false
  if not on then pcall(function() Game.UTPPSetEnabled(false) end) end
  P.Log("continuous mode " .. (P.continuous and "ON (15 deg Spine3 test rotation)" or "OFF"))
  return P.continuous
end

-- ================================================================ dual-wield D0 probe
-- GetMod("utpp_probe").DualProbe()  one line "[utpp_dual] probe ..." (UTPP log + probe_dump.log + console): PSM ShootState
-- (gamePSMShootStates: int + name + the enum table 0..7), UpperBody (int + name), IsDualWielding, RightWeapon / LeftWeapon,
-- the items in AttachmentSlots.WeaponRight / WeaponLeft (record|type|entity) and the dual-wield toggle state. The 2.5 bytecode
-- names show no readable toggle API (ToggleDualWieldWeapon / DelayedResetDualWieldItemToggled belong to the inventory UI), so
-- toggle=n/a unless one of the guessed global forms answers; every read is pcall-guarded and reports "?" when it fails.
-- GetMod("utpp_probe").DualProbeEvery(seconds | nil)  repeat it from onUpdate (nil / 0 = off), e.g. 0.25 while firing.
local DP = { every = 0, acc = 0.0 }

-- own prefix ("[utpp_dual] probe ..."), not P.Log's "[utpp_probe] "
local function dpLog(line)
  fileLog(line)
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(line) end) end
end

local function dpVal(v)
  if v == nil then return "?" end
  local s = tostring(v)
  if s == "" then return "?" end
  return (s:gsub("%s+", "_"))
end

local function dpEnum(enumName, v)
  if type(v) ~= "number" then return "?" end
  -- Game.EnumValueToString is the form the other UTPP CET mods use (utpp_vehiclecam, utpp_perspective); the bare global is the fallback
  local ok, name = pcall(function() return Game.EnumValueToString(enumName, v) end)
  if ok and type(name) == "string" and name ~= "" then return name end
  ok, name = pcall(function() return EnumValueToString(enumName, v) end)
  if ok and type(name) == "string" and name ~= "" then return name end
  return "?"
end

local function dpWeapon(player, slot)
  local ok, item = pcall(function()
    return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots." .. slot))
  end)
  if not ok or item == nil then return "-" end
  local rec = grab(function() return TDBID.ToStringDEBUG(ItemID.GetTDBID(item:GetItemID())) end)
  local ty = grab(function() return WeaponObject.GetWeaponType(item:GetItemID()) end)
  if type(ty) ~= "string" and type(ty) ~= "number" then
    local inner = grab(function() return ty.value end)
    if inner ~= nil then ty = inner end
  end
  local eid = grab(function() return EntityID.ToDebugStringDecimal(item:GetEntityID()) end)
  return dpVal(rec) .. "|" .. dpVal(ty) .. "|" .. dpVal(eid)
end

-- first answering form of a guessed global "is dual wielding" API, "?" if none
local function dpApiDual(player)
  local forms = {
    { "global", function() return IsDualWielding(player) end },
    { "game", function() return Game.IsDualWielding(player) end },
    { "method", function() return player:IsDualWielding() end },
  }
  for _, f in ipairs(forms) do
    local ok, v = pcall(f[2])
    if ok and v ~= nil then return tostring(v) .. "@" .. f[1] end
  end
  return "?"
end

function P.DualProbe()
  local parts = {}
  local player = grab(function() return Game.GetPlayer() end)
  if player == nil then
    dpLog("[utpp_dual] probe no player")
    return nil
  end
  local defs = grab(function() return GetAllBlackboardDefs().PlayerStateMachine end)
  local bb = grab(function() return Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs) end)
  local function geti(name)
    local v = grab(function() return bb:GetInt(defs[name]) end)
    return type(v) == "number" and v or nil
  end
  local upper, rw, lw = geti("UpperBody"), geti("RightWeapon"), geti("LeftWeapon")
  local flag = grab(function() return bb:GetBool(defs.IsDualWielding) end)
  -- PSM field of the shoot arbiter: try the bytecode name first, then the enum-like variants (reports the one that answered)
  local shoot, shootField = nil, "?"
  for _, name in ipairs({ "ShootState", "ShootStates", "Shoot" }) do
    local v = geti(name)
    if v ~= nil then shoot, shootField = v, name; break end
  end
  local tbl = {}
  for i = 0, 7 do
    local n = dpEnum("gamePSMShootStates", i)
    if n ~= "?" then tbl[#tbl + 1] = i .. ":" .. n end
  end
  parts[#parts + 1] = "upper=" .. dpVal(upper) .. "(" .. dpEnum("gamePSMUpperBodyStates", upper) .. ")"
  parts[#parts + 1] = "IsDualWielding=" .. dpVal(flag)
  parts[#parts + 1] = "shootState=" .. dpVal(shoot) .. "(" .. dpEnum("gamePSMShootStates", shoot) .. ")"
  parts[#parts + 1] = "shootField=" .. shootField
  parts[#parts + 1] = "rw=" .. dpVal(rw) .. "(" .. dpEnum("gamePSMRangedWeaponStates", rw) .. ")"
  parts[#parts + 1] = "lw=" .. dpVal(lw) .. "(" .. dpEnum("gamePSMRangedWeaponStates", lw) .. ")"
  parts[#parts + 1] = "R=" .. dpWeapon(player, "WeaponRight")
  parts[#parts + 1] = "L=" .. dpWeapon(player, "WeaponLeft")
  parts[#parts + 1] = "apiDual=" .. dpApiDual(player)
  parts[#parts + 1] = "toggle=n/a"
  parts[#parts + 1] = "shootEnum=" .. (#tbl > 0 and table.concat(tbl, ",") or "?")
  local line = "[utpp_dual] probe " .. table.concat(parts, " ")
  dpLog(line)
  return line
end

function P.DualProbeEvery(seconds)
  DP.every = (type(seconds) == "number" and seconds > 0) and seconds or 0
  DP.acc = 0.0
  dpLog("[utpp_dual] probe repeat " .. (DP.every > 0 and (tostring(DP.every) .. " s") or "off"))
  return DP.every
end

-- Inert until a console / harness call: the observers return at once while AimProbe is off (CET registers them in onInit
-- only), and the load lines go to the UTPP log without creating probe_dump.log in the mod folder.
local function initLog(msg)
  local line = "[utpp_probe] " .. tostring(msg)
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(line) end) end
end
-- ragdoll R0 probe: publishes frame 0 of a motion get-up clip with
-- the hips moved offsetM metres along the character-frame +Y (forward) for durationS seconds (0.5..5, default 3), then hands back to MM.
-- Needs mm.motionDriver on and V standing in TPP (unarmed / holstered). Zero effect unless called. Native log lines: "[utpp_ragdoll] getup-probe ...".
--   GetMod("utpp_probe").RagdollProbe(1.2)        -- offset 1.2 m (0 = control, 1.2, 3), hold 3 s
--   GetMod("utpp_probe").RagdollProbe(3, 4)       -- offset 3 m, hold 4 s
--   GetMod("utpp_probe").RagdollProbeStop()       -- abort at once        GetMod("utpp_probe").RagdollProbeStatus()
--   utpp_ragdoll_probe(1.2)                        -- same as RagdollProbe, global alias where the CET environment allows it
local RP = { on = false, nextAt = 0.0, t = 0.0 }
local function rpLog(msg)
  initLog("[utpp_ragdoll] cet " .. tostring(msg))
end
function P.RagdollProbe(offsetM, durationS)
  offsetM = tonumber(offsetM) or 1.2
  durationS = tonumber(durationS) or 3.0
  local ok, r = pcall(function() return Game.UTPPRagdollGetUpProbe(offsetM, durationS) end)
  r = ok and tostring(r) or ("ERROR " .. tostring(r))
  rpLog(string.format("RagdollProbe(%.2f, %.2f) -> %s", offsetM, durationS, r))
  RP.on = ok and r:sub(1, 7) == "started"
  RP.t, RP.nextAt = 0.0, 0.5
  return r
end
function P.RagdollProbeStop()
  local ok, r = pcall(function() return Game.UTPPRagdollGetUpProbeStop() end)
  RP.on = false
  rpLog("RagdollProbeStop -> " .. (ok and tostring(r) or ("ERROR " .. tostring(r))))
  return ok and r or nil
end
function P.RagdollProbeStatus()
  local ok, r = pcall(function() return Game.UTPPRagdollGetUpProbeStatus() end)
  return ok and r or ("ERROR " .. tostring(r))
end
-- ragdoll R1 (engine ragdoll on the player; needs the asset package with an entRagdollComponent "RagdollComponent"). Reds side:
-- utpp_ragdoll_probe.reds, native log lines "[utpp_ragdoll] eng ...". Zero effect unless called; Force refuses in vehicles / scenes / menus.
--   GetMod("utpp_probe").RagdollEngineStatus()
--   GetMod("utpp_probe").RagdollEngineForce(0, 0)    -- mode 0 force, 1 force without power pose, 2 forced without momentum; impulse 0..500 (30 = vanilla throw)
--   GetMod("utpp_probe").RagdollEngineStop()         -- disable event + UTPP flag restored; the reds watchdog also stops after 8 s
-- Note: UTPPSetEnabled(false) does not stop mm.v1 / mm.motionDriver pose writes. For an engine-only pose switch BOTH off first (v2 off only hands over to v1):
--   GetMod("utpp_motion").SetEnabled(false) and Game.UTPPSetMMEnabled(false); restore: SetEnabled(true) and GetMod("utpp_core").ReapplyMM("test").
function P.RagdollEngineStatus()
  local ok, r = pcall(function() return Game.UTPPRagdollEngineStatus() end)
  r = ok and tostring(r) or ("ERROR " .. tostring(r))
  rpLog("RagdollEngineStatus -> " .. r)
  return r
end
function P.RagdollEngineForce(mode, impulse)
  mode = math.floor(tonumber(mode) or 0)
  impulse = tonumber(impulse) or 0.0
  local ok, r = pcall(function() return Game.UTPPRagdollEngineForce(mode, impulse) end)
  r = ok and tostring(r) or ("ERROR " .. tostring(r))
  rpLog(string.format("RagdollEngineForce(%d, %.1f) -> %s", mode, impulse, r))
  return r
end
function P.RagdollEngineStop()
  local ok, r = pcall(function() return Game.UTPPRagdollEngineStop() end)
  r = ok and tostring(r) or ("ERROR " .. tostring(r))
  rpLog("RagdollEngineStop -> " .. r)
  return r
end
local function rpUpdate(dt)
  RP.t = RP.t + (dt or 0)
  if RP.t < RP.nextAt then return end
  RP.nextAt = RP.t + 0.5
  local s = P.RagdollProbeStatus()
  rpLog("status " .. s)
  if s:sub(1, 4) == "idle" or s:sub(1, 5) == "ERROR" or RP.t > 8.0 then RP.on = false end
end
pcall(function() rawset(_G, "utpp_ragdoll_probe", P.RagdollProbe) end)

registerForEvent("onInit", function()
  initLog("probe loaded (dump mode; GetMod(\"utpp_probe\").Cycle() / .Dump(tag) / .CamSpike(true) / .AimProbe(true))")
  local ok, err = pcall(apRegisterAll)
  initLog("aimprobe hooks " .. (ok and "registered: " .. table.concat(AP.hooks, ",") or ("FAILED: " .. tostring(err))))
end)

registerForEvent("onUpdate", function(dt)
  P.t = P.t + (dt or 0)
  if #P.queue > 0 then
    local keep = {}
    for _, e in ipairs(P.queue) do
      if P.t >= e.at then
        local ok, err = pcall(e.fn)
        if not ok then P.Log("queued step FAILED: " .. tostring(err)) end
      else
        keep[#keep + 1] = e
      end
    end
    P.queue = keep
  end
  if RP.on then pcall(rpUpdate, dt) end
  if DP.every > 0 then
    DP.acc = DP.acc + (dt or 0)
    if DP.acc >= DP.every then
      DP.acc = 0.0
      pcall(P.DualProbe)
    end
  end
  if AP.on then
    local okA, errA = pcall(apUpdate, dt)
    if not okA and not AP.hookErr.update then
      AP.hookErr.update = true
      P.Log("aimprobe update FAILED (logged once): " .. tostring(errA))
    end
  end
  if not P.continuous or Game.GetPlayer() == nil then return end
  P.frames = P.frames + 1
  P.frameTime = P.frameTime + (dt or 0)
  if not P.configured then
    P.configured = true
    pcall(function()
      Game.UTPPSetTestAxis(1)
      Game.UTPPSetTestRotation(15)
      Game.UTPPSetEnabled(true)
    end)
    fileLog("test rotation 15 deg on Spine3 enabled")
  end
  if P.t >= P.nextLog then
    P.nextLog = P.t + 5.0
    status("tick")
  end
end)

return P

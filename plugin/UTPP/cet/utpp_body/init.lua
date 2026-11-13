-- UTPP utpp_body - full-body visibility for the UTPP third-person camera.
--
-- Layers:
--   L1  the game's own TPP representation: ActivateTPPRepresentationEvent / DeactivateTPPRepresentationEvent queued on
--       the player + anim feature TPPRepresentation.IsActive. The game swaps Items.PlayerFppHead <-> Items.Player?aTppHead
--       in AttachmentSlots.TppHead and re-resolves Chest/Torso/Outfit/Head/Eyes/RightArm. Ownership: UTPP remembers
--       whether IT activated the representation; a representation the game activated (vehicle TPP camera, camera
--       takeover, quests, mirrors, scenes) is never deactivated by UTPP. The state is read from the TppHead slot.
--   L2  classifier of player body parts from a table GENERATED from vanilla 2.50 data (body_parts.lua, made by the
--       offline generator gen_bodyparts.py; female and male V): in TPP fppOnly parts are hidden, tppOnly parts the
--       game left disabled are enabled; every change is recorded and exactly that is undone in FPP. Shared / variant /
--       ambiguous / entity / head parts are never touched; unknown parts are left alone and logged once.
--   L3  garments outside the representation slots (default Legs, Feet): item appearance "&FPP" -> "&TPP" (also ArchiveXL
--       dynamic "camera=fpp" -> "camera=tpp"), restored in FPP; optional ArchiveXL garment offsets off in TPP.
--   L4  mesh chunk masks: documented only, not implemented.
-- Standalone: data from vanilla, API from the vanilla scripts + Codeware.
--
-- Public API (camera module):  local body = GetMod("utpp_body")
--   body.SetTppBody(true|false [, reason]) -> accepted, why   body.IsTppBody()   body.GetState()   body.Refresh()
--   body.SetGameOwned(reason, true|false)   (e.g. "mirror": the game owns the representation while active)
--   body.Enabled()   body.Status()   body.Dump()  (logs every player mesh component with its class; for the probe)
-- Settings (utpp_core tab "UTPP" > "Body visibility"): body.impl ("Use UTPP body visibility": on | off, default
-- on), body.hideFpp, body.showTpp, body.garments, body.garmentOffsets, body.headGuard (head guard around
-- vehicles; API SetHeadGuard(on|nil) A/B not saved, HeadGuardStatus();
-- HeadProbe() / HeadRepair("event"|"item") = one-line console checks of the repair path, see GameApi;
-- representation events through the redscript bridge scripts\utpp_body.reds (M.QueueRep), head item spawn waited
-- for instead of being removed and added again every 0.15 s;
-- default environment of QueueRep / RedsSlot / CreateNative works in the CET sandbox where _G is nil, see M.DefaultEnv;
-- Game / NewObject / GetPlayer / GetSingleton resolved live by name at call time (CET sets them after load);
-- FPP entity meshes the TPP body
-- duplicates are hidden in TPP and restored in FPP - class shadowProxy (t0_000_p?a_base__full_shadow, head_shadowmesh:
-- the FPP shadow-only proxy -> V had two shadows) and renderPlaneFpp (entity meshes with renderingPlaneAnimationParam
-- "renderPlane": weapon grip cyberware device / decals, FPP fists, FPP lights -> drawn on the weapon plane through the
-- TPP body); setting body.shadowProxy; one log line per corrected / restored component. The head guard
-- windows (mount / exit / unmount) are opened by utpp_body itself from the PSM vehicle state + GetMountedVehicle
-- (M.NewVehicleWindows), and the unmount re-checks the TPP representation at once (retry budget reset)).
-- After the save load the body was
-- requested at the unfreeze (0 s after the load), three retries 1 s apart stayed on the FPP head and utpp_body gave up for
-- good; the camera's SetTppBody(true) 1.5 s later was a no-op (already wanted -> no re-arm). At every aim enter / exit the
-- game then re-resolved the FPP appearance on the shown body (male V: shirt + right arm gone, pistol floating). Setting
-- body.retry (default on; off = no retry): (1) a repeated request re-arms the retry budget; (2) "gave up" is never
-- final - a new round every retryEvery s, the strategies cycle event / event / toggle (deactivate + activate) / head item
-- + event; (3) Nudge(why) from the camera on aim / zoom enter and exit (and any caller): parts verified at once and every
-- fastVerifyInterval s for nudgeWindow s, a failed request re-armed; (4) the first request after a player entity change /
-- unfreeze waits settle s (the event sent at the unfreeze was ignored by the loading game); (5) every activate line
-- carries the strategy and the TppHead slot state ("via event slot=fpp"). Stats: rounds / nudges / toggles / items.
-- (the head vanished or floated ~0.47 s into every bike exit; the
-- strategy counter kept climbing over the session, so 2 of 3 exits were repaired by toggle / item):
--   (1) the attempt counter is reset whenever the representation reads 'tpp' (every new loss starts with 'event';
--       the only change on foot); (2) inside a vehicle window (mount / exit / unmount) the body uses the plain event
--       only, never toggle / item; (3) head guard, window 'exit': repair on the first poll that reads fpp / empty
--       (delay 0) and poll every 0.016 s for the first 1.0 s of the window; (4) a TPP head that re-spawns inside a
--       window is kept hidden (its mesh components) until the slot reads 'tpp' twice in a row plus one frame, so a
--       head at a stale transform is never drawn (bounded by hideMax); (5) optional vehicle camera hook: when
--       utpp_vehiclecam exposes the callback field onVehiclePerspective(prev, new) (game-owned ride camera), the TPP
--       head is pre-equipped on FPP -> TPP and the TPP head item is hidden at the end of the 0.45 s blend on
--       TPP -> FPP until the game swaps the head; detected at run time, absent hook = no change (M.OnVehiclePerspective
--       is the same handler for a direct call).
-- (6-frame flash 0.47 s into every bike exit - head gone, the
--   vehicle graph on its first-person branch): exit keep of the TPP representation (M.NewRepKeep): in the 'exit' window,
--   while UTPP wants the TPP body, the anim feature TPPRepresentation.IsActive is applied every frame and a redscript
--   latch answers a deactivation that reaches the player at once (scripts\utpp_body.reds); the bridge counters are
--   logged ("representation keep diag"). api.feature goes through the bridge first. Switch: M.SetRepKeep(on).
--   Native guard (UTPP.dll, world\TppRepGuard.cpp): the same window opens / closes Game.UTPPSetVehicleExitRepGuard; while
--   open (at most 3 s natively) the vehicle camera does not send its end-of-switch deactivation. Absent native = no change.
-- (one render-plane policy for V in TPP, scripts\utpp_itemplane.reds): diagnostics only. Dump lines and
--   the one-time unknown-part lines also print renderingPlaneAnimationParam (the FPP animation parameter a skinned mesh's
--   drawn plane follows; reflected field, unavailable:* when absent) and itemPlane = the matching line of the native
--   component registry (Game.UTPPItemPlaneDump: requested / current plane), "not-registered" (never passed the hooked
--   broadcast) or unavailable:* (old DLL / scripts; guarded, never an error). The registry is read at most once per Dump()
--   and once per verify scan (lazily, only when an unknown part is logged). Console helper, no setting:
--   print(GetMod("utpp_body").PlaneDump([max]))  -> the native dump text (each line also printed to the CET console).
--   (read-only): PlaneDump then logs one "W slot= item= app= fppVariant=" line per worn
--   item of every occupied AttachmentSlots.* / OutfitSlots.* slot (fppVariant 1 = an FPP appearance: _fpp, &FPP, camera=fpp),
--   and unknown-part / Dump lines without a scan path probe the mesh depot path lazily (path=<depot path> | unresolved |
--   unavailable:*).

-- (on-foot idle fix): anim wrapper variable utpp_foot_tpp = 1 while the UTPP third-person body is wanted
--   (api.footTpp -> Game.UTPPSetFootTpp, scripts\utpp_body.reds), 0 otherwise and during a vanilla take-over of control
--   (security camera). Cause: the 2.5 player entities register tpp_p?a_security_cam.anims (priority 199, vars
--   [player_tpp_head]) ahead of player_locomotion.anims (200); the set holds fpp_idle_stand and fpp_crouch_idle as copies of the
--   hacking crouch, so every visible game-graph idle with the TPP representation on showed it. archive\utpp_foot_tpp.archive.xl
--   registers player_locomotion.anims again at priority 198 gated by this variable; player_locomotion.animgraph
--   (in the UTPP archive) lets the ANY -> tpp_securitycam_hacking edge fire only while the variable is not set.
--   Sent from Body:Step before any activation branch (the graph edge needs the variable in place first), on change, again after a
--   player change / unfreeze and every opt.footReassert s (a reloaded player starts with every wrapper at 0). It stays 1 through
--   vehicle rides and scenes (the set is the same locomotion set, only its rank changes), 0 for FPP, a disabled setting and
--   game-owned take-over. The legacy utpp_camera fallback (representation() without utpp_body) does not set it.
local M = { VERSION = "0.1.11" }

-- ================================================================ helpers
local function lower(s) return s and string.lower(s) or nil end
-- forward declaration - Body:Activate (above the redscript bridge, where it is defined) called a nil global
-- 'short' when a toggle deactivate or a head item failed
local short

-- uint64 hash from CET (cdata "123ULL", number, string) -> decimal string; nil when unknown or precision was lost
function M.NormHash(v)
  local t = type(v)
  if t == "string" then
    local d = string.match(v, "^%s*(%d+)")
    if d == nil or d == "0" then return nil end
    return d
  elseif t == "number" then
    if v > 0 and v < 9007199254740992 and v == math.floor(v) then return string.format("%.0f", v) end
    return nil
  elseif t == "cdata" or t == "userdata" then
    return M.NormHash(tostring(v))
  end
  return nil
end

-- ArchiveXL-compatible appearance name conversion (own implementation of the documented name format):
-- static names carry "&FPP" / "&TPP"; dynamic names are "base!attr=v+attr2=motion%hash" with attribute camera=fpp|tpp.
local function setDynamicAttr(app, attr, value)
  local base, dyn = string.match(app, "^(.-)!(.*)$")
  local variant, hash = string.match(dyn, "^(.-)%%(.*)$")
  if variant == nil then variant, hash = dyn, nil end
  local parts, found = {}, false
  for p in string.gmatch(variant, "[^+]+") do
    if string.sub(p, 1, #attr + 1) == attr .. "=" then p = attr .. "=" .. value; found = true end
    parts[#parts + 1] = p
  end
  if not found then
    if variant == "" then parts = {} end
    parts[#parts + 1] = attr .. "=" .. value
  end
  local out = base .. "!" .. table.concat(parts, "+")
  if hash ~= nil and hash ~= "" then out = out .. "%" .. hash end
  return out
end
function M.ToTPPName(app)
  if type(app) ~= "string" or app == "" or app == "None" then return app end
  if string.find(app, "!", 1, true) then
    if not string.find(app, "camera=fpp", 1, true) then return app end -- only names that select the FPP variant
    return setDynamicAttr(app, "camera", "tpp")
  end
  return (string.gsub(app, "&FPP", "&TPP"))
end
function M.ToFPPName(app)
  if type(app) ~= "string" or app == "" or app == "None" then return app end
  if string.find(app, "!", 1, true) then
    if not string.find(app, "camera=tpp", 1, true) then return app end
    return setDynamicAttr(app, "camera", "fpp")
  end
  return (string.gsub(app, "&TPP", "&FPP"))
end

-- ================================================================ L2 data and classifier
M.ACTING = { fppOnly = true, tppOnly = true, shadowProxy = true, renderPlaneFpp = true }
-- player entity meshes hidden in TPP (generator 1.1); looked up like "entity" (component name), but acted on
M.ENTITY_HIDE = { shadowProxy = true, renderPlaneFpp = true }
M.data = nil

-- tbl = body_parts.lua table: parts = { {gender, class, name, hash, path|false, sources}, ... }
function M.LoadData(tbl)
  if type(tbl) ~= "table" or type(tbl.parts) ~= "table" then return false, "no parts table" end
  local idx = {}
  local function put(map, k, cls)
    local cur = map[k]
    if cur == nil then map[k] = cls elseif cur ~= cls then map[k] = "ambiguous" end
  end
  local counts = {}
  for _, p in ipairs(tbl.parts) do
    local g, cls, name, hash = p[1], p[2], p[3], p[4]
    local gi = idx[g]
    if gi == nil then gi = { exact = {}, byHash = {}, byName = {}, owned = {} }; idx[g] = gi end
    counts[g .. "." .. cls] = (counts[g .. "." .. cls] or 0) + 1
    local h = (hash ~= nil and hash ~= "0") and hash or nil
    if name then gi.exact[name .. "|" .. (h or "-")] = cls end
    if cls == "entity" or cls == "head" or M.ENTITY_HIDE[cls] then
      if name then gi.owned[name] = cls end
    else
      if h then put(gi.byHash, h, cls) end
      if name then put(gi.byName, name, cls) end
    end
  end
  -- an acted-on entity name that is also used by an appearance part (female a0_004__weapon_grip_decal_01: TPP
  -- photo-mode decal = tppOnly) is never acted on by the name alone (hash unreadable) - only by the exact name + hash key
  for _, gi in pairs(idx) do
    for name, cls in pairs(gi.owned) do
      if M.ENTITY_HIDE[cls] and gi.byName[name] ~= nil then gi.owned[name] = "ambiguous" end
    end
  end
  M.data = { idx = idx, counts = counts, version = tbl.version, generator = tbl.generator, game = tbl.game,
             parts = #tbl.parts }
  return true
end

-- -> class, rule   (class: fppOnly | tppOnly | tppVariant | shared | ambiguous | entity | head | shadowProxy |
--                   renderPlaneFpp | unknown)
function M.Classify(gender, name, hash, path)
  local d = M.data
  if d == nil then return "unknown", "nodata" end
  local gi = d.idx[gender]
  if gi == nil then return "unknown", "nogender" end
  local h = M.NormHash(hash)
  if name then
    local c = gi.exact[name .. "|" .. (h or "-")]
    if c then return c, "exact" end
    c = gi.owned[name]
    if c then return c, "owned" end
  end
  if h and gi.byHash[h] then return gi.byHash[h], "hash" end
  if name and gi.byName[name] then return gi.byName[name], "name" end
  local p = lower(path)
  if p and (string.find(p, "\\player_female_average\\fpp\\", 1, true) or string.find(p, "\\player_man_average\\fpp\\", 1, true)) then
    return "fppOnly", "path"
  end
  return "unknown", "none"
end

-- ================================================================ body controller (pure; game access through api)
-- api = {
--   gender() -> "female"|"male"|nil        repState() -> "tpp"|"fpp"|"unknown"
--   activateRep(), deactivateRep(), feature(on)          gameOwnedReason() -> string|nil
--   footTpp(on) -> false when there is no player   (optional: wrapper variable utpp_foot_tpp)
--   frozen() -> string|nil (photo mode, no player)       playerKey() -> any (changes on load / respawn)
--   components() -> { {name, hash, path, enabled, ref}, ... }    setEnabled(comp, on) -> bool
--   garments() -> { {slot, key, appearance, ref}, ... }           setAppearance(garment, name) -> bool
--   setGarmentOffsets(on) -> bool          log(msg)
--   vehicle() -> { state = PSM Vehicle int, mounted = bool }   (head guard windows)
-- }   cfg = function() -> { enabled, hideFpp, showTpp, garments, garmentOffsets, shadowProxy }
M.DEFAULTS = { activationTimeout = 1.0, maxRetries = 3, verifyInterval = 0.5, unknownLogCap = 48, fastVerifyInterval = 0.1,
               -- (cfg.retry): s between retry rounds, s of fast verification after a nudge, s the first request
               -- after a player change / unfreeze waits, max logged rounds (then every 10th)
               retryEvery = 5.0, nudgeWindow = 1.0, settle = 0.5, retryLogCap = 10, maxStrategyRounds = 6,
               -- s between repeated sends of the utpp_foot_tpp wrapper value
               footReassert = 2.0 }

local Body = {}
Body.__index = Body
M.Body = Body

function M.NewBody(api, cfg, opts)
  local b = setmetatable({ api = api, cfgFn = cfg, opt = {} }, Body)
  for k, v in pairs(M.DEFAULTS) do b.opt[k] = (opts and opts[k] ~= nil) and opts[k] or v end
  b.wanted, b.wantReason = false, nil
  b.external = {}
  b.loggedOnce, b.unknownLogged, b.unknownSuppressed = {}, 0, 0
  b.stats = { activations = 0, deactivations = 0, retries = 0, handovers = 0, hidden = 0, shown = 0, reapplied = 0,
              restored = 0, restoreSkipped = 0, garments = 0, garmentRestored = 0, unknown = 0, steps = 0,
              failures = 0, resets = 0, entityHidden = 0, unmountChecks = 0,
              rounds = 0, nudges = 0, rearms = 0, toggles = 0, items = 0,
              footSends = 0, footFails = 0 }
  b:ResetEntity()
  return b
end

function Body:Log(msg) if self.api.log then pcall(self.api.log, "[utpp_body] " .. msg) end end
function Body:LogOnce(key, msg)
  if self.loggedOnce[key] then return end
  self.loggedOnce[key] = true
  self:Log(msg)
end

-- per-entity state (cleared when the player entity changes: load, respawn)
function Body:ResetEntity()
  self.owner = nil          -- nil | "utpp" | "game"
  self.pendingSince = nil   -- time of the last activation request
  self.retries = 0
  self.failed = false
  self.repOkSince = nil
  self.nextVerify = 0
  self.l2 = {}              -- key -> { from = bool, to = bool, name }
  self.l2Count = 0
  self.l3 = {}              -- garment key -> { from, to, slot }
  self.l3Count = 0
  self.offsetsOff = false
  self.rep = "unknown"
  -- (cfg.retry)
  self.attempt = 0          -- activation attempts of this request (strategy selection)
  self.retryAt = nil        -- next retry round after "did not activate"
  self.rounds = 0
  self.nudgeUntil = nil     -- fast verification window opened by Nudge
  self.settleUntil = nil    -- first activation not before this (player change / unfreeze)
  self.lastNudgeAt = nil
  -- utpp_foot_tpp wrapper value last sent (nil = unknown: the new entity starts at 0) and the next re-send time
  self.footSent, self.footNext, self.footRetry = nil, 0, false
  -- the same for the melee wrapper variable utpp_melee_tpp (value = foot AND the mod's animations are the pack in force)
  self.meleeSent, self.meleeNext, self.meleeRetry = nil, 0, false
end

-- sets the wrapper variable utpp_melee_tpp, the gate of the UTPP melee clips (archive\utpp_melee_tpp.archive.xl): 1 while the
-- UTPP body is wanted on foot (the value of utpp_foot_tpp) and the Game Animations pack is not in force (cfg.packGame), so the
-- game's own melee animations play with that pack. Same cadence and failure handling as the foot variable; no api entry
-- (older scripts) = nothing is sent.
function Body:SyncMelee(now, want)
  local api = self.api
  if not api.meleeTpp then return end
  local v = (want and not self:Cfg().packGame) and 1 or 0
  if (self.meleeSent == v or self.meleeRetry) and now < (self.meleeNext or 0) then return end
  local ok, res = pcall(api.meleeTpp, v == 1)
  if ok and res ~= false then
    if self.meleeSent ~= v then self:Log(string.format("t=%.2f utpp_melee_tpp=%d", now, v)) end
    self.meleeSent, self.meleeRetry = v, false
    self.meleeNext = now + self.opt.footReassert
  else
    self.meleeNext, self.meleeRetry = now + 0.25, true
    if ok then self:LogOnce("meleenoplayer", "utpp_melee_tpp not sent yet (no player); retrying")
    else self:LogOnce("meleefail", "utpp_melee_tpp send FAILED: " .. tostring(res)) end
  end
end

-- sets the wrapper variable utpp_foot_tpp (see the header): sent when the value changes, when unknown (new entity,
-- unfreeze) and every opt.footReassert s. A failed send (no player yet, old scripts) is retried after 0.25 s and logged once.
function Body:SyncFoot(now, want)
  self:SyncMelee(now, want)
  local api = self.api
  if not api.footTpp then return end
  local v = want and 1 or 0
  if (self.footSent == v or self.footRetry) and now < (self.footNext or 0) then return end
  local ok, res = pcall(api.footTpp, v == 1)
  if ok and res ~= false then
    if self.footSent ~= v then self:Log(string.format("t=%.2f utpp_foot_tpp=%d", now, v)) end
    self.footSent, self.footRetry = v, false
    self.footNext = now + self.opt.footReassert
    self.stats.footSends = self.stats.footSends + 1
  else
    self.stats.footFails = self.stats.footFails + 1
    self.footNext, self.footRetry = now + 0.25, true
    if ok then self:LogOnce("footnoplayer", "utpp_foot_tpp not sent yet (no player); retrying")
    else self:LogOnce("footfail", "utpp_foot_tpp send FAILED: " .. tostring(res)) end
  end
end

function Body:Cfg()
  local ok, c = pcall(self.cfgFn)
  if ok and type(c) == "table" then return c end
  return { enabled = false }
end

function Body:SetWanted(on, reason)
  on = on and true or false
  if self.wanted ~= on then
    self.wanted, self.wantReason = on, reason
    self.nextVerify = 0
    self:Log(string.format("t=%.2f TPP body %s (%s)", self.now or 0, on and "requested" or "released", tostring(reason)))
    if on then self.retries, self.failed, self.attempt, self.retryAt = 0, false, 0, nil end
  elseif on and self:Cfg().retry then
    -- a repeated request (camera entry after a save load while the body was still wanted) re-arms the retries
    -- instead of being a no-op ("full body via utpp_body: requested" changed nothing, the 3 retries were spent)
    if self.failed or self.rep ~= "tpp" then
      local wasFailed = self.failed
      self.retries, self.failed, self.retryAt, self.nextVerify = 0, false, nil, 0
      self.stats.rearms = self.stats.rearms + 1
      self:Log(string.format("t=%.2f TPP body requested again (%s): rep=%s owner=%s%s - retries re-armed", self.now or 0,
        tostring(reason), tostring(self.rep), tostring(self.owner), wasFailed and ", was given up" or ""))
    end
  end
end

-- an event that may have re-resolved the player's appearance (aim / zoom enter and exit, weapon change, camera or
-- perspective change): the parts are verified at once and every fastVerifyInterval s for nudgeWindow s, a given-up
-- request is re-armed. Only while the TPP body is wanted and cfg.retry is on (off = no-op). One line per nudge
-- (nudges closer than 0.2 s to the previous one are silent).
function Body:Nudge(now, why)
  if not self.wanted or not self:Cfg().retry then return false end
  self.stats.nudges = self.stats.nudges + 1
  self.nudgeUntil = now + self.opt.nudgeWindow
  self.nextVerify = 0
  local rearmed = false
  if self.failed then
    self.retries, self.failed, self.retryAt = 0, false, nil
    self.stats.rearms = self.stats.rearms + 1
    rearmed = true
  end
  if self.lastNudgeAt == nil or now - self.lastNudgeAt >= 0.2 or rearmed then
    self:Log(string.format("t=%.2f nudge (%s): rep=%s owner=%s retries=%d - parts verified now and for %.1f s%s", now,
      tostring(why), tostring(self.rep), tostring(self.owner), self.retries, self.opt.nudgeWindow,
      rearmed and ", given-up request re-armed" or ""))
  end
  self.lastNudgeAt = now
  return true
end

function Body:SetExternal(reason, active)
  if type(reason) ~= "string" or reason == "" then return false end
  local was = self.external[reason] ~= nil
  self.external[reason] = active and true or nil
  if was ~= (active and true or false) then self:Log("game-owned reason " .. reason .. (active and " on" or " off")) end
  return true
end

function Body:GameReason()
  local r = nil
  for k in pairs(self.external) do r = r and (r .. "+" .. k) or k end
  if r then return r end
  if self.api.gameOwnedReason then
    local ok, v = pcall(self.api.gameOwnedReason)
    if ok and type(v) == "string" and v ~= "" then return v end
  end
  return nil
end

function Body:HasChanges() return self.l2Count > 0 or self.l3Count > 0 or self.offsetsOff end

-- ---------------------------------------------------------------- L2
local function compKey(c) return tostring(c.name) .. "|" .. (M.NormHash(c.hash) or "-") end

function Body:ApplyL2(cfg)
  local gender = self.api.gender and self.api.gender() or nil
  if gender ~= "female" and gender ~= "male" then
    self:LogOnce("gender", "L2 skipped: player gender unknown (" .. tostring(gender) .. ")")
    return
  end
  local ok, comps = pcall(self.api.components)
  if not ok or type(comps) ~= "table" then
    self:LogOnce("comps", "L2 skipped: component list unavailable (" .. tostring(comps) .. ")")
    return
  end
  local planes = nil -- native plane registry, read once per scan when the first unknown part is logged
  for _, c in ipairs(comps) do
    local cls = M.Classify(gender, c.name, c.hash, c.path)
    local key = compKey(c)
    local want = nil
    if cls == "fppOnly" and cfg.hideFpp ~= false then want = false
    elseif cls == "tppOnly" and cfg.showTpp ~= false then want = true
    elseif cls == "shadowProxy" and cfg.shadowProxy ~= false then want = false
    elseif cls == "renderPlaneFpp" and cfg.hideFpp ~= false then want = false
    elseif cls == "unknown" then
      local lk = gender .. "|" .. key
      if not self.loggedOnce[lk] then
        self.loggedOnce[lk] = true
        self.stats.unknown = self.stats.unknown + 1
        if self.unknownLogged < self.opt.unknownLogCap then
          self.unknownLogged = self.unknownLogged + 1
          planes = planes or self:PlaneIndex()
          self:Log(string.format("unknown part left untouched: %s hash=%s enabled=%s componentType=%s resourceField=%s%s%s",
            tostring(c.name), tostring(M.NormHash(c.hash)), tostring(c.enabled), tostring(c.class),
            tostring(c.resourceField), M.PathNote(c, self.api), M.RenderMetaLine(c, planes)))
        else
          self.unknownSuppressed = self.unknownSuppressed + 1
        end
      end
    end
    if want ~= nil and c.enabled ~= want then
      local was = c.enabled
      local okS, res = pcall(self.api.setEnabled, c, want)
      if okS and res ~= false then
        local rec = self.l2[key]
        if rec == nil then
          self.l2[key] = { from = was, to = want, name = c.name, cls = cls }
          self.l2Count = self.l2Count + 1
          if want then self.stats.shown = self.stats.shown + 1 else self.stats.hidden = self.stats.hidden + 1 end
          if M.ENTITY_HIDE[cls] then -- one line per corrected FPP entity mesh
            self.stats.entityHidden = self.stats.entityHidden + 1
            self:Log(string.format("t=%.2f TPP: %s %s disabled (%s; restored in FPP)", self.now or 0,
              cls == "shadowProxy" and "shadow proxy" or "weapon-plane FPP part", tostring(c.name),
              cls == "shadowProxy" and "separate FPP shadow proxy"
                or "renderingPlaneAnimationParam renderPlane - drawn over the TPP body"))
          end
        else
          self.stats.reapplied = self.stats.reapplied + 1 -- the game reverted our change: apply again, keep `from`
        end
      else
        self:LogOnce("set|" .. key, "toggle FAILED for " .. tostring(c.name) .. ": " .. tostring(res))
      end
    end
  end
end

function Body:RestoreL2()
  if self.l2Count == 0 then return end
  local ok, comps = pcall(self.api.components)
  if not ok or type(comps) ~= "table" then comps = {} end
  local seen = {}
  for _, c in ipairs(comps) do
    local key = compKey(c)
    local rec = self.l2[key]
    if rec then
      seen[key] = true
      if c.enabled == rec.to then
        local okS, res = pcall(self.api.setEnabled, c, rec.from)
        if okS and res ~= false then
          self.stats.restored = self.stats.restored + 1
          if M.ENTITY_HIDE[rec.cls] then
            self:Log(string.format("t=%.2f FPP: %s restored (enabled=%s)", self.now or 0, tostring(rec.name), tostring(rec.from)))
          end
        else self.stats.restoreSkipped = self.stats.restoreSkipped + 1 end
      else
        self.stats.restoreSkipped = self.stats.restoreSkipped + 1 -- the game changed it since: leave it to the game
      end
    end
  end
  for key in pairs(self.l2) do
    if not seen[key] then self.stats.restoreSkipped = self.stats.restoreSkipped + 1 end
  end
  self.l2, self.l2Count = {}, 0
end

-- ---------------------------------------------------------------- L3
function Body:ApplyL3(cfg)
  if cfg.garmentOffsets and not self.offsetsOff and self.api.setGarmentOffsets then
    local ok, res = pcall(self.api.setGarmentOffsets, false)
    if ok and res ~= false then self.offsetsOff = true
    else self:LogOnce("offsets", "garment offsets toggle unavailable (ArchiveXL missing?)") end
  elseif not cfg.garmentOffsets and self.offsetsOff then
    self:RestoreOffsets()
  end
  if not cfg.garments or not self.api.garments then return end
  local ok, list = pcall(self.api.garments)
  if not ok or type(list) ~= "table" then return end
  local present = {}
  for _, g in ipairs(list) do
    present[g.key] = true
    local rec = self.l3[g.key]
    local target = M.ToTPPName(g.appearance)
    if rec ~= nil and g.appearance == rec.to then
      -- already ours
    elseif target ~= nil and target ~= g.appearance then
      local was = g.appearance
      local okS, res = pcall(self.api.setAppearance, g, target)
      if okS and res ~= false then
        if rec == nil then
          self.l3[g.key] = { from = was, to = target, slot = g.slot }
          self.l3Count = self.l3Count + 1
          self.stats.garments = self.stats.garments + 1
        else
          rec.to = target
          self.stats.reapplied = self.stats.reapplied + 1
        end
      else
        self:LogOnce("app|" .. tostring(g.key), "garment appearance change FAILED for " .. tostring(g.key) .. ": " .. tostring(res))
      end
    end
  end
  for key in pairs(self.l3) do -- item unequipped / replaced: its record is void
    if not present[key] then self.l3[key] = nil; self.l3Count = self.l3Count - 1 end
  end
end

function Body:RestoreOffsets()
  if not self.offsetsOff then return end
  pcall(self.api.setGarmentOffsets, true)
  self.offsetsOff = false
end

function Body:RestoreL3()
  self:RestoreOffsets()
  if self.l3Count == 0 then return end
  local ok, list = pcall(self.api.garments)
  if ok and type(list) == "table" then
    for _, g in ipairs(list) do
      local rec = self.l3[g.key]
      if rec and g.appearance == rec.to then
        local okS, res = pcall(self.api.setAppearance, g, rec.from)
        if okS and res ~= false then self.stats.garmentRestored = self.stats.garmentRestored + 1 end
      end
    end
  end
  self.l3, self.l3Count = {}, 0
end

-- ---------------------------------------------------------------- L1 + step
-- strategy of the n-th attempt of a request while cfg.retry is on: 1-2 event, then toggle / item / event cycling
-- (toggle = DeactivateTPPRepresentationEvent + ActivateTPPRepresentationEvent queued in order: an activation the game
-- believes is already active is re-done; item = the TPP head item of V's gender equipped as the head guard does, plus
-- the event). Off: always the event.
function Body:Strategy(cfg)
  if not cfg.retry then return "event" end
  -- inside a vehicle window (mount / exit / unmount, set by M.StepVehicle) only the plain event: an explicit
  -- deactivate or a removed and re-added head item during the exit showed V headless / with a floating head
  if self.vehWindow ~= nil then return "event" end
  local n = self.attempt
  if n <= 2 then return "event" end
  -- bounded: toggle / item only in the first maxStrategyRounds rounds (legit FPP states must not flicker forever)
  if (self.rounds or 0) > (self.opt.maxStrategyRounds or 6) then return "event" end
  local k = (n - 3) % 3
  if k == 0 then return "toggle" elseif k == 1 then return "item" end
  return "event"
end

function Body:Activate(now, retry, cfg)
  cfg = cfg or self:Cfg()
  self.attempt = (self.attempt or 0) + 1
  local how = self:Strategy(cfg)
  local extra = ""
  if how == "toggle" then
    local okD, errD = pcall(self.api.deactivateRep)
    self.stats.toggles = self.stats.toggles + 1
    if not okD then extra = " (deactivate FAILED: " .. short(errD) .. ")" end
  elseif how == "item" then
    self.stats.items = self.stats.items + 1
    if self.api.equipTppHead then
      local okI, res, why = pcall(self.api.equipTppHead)
      extra = " (head item " .. ((okI and res) and ("equipped: " .. tostring(why)) or ("FAILED: " .. short(okI and why or res))) .. ")"
    else
      extra = " (no head item api)"
    end
  end
  local ok, err = pcall(self.api.activateRep)
  if not ok then self:LogOnce("act", "activate representation FAILED: " .. tostring(err)) end
  if self.api.feature then pcall(self.api.feature, true) end
  self.owner, self.pendingSince = "utpp", now
  self.stats.activations = self.stats.activations + 1
  if retry then self.stats.retries = self.stats.retries + 1 end
  -- strategy and the TppHead slot state appended (append-only: the earlier text is the prefix)
  self:Log(string.format("t=%.2f representation activate%s via %s slot=%s attempt=%d%s", now,
    retry and (" (retry " .. self.retries .. ")") or "", how, tostring(self.rep), self.attempt, extra))
end

function Body:Deactivate(now, why)
  local ok, err = pcall(self.api.deactivateRep)
  if not ok then self:LogOnce("deact", "deactivate representation FAILED: " .. tostring(err)) end
  if self.api.feature then pcall(self.api.feature, false) end
  self.stats.deactivations = self.stats.deactivations + 1
  self.owner, self.pendingSince = nil, nil
  self:Log(string.format("t=%.2f representation deactivate (%s)", now, why))
end

function Body:RestoreAll(now, why)
  if self:HasChanges() then self:Log(string.format("t=%.2f restore FPP parts (%s)", now, why)) end
  self:RestoreL3()
  self:RestoreL2()
end

function Body:Step(now)
  self.now = now
  self.stats.steps = self.stats.steps + 1
  local cfg = self:Cfg()
  local api = self.api
  if api.playerKey then
    local ok, key = pcall(api.playerKey)
    key = ok and key or nil
    if key ~= self.playerKey then
      if self.playerKey ~= nil then
        self.stats.resets = self.stats.resets + 1
        self:Log("player entity changed - state reset (old records dropped)")
      end
      local changed = self.playerKey ~= nil
      self.playerKey = key
      self:ResetEntity()
      -- after a player change (load / respawn) the loading game dropped the event sent at once
      if changed and cfg.retry then self.settleUntil = now + self.opt.settle end
    end
  end
  local frozen = nil
  if api.frozen then
    local ok, f = pcall(api.frozen)
    if ok and type(f) == "string" and f ~= "" then frozen = f end
  end
  if frozen then
    if self.frozen ~= frozen then self:Log("frozen: " .. frozen .. " (no writes)") end
    self.frozen = frozen
    return
  end
  if self.frozen then
    self:Log("unfrozen (" .. self.frozen .. ") - full verification")
    -- after "noplayer" (load / respawn) the first request waits settle s (the event sent at the
    -- unfreeze and three retries were ignored while the game was still loading)
    if cfg.retry and self.frozen == "noplayer" and self.owner == nil then self.settleUntil = now + self.opt.settle end
    self.frozen = nil
    self.nextVerify = 0
    self.footSent = nil -- the wrapper value is unknown after a load
    self.meleeSent = nil
  end
  if not self.wanted and self.owner == nil and not self:HasChanges() then
    self:SyncFoot(now, false) -- sends only when the value is unknown or due (no game reads otherwise)
    self.rep = "idle"
    return
  end

  local okR, rep = pcall(api.repState)
  rep = (okR and (rep == "tpp" or rep == "fpp")) and rep or "unknown"
  self.rep = rep
  local gameReason = self:GameReason()
  self.gameReason = gameReason
  local wanted = self.wanted and cfg.enabled -- setting off -> treated like FPP (restore + release)
  -- before any activation below (the graph edge ANY -> tpp_securitycam_hacking needs the variable in place first);
  -- a vanilla take-over of control (security camera) keeps its own behaviour
  self:SyncFoot(now, wanted and not (gameReason and string.find(gameReason, "takeover", 1, true)))

  if wanted then
    if rep == "tpp" then
      if self.owner == nil then
        self.owner = "game"
        self:Log("representation already active (game-owned" .. (gameReason and (": " .. gameReason) or "") .. ") - never deactivated by UTPP")
      end
      self.pendingSince, self.retries = nil, 0
      self.attempt = 0 -- a success ends the attempt series; the next loss starts again with the plain event
    elseif gameReason then
      -- the game drives the representation now (takeover, scene ...): do not fight it
    elseif self.failed then
      -- gave up until the next request; with cfg.retry a new round every retryEvery s, never final
      if cfg.retry and self.retryAt ~= nil and now >= self.retryAt then
        self.rounds = self.rounds + 1
        self.stats.rounds = self.stats.rounds + 1
        self.retries, self.failed, self.retryAt = 0, false, nil
        if self.rounds <= self.opt.retryLogCap or self.rounds % 10 == 0 then
          self:Log(string.format("t=%.2f representation retry round %d (every %.0f s): slot=%s owner=%s", now, self.rounds,
            self.opt.retryEvery, tostring(rep), tostring(self.owner)))
        end
        self:Activate(now, false, cfg)
      end
    elseif cfg.retry and self.settleUntil ~= nil and now < self.settleUntil and self.owner == nil then
      -- first request after a player change / unfreeze: let the game settle
    elseif self.owner == "utpp" and self.pendingSince and now - self.pendingSince < self.opt.activationTimeout then
      -- waiting for the game to process the event
    elseif rep == "unknown" and self.owner == "utpp" then
      -- cannot read the TppHead slot: trust the request after the timeout
    elseif self.owner == "utpp" and self.pendingSince then
      if self.retries < self.opt.maxRetries then
        self.retries = self.retries + 1
        self:Activate(now, true, cfg)
      else
        self.failed = true
        self.stats.failures = self.stats.failures + 1
        if cfg.retry then
          -- backoff: after maxStrategyRounds the rounds come 6x slower
          self.retryAt = now + self.opt.retryEvery * ((self.rounds or 0) > (self.opt.maxStrategyRounds or 6) and 6 or 1)
          self:Log(string.format("representation did not activate after %d retries (slot=%s) - next round in %.0f s, or at "
            .. "the next request / nudge", self.retries, tostring(rep), self.opt.retryEvery))
        else
          self:Log("representation did not activate after " .. self.retries .. " retries - giving up until the next request")
        end
      end
    else
      if self.owner == "utpp" then self:Log("representation was deactivated by the game - activating again") end
      self.retries = 0
      self.settleUntil = nil
      self:Activate(now, false, cfg)
    end
    local repOk = rep == "tpp" or (rep == "unknown" and self.owner == "utpp" and self.pendingSince ~= nil
      and now - self.pendingSince >= self.opt.activationTimeout)
    if repOk then
      if self.repOkSince == nil then self.repOkSince = now; self.nextVerify = 0 end
      -- a nudge window (aim / zoom enter and exit ...) verifies as fast as a vehicle head guard window
      local fast = self.fastVerify or (self.nudgeUntil ~= nil and now < self.nudgeUntil)
      if fast and self.nextVerify > now + self.opt.fastVerifyInterval then
        self.nextVerify = now + self.opt.fastVerifyInterval -- a window just opened: do not wait for the slow watchdog
      end
      if now >= self.nextVerify then
        -- inside a vehicle head guard window (mount / exit / unmount) the parts are re-checked every 0.1 s
        self.nextVerify = now + (fast and math.min(self.opt.verifyInterval, self.opt.fastVerifyInterval)
          or self.opt.verifyInterval)
        if self.owner == "utpp" and api.feature then pcall(api.feature, true) end
        self:ApplyL2(cfg)
        self:ApplyL3(cfg)
      end
    else
      self.repOkSince = nil
    end
  else
    self.repOkSince = nil
    if self:HasChanges() then self:RestoreAll(now, self.wanted and "disabled" or "FPP") end
    if self.owner == "utpp" then
      if gameReason then
        self.owner = "game"
        self.stats.handovers = self.stats.handovers + 1
        self:Log("representation handed over to the game (" .. gameReason .. ") - not deactivated")
      elseif rep == "fpp" then
        self.owner, self.pendingSince = nil, nil
      else
        self:Deactivate(now, self.wanted and "disabled" or "FPP")
      end
    elseif self.owner == "game" and rep == "fpp" then
      self.owner = nil
    end
  end
end

-- vehicle unmount (M.NewVehicleWindows): the TPP representation is re-checked at once - retry budget
-- and "gave up" reset, parts verified now; the representation request itself runs in Step (rep fpp -> Activate)
function Body:VehicleUnmount(now)
  if not self.wanted then return false end
  self.stats.unmountChecks = self.stats.unmountChecks + 1
  local wasFailed = self.failed
  self.retries, self.failed, self.nextVerify = 0, false, 0
  self:Log(string.format("t=%.2f vehicle unmount: TPP representation re-checked now (rep=%s owner=%s%s)", now,
    tostring(self.rep), tostring(self.owner), wasFailed and ", retries re-armed" or ""))
  return true
end

function Body:State()
  return { enabled = self:Cfg().enabled and true or false, wanted = self.wanted, owner = self.owner, rep = self.rep,
           gameReason = self.gameReason, frozen = self.frozen, failed = self.failed, l2 = self.l2Count, l3 = self.l3Count,
           offsetsOff = self.offsetsOff, stats = self.stats,
           retry = self:Cfg().retry and true or false, attempt = self.attempt, rounds = self.rounds, retryAt = self.retryAt }
end

function Body:StatusLine()
  local s = self.stats
  return string.format("utpp_body %s enabled=%s wanted=%s owner=%s rep=%s game=%s frozen=%s failed=%s l2=%d l3=%d offsetsOff=%s "
    .. "act=%d deact=%d retries=%d handovers=%d hidden=%d shown=%d reapplied=%d restored=%d skipped=%d garments=%d/%d unknown=%d%s data=%s",
    M.VERSION, tostring(self:Cfg().enabled), tostring(self.wanted), tostring(self.owner), tostring(self.rep),
    tostring(self.gameReason), tostring(self.frozen), tostring(self.failed), self.l2Count, self.l3Count, tostring(self.offsetsOff),
    s.activations, s.deactivations, s.retries, s.handovers, s.hidden, s.shown, s.reapplied, s.restored, s.restoreSkipped,
    s.garments, s.garmentRestored, s.unknown, self.unknownSuppressed > 0 and (" (+" .. self.unknownSuppressed .. " not logged)") or "",
    M.data and (tostring(M.data.generator) .. " parts=" .. M.data.parts) or "MISSING")
    -- (append-only)
    .. string.format(" retry=%s attempt=%d rounds=%d rearms=%d nudges=%d toggles=%d items=%d", tostring(self:Cfg().retry and true or false),
      self.attempt or 0, s.rounds, s.rearms, s.nudges, s.toggles, s.items)
end

-- ================================================================ head guard
-- V had no head in third person for
-- ~2 s right after the unmount (and briefly at the mount start). Confirmed in game:
-- right after the unmount the TppHead slot goes Items.Player?aTppHead -> Items.PlayerFppHead (the game's own TPP
-- representation of the vehicle TPP camera ends) while the camera stays in TPP.
-- Repair: the first attempt never worked - (1) the representation event could
-- not be created ("cannot create ActivateTPPRepresentationEvent": the RTTI name is gameActivateTPPRepresentationEvent,
-- the script alias ActivateTPPRepresentationEvent was the only name tried - RED4ext.SDK
-- Generated\game\ActivateTPPRepresentationEvent.hpp NAME/ALIAS), (2) both head item equips reported success but the
-- slot stayed FPP for 2.5 s (cause unconfirmed, see GameApi.equipTppHead). The guard now polls every 0.05 s, repairs
-- after 0.05 s, re-checks 0.12 s after each repair, alternates representation event / head item (item at once when the
-- event cannot be created), waits while the TPP head is still spawning, max 4 repairs per loss and 8 per window, and
-- releases a representation it activated itself when the window ends because UTPP no longer wants the TPP body (hint "fpp").
-- slot tpp -> empty -> fpp 0.05 s after the
-- unmount; (1) CET could not create the event under ANY name (RTTI name and alias, .new() and NewObject) -> the event
-- now goes through the redscript bridge scripts\utpp_body.reds (M.QueueRep: Game.UTPPQueueTppRepresentation, then the
-- UTPPBodyBridge scriptable system, CET creation only as the last resort); (2) four head item equips, each
-- "add=true, spawning=true, inSlot=false", 0.15 s apart, each removing the slot content and adding again (hypothesis:
-- that restarts / cancels a spawn; but the polls read "fpp", not "spawning", 0.05 s after each add and even the last
-- item never arrived, so the game itself probably dropped it - the event is the main repair, the item a fallback).
-- Now an item repair is followed by a wait of >= itemWait s (longer while
-- IsSlotSpawningItem / HasItemInSlot report our item, up to itemWaitMax), and an item that is still spawning is never
-- removed or added again (GameApi.equipTppHead).
-- Narrow mode: only inside windows opened by utpp_body's own vehicle driver (M.NewVehicleWindows,
-- Guard:SetWindow). A head of any other item ("other") or an unreadable slot is never touched; a TPP head (ours or the
-- game's) is never re-equipped.
-- api = { headSlot() -> "tpp"|"fpp"|"empty"|"spawning"|"other"|"unknown", activateRep() -> how, feature(on),
--         equipTppHead() -> ok, why, headItemState() -> "spawning"|"inslot"|"none"|"unknown" (optional, our TPP head item),
--         releaseRep() -> done, why, frozen() -> string|nil, log(msg),
--         setHeadHidden(on, who) -> ok, why (optional: mesh components of the TPP head item in the slot) }
-- cfg = function() -> bool (guard allowed)
-- exitDelay / exitPoll for the first exitFastFor s of an 'exit' window; a head that re-spawns in a window stays
-- hidden until hideReads consecutive 'tpp' polls plus one frame (polls every exitPoll s meanwhile), at most hideMax s
M.HEAD_DEFAULTS = { poll = 0.05, delay = 0.05, verify = 0.12, maxRepairs = 4, maxWindowRepairs = 8,
                    itemWait = 0.5, itemWaitMax = 1.5,
                    exitDelay = 0, exitPoll = 0.016, exitFastFor = 1.0, hideReads = 2, hideMax = 0.3 }
local Guard = {}
Guard.__index = Guard
M.Guard = Guard

function M.NewHeadGuard(api, cfg, opts)
  local g = setmetatable({ api = api, cfgFn = cfg, opt = {} }, Guard)
  for k, v in pairs(M.HEAD_DEFAULTS) do g.opt[k] = (opts and opts[k] ~= nil) and opts[k] or v end
  g.active, g.loggedOnce = false, {}
  g.stats = { windows = 0, missing = 0, repairs = 0, events = 0, eventFails = 0, items = 0, restored = 0, gaveUp = 0,
              polls = 0, spawnWaits = 0, released = 0, worst = 0, itemWaits = 0,
              hidden = 0, shared = 0 } -- re-spawned heads hidden, repairs left to the body's event of the same frame
  return g
end

-- a TPP head that re-spawns inside a window (the game dropped its representation, or a repair) is drawn for a
-- few frames at a stale transform (floating head) before it follows the slot. Its mesh components are hidden from the
-- first poll that finds it in the slot (or while spawning) until hideReads consecutive 'tpp' polls plus one frame.
-- Only when the adapter offers setHeadHidden; bounded by hideMax; shown again when the window closes.
function Guard:HideStart(now)
  if self.hide ~= nil or self.api.setHeadHidden == nil then return end
  self.hide = { since = now, reads = 0, applied = false, appliedAt = nil, readyAt = nil }
end

function Guard:HideTry(now)
  local h = self.hide
  if h == nil or h.applied then return end
  local ok, done, why = pcall(self.api.setHeadHidden, true, "guard")
  if ok and done then
    h.applied, h.appliedAt = true, now
    self.stats.hidden = self.stats.hidden + 1
    self:Log(string.format("t=%.2f re-spawned TPP head hidden until settled (%s, %s)", now, tostring(why), tostring(self.reason)))
  end
end

function Guard:HideEnd(now, why)
  local h = self.hide
  if h == nil then return end
  self.hide = nil
  if not h.applied then return end
  local ok, done, whyS = pcall(self.api.setHeadHidden, false, "guard")
  self:Log(string.format("t=%.2f TPP head shown again (%s, hidden %.3f s, %d tpp read(s))%s", now, why, now - h.appliedAt,
    h.reads, (ok and done ~= false) and "" or (" - show FAILED: " .. tostring(ok and whyS or done))))
end

function Guard:Log(msg) if self.api.log then pcall(self.api.log, "[utpp_body] " .. msg) end end
-- Window source: utpp_body's own vehicle driver (M.NewVehicleWindows), set every frame. reason nil = no window;
-- hint (optional, with reason nil): "fpp" = the window ended because UTPP no longer wants the TPP body
function Guard:SetWindow(reason, hint) self.want, self.hint = reason, hint end

function Guard:Close(now, why, hint)
  self:HideEnd(now, "window closed")
  self:Log(string.format("t=%.2f head guard closed (%s, %s): %.2f s, %d repair(s), head %s", now, tostring(self.reason),
    why, now - (self.since or now), self.windowRepairs or 0, tostring(self.slot)))
  local noPlayer = false
  if self.repByGuard and hint == "fpp" and self.api.frozen then -- game exit / loading: no game call through a dead player
    local okF, f = pcall(self.api.frozen)
    noPlayer = okF and f == "noplayer"
  end
  if self.repByGuard and hint == "fpp" and self.api.releaseRep and not noPlayer then
    local ok, done, whyR = pcall(self.api.releaseRep)
    if ok and done then self.stats.released = self.stats.released + 1 end
    self:Log(string.format("t=%.2f head guard: TPP body no longer wanted - representation activated by the guard %s", now,
      (ok and done) and "released" or ("left to the game (" .. tostring(ok and whyR or done) .. ")")))
  end
  self.active, self.reason, self.missingSince, self.repByGuard, self.itemSince = false, nil, nil, false, nil
end

function Guard:RepairEvent(now, slot)
  -- with delay 0 (exit) the body's own re-activation and the guard's repair fall on the same frame: the event the
  -- body queued in this frame counts as this repair (never two activation events in one frame)
  if self.peerActivated then
    local okP, same = pcall(self.peerActivated, now)
    if okP and same then
      self.stats.shared = self.stats.shared + 1
      self:Log(string.format("t=%.2f head missing (slot %s, %s) -> TPP representation already requested by the body this frame",
        now, slot, self.reason))
      return true
    end
  end
  local ok, how = pcall(self.api.activateRep)
  if self.api.feature then pcall(self.api.feature, true) end
  self.stats.events = self.stats.events + 1
  if ok then self.repByGuard = true else self.stats.eventFails = self.stats.eventFails + 1 end
  self:Log(string.format("t=%.2f head missing (slot %s, %s) -> TPP representation requested%s", now, slot, self.reason,
    ok and (how and (" (" .. tostring(how) .. ")") or "") or (" FAILED: " .. tostring(how))))
  return ok
end

function Guard:RepairItem(now, slot)
  local ok, res, why = pcall(self.api.equipTppHead)
  self.stats.items = self.stats.items + 1
  if ok and res then self.itemSince = now end -- wait for the spawn, never remove / add again meanwhile
  self:Log(string.format("t=%.2f head missing (slot %s, %s) -> TPP head item %s", now, slot, self.reason,
    (ok and res) and ("equipped (" .. tostring(why) .. ")") or ("FAILED: " .. tostring(ok and why or res))))
end

-- while an item repair is pending -> true (no new repair). Waits at least itemWait s, longer (up to itemWaitMax)
-- while the game reports our TPP head item as spawning / in the slot; the outcome is logged once.
function Guard:ItemPending(now, slot)
  local elapsed = now - self.itemSince
  local st = "unknown"
  if self.api.headItemState then
    local ok, s = pcall(self.api.headItemState)
    if ok and type(s) == "string" then st = s end
  end
  if (st == "spawning" or st == "inslot") and elapsed < self.opt.itemWaitMax then
    self.stats.itemWaits = self.stats.itemWaits + 1
    return true
  end
  if elapsed < self.opt.itemWait then
    self.stats.itemWaits = self.stats.itemWaits + 1
    return true
  end
  self:Log(string.format("t=%.2f head item after %.2f s: item %s, slot %s - next repair allowed", now, elapsed, st, slot))
  self.itemSince = nil
  return false
end

function Guard:Step(now)
  local reason = self.want
  if reason == nil then
    if self.active then self:Close(now, "window ended", self.hint) end
    return
  end
  local okC, on = pcall(self.cfgFn)
  if not (okC and on) then
    if self.active then self:Close(now, "switched off") end
    if not self.loggedOnce.off then self.loggedOnce.off = true; self:Log("head guard requested but switched off (setting)") end
    return
  end
  if not self.active then
    self.active, self.reason, self.since, self.slot = true, reason, now, nil
    self.repairs, self.windowRepairs, self.lastRepairAt, self.missingSince = 0, 0, nil, nil
    self.gaveUp, self.nextPoll, self.repByGuard, self.itemSince = false, 0, false, nil
    self.reasonSince, self.hide = now, nil
    self.stats.windows = self.stats.windows + 1
    self:Log(string.format("t=%.2f head guard open (%s)", now, reason))
  elseif reason ~= self.reason then
    self:Log(string.format("t=%.2f head guard %s -> %s", now, tostring(self.reason), reason))
    self.reason, self.reasonSince = reason, now
  end
  if self.api.frozen then
    local okF, f = pcall(self.api.frozen)
    if okF and type(f) == "string" and f ~= "" then return end
  end
  -- a settled head is shown one frame after its last 'tpp' poll (a later Step = a rendered frame); bounded hide
  local h = self.hide
  if h ~= nil then
    if h.readyAt ~= nil and now > h.readyAt then self:HideEnd(now, "settled")
    elseif h.applied and now - h.appliedAt >= self.opt.hideMax then self:HideEnd(now, "hideMax") end
  end
  -- the game drops its representation ~0.47 s into a bike exit; the first exitFastFor s of an exit
  -- window are polled every exitPoll s and repaired on the first poll that reads fpp / empty (exitDelay)
  local exitFast = self.reason == "exit" and now - (self.reasonSince or self.since) < self.opt.exitFastFor
  if now < (self.nextPoll or 0) then return end
  self.nextPoll = now + (((exitFast or (self.hide ~= nil and not self.gaveUp)) and self.opt.exitPoll) or self.opt.poll)
  self.stats.polls = self.stats.polls + 1
  local okS, slot = pcall(self.api.headSlot)
  slot = okS and slot or "unknown"
  if slot ~= self.slot then
    self:Log(string.format("t=%.2f head slot %s -> %s (%s, %.2f s into the window)", now, tostring(self.slot), tostring(slot),
      self.reason, now - self.since))
    self.slot = slot
  end
  h = self.hide
  if slot == "tpp" then
    if h ~= nil then
      self:HideTry(now) -- the re-spawned head is in the slot now: hidden before it is drawn at a stale transform
      h.reads = h.reads + 1
      if h.reads >= self.opt.hideReads and h.readyAt == nil then h.readyAt = now end
    end
  elseif slot == "spawning" then
    self:HideStart(now)
    self:HideTry(now)
    if self.hide then self.hide.reads, self.hide.readyAt = 0, nil end
  elseif slot == "fpp" or slot == "empty" then
    if h ~= nil and h.applied then self:HideEnd(now, "head lost again") end -- the hidden item left the slot
    self:HideStart(now)
    if self.hide then self.hide.reads, self.hide.readyAt = 0, nil end
  elseif h ~= nil then
    h.reads, h.readyAt = 0, nil
  end
  if slot == "tpp" then
    if self.missingSince ~= nil then
      local lost = now - self.missingSince
      self.stats.restored = self.stats.restored + 1
      if lost > self.stats.worst then self.stats.worst = lost end
      self:Log(string.format("t=%.2f head back after %.2f s (%d repair(s))", now, lost, self.repairs))
    end
    self.missingSince, self.repairs, self.lastRepairAt, self.gaveUp, self.itemSince = nil, 0, nil, false, nil
    return
  end
  if slot == "spawning" then -- a TPP head (ours or the game's) is on its way: wait, never stack a second one
    if self.missingSince ~= nil then self.stats.spawnWaits = self.stats.spawnWaits + 1 end
    return
  end
  if slot ~= "fpp" and slot ~= "empty" then return end -- other head item / unreadable: not ours to touch
  if self.missingSince == nil then
    self.missingSince = now
    self.stats.missing = self.stats.missing + 1
  end
  if self.gaveUp or now - self.missingSince < (exitFast and self.opt.exitDelay or self.opt.delay) then return end
  if self.itemSince ~= nil and self:ItemPending(now, slot) then return end
  if self.lastRepairAt ~= nil and now - self.lastRepairAt < self.opt.verify then return end
  if self.repairs >= self.opt.maxRepairs or self.windowRepairs >= self.opt.maxWindowRepairs then
    self.gaveUp = true
    self.stats.gaveUp = self.stats.gaveUp + 1
    self:Log(string.format("t=%.2f head still missing after %d repair(s) (%d in this window) - giving up until the next %s",
      now, self.repairs, self.windowRepairs, self.windowRepairs >= self.opt.maxWindowRepairs and "window" or "loss / window"))
    return
  end
  self.repairs, self.windowRepairs, self.lastRepairAt = self.repairs + 1, self.windowRepairs + 1, now
  self.stats.repairs = self.stats.repairs + 1
  if self.repairs % 2 == 1 then
    -- odd repair: the game's own representation event (+ anim feature); cannot be created -> head item at once
    if not self:RepairEvent(now, slot) then self:RepairItem(now, slot) end
  else
    self:RepairItem(now, slot)
  end
end

function Guard:StatusLine()
  local s = self.stats
  return string.format("headGuard=%s active=%s reason=%s slot=%s windows=%d missing=%d repairs=%d events=%d(failed %d) items=%d "
    .. "spawnWaits=%d itemWaits=%d restored=%d worst=%.2fs gaveUp=%d released=%d rep=%s",
    tostring(M.HeadGuardEnabled()), tostring(self.active), tostring(self.reason), tostring(self.slot), s.windows, s.missing,
    s.repairs, s.events, s.eventFails, s.items, s.spawnWaits, s.itemWaits, s.restored, s.worst, s.gaveUp, s.released,
    tostring(M.bridgeHow))
    .. string.format(" headsHidden=%d sharedRepairs=%d", s.hidden, s.shared) -- (append-only)
end

-- ================================================================ vehicle windows
-- Observed in a motorcycle exit: exit start (PSM Vehicle 1 -> 4),
-- the game dropped its TPP representation 0.47 s later ("representation was deactivated by the game - activating
-- again") and V vanished for a moment. The session had no "head guard" line: nothing opened
-- the windows, so the head repair and the faster checks never ran. utpp_body opens the windows itself while UTPP wants
-- the TPP body (game reads only then, or while a vehicle phase is still running):
--   mount   from the mount start (PSM Vehicle Transition 4 or GetMountedVehicle) until seated (PSM 1/2/3/5/6/7)
--   exit    from the exit start (PSM Transition 4 after seated) until the unmount (not mounted and PSM Vehicle 0)
--   unmount `after` s (3.0 = utpp_vehiclecam headGuardAfter) after the unmount
-- At the unmount Body:VehicleUnmount re-checks the representation at once; inside a window the parts are verified
-- every fastVerifyInterval s. api = { vehicle() -> { state, mounted }, log(msg) }; wanted() -> bool; onUnmount(now).
M.VEH_DEFAULTS = { after = 3.0, mountMax = 8.0 }
M.VEH_WINDOW = { mount = "mount", exit = "exit", after = "unmount" }
local SEATED = { [1] = true, [2] = true, [3] = true, [5] = true, [6] = true, [7] = true }
local VehWin = {}
VehWin.__index = VehWin
M.VehWin = VehWin

function M.NewVehicleWindows(api, wanted, onUnmount, opts)
  local v = setmetatable({ api = api, wantedFn = wanted, onUnmount = onUnmount, opt = {} }, VehWin)
  for k, d in pairs(M.VEH_DEFAULTS) do v.opt[k] = (opts and opts[k] ~= nil) and opts[k] or d end
  v.phase, v.since, v.window = "idle", 0, nil
  v.stats = { mounts = 0, exits = 0, unmounts = 0, windows = 0, reads = 0 }
  return v
end

function VehWin:Log(msg) if self.api.log then pcall(self.api.log, "[utpp_body] " .. msg) end end

function VehWin:SetPhase(p, now, why, obs)
  self:Log(string.format("t=%.2f vehicle %s -> %s (%s, PSM vehicle=%d mounted=%s)", now, self.phase, p, why,
    obs.state, tostring(obs.mounted)))
  self.phase, self.since = p, now
end

function VehWin:Unmounted(now, why, obs, wanted)
  self.stats.unmounts = self.stats.unmounts + 1
  self:SetPhase("after", now, why, obs)
  if wanted and self.onUnmount then pcall(self.onUnmount, now) end
end

-- -> window reason (nil = none), close hint ("fpp" = UTPP no longer wants the TPP body)
function VehWin:Step(now)
  local okW, wanted = pcall(self.wantedFn)
  wanted = okW and wanted == true
  if wanted or self.phase ~= "idle" then
    local obs = { state = 0, mounted = false }
    if self.api.vehicle then
      local ok, o = pcall(self.api.vehicle)
      if ok and type(o) == "table" then obs.state = tonumber(o.state) or 0; obs.mounted = o.mounted == true end
    end
    self.stats.reads = self.stats.reads + 1
    local inVeh = obs.mounted or obs.state ~= 0
    local seated = SEATED[obs.state] == true
    local p = self.phase
    if p == "idle" or p == "after" then
      if inVeh then
        self.stats.mounts = self.stats.mounts + 1
        if seated then self:SetPhase("ride", now, "already seated", obs) else self:SetPhase("mount", now, "mount start", obs) end
      elseif p == "after" and now - self.since >= self.opt.after then
        self:SetPhase("idle", now, "unmount window over", obs)
      end
    elseif p == "mount" then
      if not inVeh then self:Unmounted(now, "mount aborted", obs, wanted)
      elseif seated then self:SetPhase("ride", now, "seated", obs)
      elseif now - self.since > self.opt.mountMax then self:SetPhase("ride", now, "mount timeout", obs) end
    elseif p == "ride" then
      if not inVeh then self:Unmounted(now, "unmount", obs, wanted)
      elseif obs.state == 4 then self.stats.exits = self.stats.exits + 1; self:SetPhase("exit", now, "exit start", obs) end
    elseif p == "exit" then
      if not inVeh then self:Unmounted(now, "unmount", obs, wanted)
      elseif seated then self:SetPhase("ride", now, "exit aborted", obs) end
    end
  end
  local window = wanted and M.VEH_WINDOW[self.phase] or nil
  if window ~= self.window then
    if window ~= nil then self.stats.windows = self.stats.windows + 1 end
    self:Log(string.format("t=%.2f head guard window %s -> %s (vehicle %s)", now, tostring(self.window),
      tostring(window), self.phase))
    self.window = window
  end
  return window, (not wanted) and "fpp" or nil
end

-- ================================================================ vehicle camera perspective hook
-- While utpp_vehiclecam hands the ride camera to the game (keep 'game', e.g. driver combat), the game switches its own
-- vehicle camera (activePersp 0 = FPP, 1-6 = TPP / driver combat TPP) and UTPP did no head work around it. Observed:
-- a bald TPP head for ~4 frames after FPP -> TPP; a wrong arm / strip 1-3 frames ~0.45 s
-- after TPP -> FPP (2-wheel cameraBlendTime 0.4499 s) while the TPP head was still in the slot. utpp_vehiclecam reports
-- the change through its optional callback field onVehiclePerspective(prev, new) (set by utpp_body at run time,
-- M.TryHookVehicleCam) or a direct call of M.OnVehiclePerspective(prev, new):
--   FPP -> TPP: the TPP head item is equipped at once (only when the slot reads fpp / empty; a spawning head is never
--               stacked), and a head hidden by an earlier TPP -> FPP is shown first;
--   TPP -> FPP: at the end of the blend (blend s), if the camera is still FPP and the TPP head (with its hair) is still in
--               the slot, its meshes are hidden until the slot leaves 'tpp', the unmount, the next FPP -> TPP or holdMax s.
-- Player body parts (arms) are not touched here: the game keeps TPP arm parts enabled in FPP (shadows); which part draws
-- the wrong arm is not identified yet. No game reads while nothing is pending.
-- api = { headSlot(), equipTppHead(), setHeadHidden(on, who), vehicle() -> { state, mounted }, log(msg) };
-- enabled() -> bool (body.impl and body.headGuard)
M.VPERSP_DEFAULTS = { blend = 0.45, holdMax = 0.5 }
local VPersp = {}
VPersp.__index = VPersp
M.VPersp = VPersp

function M.NewVehiclePerspective(api, enabled, opts)
  local v = setmetatable({ api = api, enabledFn = enabled, opt = {}, loggedOnce = {} }, VPersp)
  for k, d in pairs(M.VPERSP_DEFAULTS) do v.opt[k] = (opts and opts[k] ~= nil) and opts[k] or d end
  v.last, v.hideAt, v.hiddenAt = nil, nil, nil
  v.stats = { events = 0, preEquips = 0, hides = 0, shows = 0, ignored = 0 }
  return v
end

function VPersp:Log(msg) if self.api.log then pcall(self.api.log, "[utpp_body] " .. msg) end end

local function vtpp(p) return p ~= nil and p >= 1 and p <= 6 end

function VPersp:Show(now, why)
  if self.hiddenAt == nil then return end
  local ok, done, whyS = pcall(self.api.setHeadHidden, false, "vehcam")
  self.stats.shows = self.stats.shows + 1
  self:Log(string.format("t=%.2f vehicle camera: TPP head shown again (%s, hidden %.3f s)%s", now, why, now - self.hiddenAt,
    (ok and done ~= false) and "" or (" - show FAILED: " .. tostring(ok and whyS or done))))
  self.hiddenAt = nil
end

-- -> true when the change was handled
function VPersp:OnChange(now, prev, new)
  prev, new = tonumber(prev), tonumber(new)
  if new == nil or prev == new then return false end
  if self.lastAt == now and self.lastPrev == prev and self.last == new then return false end -- same event, second route
  self.lastAt, self.lastPrev, self.last = now, prev, new
  self.stats.events = self.stats.events + 1
  local okE, on = pcall(self.enabledFn)
  if not (okE and on) then
    self.stats.ignored = self.stats.ignored + 1
    if not self.loggedOnce.off then
      self.loggedOnce.off = true
      self:Log("vehicle camera perspective change ignored: body visibility or head guard switched off (setting)")
    end
    self.hideAt = nil
    self:Show(now, "switched off")
    return false
  end
  if new == 0 and vtpp(prev) then
    self.hideAt = now + self.opt.blend
    self:Log(string.format("t=%.2f vehicle camera TPP -> FPP (%d -> 0): TPP head checked at the end of the %.2f s blend", now,
      prev, self.opt.blend))
    return true
  end
  if vtpp(new) and prev == 0 then
    self.hideAt = nil
    self:Show(now, "FPP -> TPP")
    local okS, slot = pcall(self.api.headSlot)
    slot = okS and slot or "unknown"
    if (slot == "fpp" or slot == "empty") and self.api.equipTppHead then
      local okI, res, why = pcall(self.api.equipTppHead)
      self.stats.preEquips = self.stats.preEquips + 1
      self:Log(string.format("t=%.2f vehicle camera FPP -> TPP (0 -> %d): TPP head pre-equipped %s", now, new,
        (okI and res) and ("(" .. tostring(why) .. ")") or ("FAILED: " .. tostring(okI and why or res))))
    else
      self:Log(string.format("t=%.2f vehicle camera FPP -> TPP (0 -> %d): head slot %s - nothing pre-equipped", now, new, slot))
    end
    return true
  end
  return false -- TPP -> TPP (distance / combat camera): nothing to do
end

-- review fix: true while the UTPP camera shows V in third person (viewTpp, set by M.Attach: the TPP body is requested,
-- e.g. by the exit take-over from the vehicle camera). The head is never hidden then and a hidden head is shown again;
-- the vehicle camera reports nothing outside its own ride (no FPP -> TPP event for the take-over).
function VPersp:ViewTpp()
  if type(self.viewTpp) ~= "function" then return false end
  local ok, r = pcall(self.viewTpp)
  return ok and r == true
end

function VPersp:Step(now)
  if self.hideAt == nil and self.hiddenAt == nil then return end -- nothing pending: no game reads
  local api = self.api
  if self.hideAt ~= nil and now >= self.hideAt then
    self.hideAt = nil
    if self.last == 0 and api.setHeadHidden and self:ViewTpp() then
      self:Log(string.format("t=%.2f vehicle camera FPP blend over: third-person view (UTPP camera) - nothing hidden", now))
    elseif self.last == 0 and api.setHeadHidden then
      local okS, slot = pcall(api.headSlot)
      slot = okS and slot or "unknown"
      if slot == "tpp" then
        local ok, done, why = pcall(api.setHeadHidden, true, "vehcam")
        if ok and done then
          self.hiddenAt = now
          self.stats.hides = self.stats.hides + 1
          self:Log(string.format("t=%.2f vehicle camera FPP blend over, TPP head still in the slot: hidden (%s)", now, tostring(why)))
        else
          self:Log(string.format("t=%.2f vehicle camera FPP blend over, TPP head still in the slot: hide FAILED (%s)", now,
            tostring(ok and why or done)))
        end
      else
        self:Log(string.format("t=%.2f vehicle camera FPP blend over: head slot %s - nothing hidden", now, slot))
      end
    end
    return
  end
  if self.hiddenAt ~= nil then
    local okV, o = pcall(api.vehicle or function() return nil end)
    if okV and type(o) == "table" and o.mounted ~= true and (tonumber(o.state) or 0) == 0 then
      self:Show(now, "unmount")
      return
    end
    if self:ViewTpp() then
      self:Show(now, "third-person view")
      return
    end
    local okS, slot = pcall(api.headSlot)
    if not (okS and slot == "tpp") then self:Show(now, "head swapped, slot " .. tostring(okS and slot or "unknown"))
    elseif now - self.hiddenAt >= self.opt.holdMax then self:Show(now, "holdMax") end
  end
end

function VPersp:StatusLine()
  local s = self.stats
  return string.format("vehPersp=%s events=%d preEquips=%d hides=%d shows=%d ignored=%d hook=%s", tostring(self.last),
    s.events, s.preEquips, s.hides, s.shows, s.ignored, tostring(M.vcamHook and M.vcamHook.state))
end

-- ================================================================ exit keep of the TPP representation
-- Both bike exits: 'head slot tpp -> empty' 0.47 s after the exit start,
-- head re-spawned 0.10 s later (head guard repair), and for 6 frames the vehicle graph showed its first-person branch
-- (Switch on TPPRepresentation.IsActive): V standing beside the bike, arm raised, Hips +22.7 cm sideways, Spine3 49 deg.
-- Trigger (40+ exits): the drop comes 0.45-0.48 s after the vehicle camera's active perspective
-- turns FPP (2-wheel camera blend 0.45 s), also when that turn is 0.2 s after the exit start (car exit delay) and when
-- the ride camera was the game's own; no script sends it (native). The head guard repairs after the fact only.
-- While the guard window is 'exit' and UTPP wants the TPP body (no scene / takeover reason), this keeps:
--   (1) the anim feature TPPRepresentation.IsActive = true every frame (feature(true)), so the graph stays on its
--       third-person branch through a drop;
--   (2) the redscript latch (setRepKeep(true, latch), refreshed every refresh s): a deactivation that reaches the player
--       is answered in the same dispatch with an activation + the feature (scripts\utpp_body.reds);
--   (3) diagnostics: the bridge counters (repKeepDiag) are polled every diagPoll s while open and for tail s after;
--       every change is logged ("representation keep diag ...": game = deactivations seen by the script handler).
-- Bounded: maxWindow s per window (then off until the window ends), the latch expires latch s after the last refresh.
--   (4) the native guard (nativeGuard(on) -> hook live): opened with the window, closed with it (UTPP.dll caps it at
--       3 s); while open the vehicle camera's own deactivation at the end of its TPP -> FPP switch is not sent.
-- api = { feature(on), setRepKeep(on, s) -> bool, repKeepDiag() -> string, nativeGuard(on) -> bool, log(msg) };
-- want(now) -> bool
M.REPKEEP_DEFAULTS = { latch = 0.5, refresh = 0.25, maxWindow = 3.0, diagPoll = 0.1, tail = 0.6 }
local RepKeep = {}
RepKeep.__index = RepKeep
M.RepKeep = RepKeep

function M.NewRepKeep(api, opts)
  local k = setmetatable({ api = api, opt = {}, loggedOnce = {} }, RepKeep)
  for n, d in pairs(M.REPKEEP_DEFAULTS) do k.opt[n] = (opts and opts[n] ~= nil) and opts[n] or d end
  k.active, k.blocked, k.since, k.nextRefresh, k.diagUntil, k.nextDiag, k.diag = false, false, nil, 0, nil, 0, nil
  k.stats = { windows = 0, features = 0, featureFails = 0, latches = 0, latchFails = 0, timeouts = 0, diagChanges = 0,
              nativeOn = 0, nativeOff = 0, nativeAbsent = 0 }
  return k
end

function RepKeep:Log(msg) if self.api.log then pcall(self.api.log, "[utpp_body] " .. msg) end end

function RepKeep:Latch(on)
  if not self.api.setRepKeep then return false end
  local ok, r = pcall(self.api.setRepKeep, on, on and self.opt.latch or 0)
  if on then
    if ok and r ~= false then self.stats.latches = self.stats.latches + 1 else self.stats.latchFails = self.stats.latchFails + 1 end
    if not (ok and r ~= false) and not self.loggedOnce.latch then
      self.loggedOnce.latch = true
      self:Log("representation keep: redscript latch not set (" .. tostring(ok and r or ("error " .. short(r)))
        .. ") - anim feature only")
    end
  end
  return ok and r ~= false
end

-- native guard of the vehicle camera deactivation; a missing native / not installed hook is logged once
function RepKeep:NativeGuard(on)
  if not self.api.nativeGuard then return false end
  local ok, r = pcall(self.api.nativeGuard, on)
  local live = ok and r == true
  if on then self.stats.nativeOn = self.stats.nativeOn + 1 else self.stats.nativeOff = self.stats.nativeOff + 1 end
  if on and not live then
    self.stats.nativeAbsent = self.stats.nativeAbsent + 1
    if not self.loggedOnce.native then
      self.loggedOnce.native = true
      self:Log("representation keep: native vehicle exit guard not live (" .. tostring(ok and r or ("error " .. short(r)))
        .. ") - script keep only")
    end
  end
  self.nativeLive = live
  return live
end

function RepKeep:Open(now, why)
  self.active, self.since, self.nextRefresh = true, now, 0
  self.windowFeatures = 0
  self.stats.windows = self.stats.windows + 1
  self.diagUntil, self.nextDiag = math.huge, 0
  local native = self:NativeGuard(true)
  self:Log(string.format("t=%.2f representation keep on (%s): anim feature every frame, latch %.2f s refreshed every %.2f s, "
    .. "max %.1f s, native guard %s", now, why, self.opt.latch, self.opt.refresh, self.opt.maxWindow,
    native and "on" or "absent"))
end

function RepKeep:Close(now, why)
  self.active = false
  self:Latch(false)
  if self.api.nativeGuard then self:NativeGuard(false) end
  self.diagUntil = now + self.opt.tail
  self:Log(string.format("t=%.2f representation keep off (%s): %.2f s, feature x%d, diag %s", now, why, now - (self.since or now),
    self.windowFeatures or 0, tostring(self.diag)))
end

function RepKeep:PollDiag(now)
  if self.diagUntil == nil or now > self.diagUntil or now < self.nextDiag or not self.api.repKeepDiag then return end
  self.nextDiag = now + self.opt.diagPoll
  local ok, d = pcall(self.api.repKeepDiag)
  if not (ok and type(d) == "string" and d ~= "") then return end
  -- only the counters decide a change (the 'last' / 'keep' clocks run every poll)
  local counters = string.match(d, "^(.-) last=") or d
  if counters ~= self.diagCounters then
    if self.diagCounters ~= nil then
      self.stats.diagChanges = self.stats.diagChanges + 1
      self:Log(string.format("t=%.2f representation keep diag: %s (%s)", now, d, self.active and "window open" or "after the window"))
    end
    self.diagCounters = counters
  end
  self.diag = d
end

-- one frame: want = the exit window is open and UTPP wants the TPP body
function RepKeep:Step(now, want)
  if not want then
    if self.active then self:Close(now, "window ended") end
    self.blocked = false
  elseif not self.active and not self.blocked then
    self:Open(now, "exit window")
  end
  if self.active and now - self.since > self.opt.maxWindow then
    self.stats.timeouts = self.stats.timeouts + 1
    self.blocked = true
    self:Close(now, string.format("maxWindow %.1f s", self.opt.maxWindow))
  end
  if self.active then
    if self.api.feature then
      local ok = pcall(self.api.feature, true)
      if ok then
        self.stats.features = self.stats.features + 1
        self.windowFeatures = (self.windowFeatures or 0) + 1
      else
        self.stats.featureFails = self.stats.featureFails + 1
      end
    end
    if now >= self.nextRefresh then
      self.nextRefresh = now + self.opt.refresh
      self:Latch(true)
    end
  end
  self:PollDiag(now)
end

function RepKeep:StatusLine()
  local s = self.stats
  return string.format("repKeep=%s active=%s windows=%d features=%d(failed %d) latches=%d(failed %d) timeouts=%d diagChanges=%d "
    .. "native=%d/%d(absent %d) diag=%s",
    tostring(M.settings.repKeep), tostring(self.active), s.windows, s.features, s.featureFails, s.latches, s.latchFails,
    s.timeouts, s.diagChanges, s.nativeOn, s.nativeOff, s.nativeAbsent, tostring(self.diag))
end

-- ================================================================ settings (utpp_core) and module API
M.settings = { enabled = true, hideFpp = true, showTpp = true, garments = true, garmentOffsets = false, headGuard = true,
               shadowProxy = true, retry = true, repKeep = true } -- repKeep: module switch (M.SetRepKeep), no menu entry
M.core = nil

function M.RegisterSettings(core)
  if type(core) ~= "table" or type(core.Register) ~= "function" then return false end
  M.core = core
  core.RegisterSection("body", "Body visibility", { owner = "utpp_body" })
  core.Register({ key = "body.impl", section = "body", type = "bool", default = true, label = "Use UTPP body visibility",
    desc = "Full body of V in the UTPP third-person camera (game TPP representation + first-person part cleanup)." })
  core.Register({ key = "body.hideFpp", section = "body", type = "bool", default = true, label = "Hide first-person body parts",
    desc = "Third person: hide parts that exist only in first person (arms, torso, neck, nails of the FPP body)." })
  core.Register({ key = "body.showTpp", section = "body", type = "bool", default = true, label = "Show third-person body parts",
    desc = "Third person: enable full-body parts the game left disabled (table generated from game data)." })
  core.Register({ key = "body.shadowProxy", section = "body", type = "bool", default = true, label = "Hide first-person shadow proxy",
    desc = "Third person: disable the recognized first-person shadow-only proxy. Other meshes and their shadow settings remain unchanged." })
  core.Register({ key = "body.garments", section = "body", type = "bool", default = true, label = "Third-person clothing variants",
    desc = "Third person: switch trousers and shoes from their first-person variant (&FPP) to the third-person one." })
  core.Register({ key = "body.headGuard", section = "body", type = "bool", default = true, label = "Keep V's head around vehicles",
    desc = "If V's head disappears in third person while getting into or out of a vehicle, restore it." })
  core.Register({ key = "body.garmentOffsets", section = "body", type = "bool", default = false, label = "Disable garment offsets in TPP",
    desc = "ArchiveXL garment offsets off while in third person (only if clothing clips; global ArchiveXL switch)." })
  core.Register({ key = "body.retry", section = "body", type = "bool", default = true, label = "Keep retrying the third-person body",
    desc = "If the game ignores the third-person body request (e.g. right after loading a save: shirt / arm missing when aiming), "
      .. "ask again every 5 s with changing methods, and re-check the body parts right after aiming / zooming. Off = three tries, then give up." })
  return true
end

-- effective configuration (utpp_core when available, else module defaults)
function M.Config()
  local core = M.core
  local c = {}
  if core then
    c.enabled = core.Effective("body.impl") == true
    c.hideFpp = core.Get("body.hideFpp") ~= false
    c.showTpp = core.Get("body.showTpp") ~= false
    c.garments = core.Get("body.garments") ~= false
    c.garmentOffsets = core.Get("body.garmentOffsets") == true
    c.shadowProxy = core.Get("body.shadowProxy") ~= false
    c.retry = core.Get("body.retry") ~= false
    c.packGame = type(core.PackIsGame) == "function" and core.PackIsGame() == true
  else
    local s = M.settings
    c.enabled = s.enabled == true
    c.hideFpp, c.showTpp, c.garments, c.garmentOffsets = s.hideFpp, s.showTpp, s.garments, s.garmentOffsets
    c.shadowProxy = s.shadowProxy ~= false
    c.retry = s.retry ~= false
  end
  return c
end

-- head guard switch: utpp_core setting body.headGuard (default on); SetHeadGuard(true|false|nil) = A/B override, not saved
M.headGuardOverride = nil
function M.HeadGuardEnabled()
  if M.headGuardOverride ~= nil then return M.headGuardOverride end
  if M.core then return M.core.Get("body.headGuard") ~= false end
  return M.settings.headGuard ~= false
end

M.body = nil
M.guard = nil
M.veh = nil
-- vehicle windows only while UTPP wants the TPP body and the module is enabled
function M.VehicleWanted()
  return M.body ~= nil and M.body.wanted == true and M.Config().enabled == true
end
function M.Attach(api, opts)
  M.api = api
  M.body = M.NewBody(api, M.Config, opts)
  M.guard = M.NewHeadGuard(api, M.HeadGuardEnabled, opts and opts.guard)
  M.veh = M.NewVehicleWindows(api, M.VehicleWanted, function(now) if M.body then M.body:VehicleUnmount(now) end end,
    opts and opts.veh)
  -- the body's activation of this frame counts as the guard's event repair (no second event in one frame)
  M.guard.peerActivated = function(now)
    local b = M.body
    return b ~= nil and b.owner == "utpp" and b.pendingSince == now
  end
  M.vpersp = M.NewVehiclePerspective(api, M.VehiclePerspectiveEnabled, opts and opts.vpersp)
  M.vpersp.viewTpp = function() return M.IsTppBody() end -- review fix: no head hide in the UTPP third-person view
  M.repkeep = M.NewRepKeep(api, opts and opts.repkeep)
  return M.body
end
-- one frame of the vehicle windows (after Body:Step, before Guard:Step); frozen (photo mode / no player) = no step
function M.StepVehicle(now)
  if M.veh == nil or M.body == nil or M.body.frozen ~= nil then return nil end
  local reason, hint = M.veh:Step(now)
  if M.guard then M.guard:SetWindow(reason, hint) end
  M.body.fastVerify = reason ~= nil
  M.body.vehWindow = reason -- Body:Strategy uses the plain event inside a window
  if M.vpersp then M.vpersp:Step(now) end
  if M.repkeep then M.repkeep:Step(now, M.RepKeepWanted(reason)) end
  return reason
end
-- the exit keep runs in the 'exit' window while UTPP wants the TPP body, unless the game drives the representation
-- for a scene / takeover / external reason (the vehicle's own TPP camera does not count: its release is the trigger)
function M.RepKeepWanted(reason)
  if reason ~= "exit" or M.body == nil or M.body.wanted ~= true or not M.RepKeepEnabled() then return false end
  local g = M.body.gameReason
  return g == nil or g == "vehicleTPP"
end
-- vehicle camera perspective hook (see VPersp). Enabled with the body module and the head guard setting.
function M.VehiclePerspectiveEnabled() return M.Config().enabled == true and M.HeadGuardEnabled() end
function M.OnVehiclePerspective(prev, new)
  if M.vpersp == nil then return false end
  local ok, handled = pcall(M.vpersp.OnChange, M.vpersp, M.clock, prev, new)
  return ok and handled == true
end
-- Run-time detection of utpp_vehiclecam's optional callback field onVehiclePerspective (nil by default there): set to
-- M.OnVehiclePerspective; an existing function there is kept and called after ours. Tried once per second for 30 s
-- (CET load order), then "absent" (logged once). getMod = CET GetMod. -> state "pending" | "attached" | "absent" | "incompatible"
M.vcamHook = { state = "pending", tries = 0, nextTry = 0 }
M.VCAM_HOOK_TRIES = 30
function M.TryHookVehicleCam(now, getMod)
  local hk = M.vcamHook
  if hk.state ~= "pending" or now < hk.nextTry or type(getMod) ~= "function" then return hk.state end
  hk.tries, hk.nextTry = hk.tries + 1, now + 1.0
  local ok, vc = pcall(getMod, "utpp_vehiclecam")
  if ok and type(vc) == "table" then
    local cur = vc.onVehiclePerspective
    if cur == nil or cur == M.OnVehiclePerspective then
      vc.onVehiclePerspective = M.OnVehiclePerspective
      hk.state = "attached"
    elseif type(cur) == "function" then
      vc.onVehiclePerspective = function(a, b)
        local r = M.OnVehiclePerspective(a, b)
        pcall(cur, a, b)
        return r
      end
      hk.state = "attached"
    else
      hk.state = "incompatible"
    end
    M.LogLine(string.format("[utpp_body] vehicle camera hook: utpp_vehiclecam.onVehiclePerspective %s", hk.state == "attached"
      and ((cur ~= nil and cur ~= M.OnVehiclePerspective) and "set (existing function chained)" or "set")
      or ("left alone (field is a " .. type(cur) .. ")")))
  elseif hk.tries >= M.VCAM_HOOK_TRIES then
    hk.state = "absent"
    M.LogLine("[utpp_body] vehicle camera hook: utpp_vehiclecam not found - vehicle camera head handling off")
  end
  return hk.state
end
M.clock = 0
function M.SetHeadGuard(on)
  if on == nil then M.headGuardOverride = nil else M.headGuardOverride = on and true or false end
  return M.HeadGuardEnabled()
end
-- exit keep of the TPP representation (RepKeep); A/B switch for the console, not saved
function M.SetRepKeep(on)
  M.settings.repKeep = on and true or false
  return M.RepKeepEnabled()
end
function M.RepKeepEnabled() return M.settings.repKeep ~= false and M.Config().enabled == true and M.HeadGuardEnabled() end
function M.HeadGuardStatus() return M.guard and M.guard:StatusLine() or "headGuard not attached" end
-- one-line console checks (in game): print(GetMod("utpp_body").HeadProbe())  -> slot, gender, which event / ItemID
-- creation works (nothing is queued or equipped); print(GetMod("utpp_body").HeadRepair("event"|"item")) -> one repair
-- now + the slot right after (a queued event / spawned head changes the slot a few frames later: HeadProbe again).
function M.HeadProbe()
  local api = M.api
  if api == nil or api.headSlot == nil then return "utpp_body: no game api" end
  local out = {}
  local okS, slot = pcall(api.headSlot)
  out[#out + 1] = "slot=" .. tostring(okS and slot or ("ERR " .. tostring(slot)))
  -- the redscript bridge (utpp_body.reds) answers without queueing anything -> proves it is loaded and which
  -- route CET resolves (Game.<global> or the UTPPBodyBridge scriptable system)
  local okB, rs, rhow = pcall(M.RedsSlot)
  out[#out + 1] = "reds=" .. (okB and (tostring(rs) .. " via " .. tostring(rhow)) or ("FAILED " .. tostring(rs)))
  if api.headItemState then
    local okI, st = pcall(api.headItemState)
    out[#out + 1] = "item=" .. tostring(okI and st or ("ERR " .. tostring(st)))
  end
  if api.gender then
    local okG, g = pcall(api.gender)
    out[#out + 1] = "gender=" .. tostring(okG and g or "?")
  end
  for _, kind in ipairs({ "activate", "deactivate", "feature" }) do
    local ok, o, how = pcall(M.CreateNative, M.EVENT_NAMES[kind])
    out[#out + 1] = kind .. "=" .. (ok and tostring(how) or ("FAILED " .. tostring(o)))
  end
  if M.ItemIdOf then
    local ok, id, how = pcall(M.ItemIdOf, "Items.PlayerWaTppHead")
    out[#out + 1] = "itemId=" .. (ok and tostring(how) or ("FAILED " .. tostring(id)))
  end
  local line = "[utpp_body] head probe: " .. table.concat(out, " ")
  M.LogLine(line)
  return line
end
function M.HeadRepair(kind)
  local api = M.api
  if api == nil then return "utpp_body: no game api" end
  local ok, a, b
  if kind == "item" then ok, a, b = pcall(api.equipTppHead)
  else
    ok, a = pcall(api.activateRep)
    if api.feature then pcall(api.feature, true) end
  end
  local okS, slot = pcall(api.headSlot)
  local line = string.format("[utpp_body] head repair %s: %s %s%s -> slot now %s", tostring(kind or "event"),
    ok and "ok" or "FAILED", tostring(a), b ~= nil and (" " .. tostring(b)) or "", tostring(okS and slot or "?"))
  M.LogLine(line)
  return line
end

function M.Enabled() return M.Config().enabled and true or false end
function M.SetTppBody(on, reason)
  if M.body == nil then return false, "not attached" end
  if on and not M.Enabled() then
    M.body:LogOnce("reqdisabled", "SetTppBody(true) ignored: UTPP body visibility is off (setting)")
    M.body.wanted = false
    return false, "disabled"
  end
  M.body:SetWanted(on, reason or "api")
  return true
end
function M.IsTppBody() return M.body ~= nil and M.body.wanted end
-- utpp_camera (aim / zoom enter and exit) and any caller: verify the parts now, re-arm a given-up request
function M.Nudge(why)
  if M.body == nil then return false, "not attached" end
  return M.body:Nudge(M.clock, why or "api")
end
function M.SetGameOwned(reason, active) return M.body ~= nil and M.body:SetExternal(reason, active) end
function M.Refresh() if M.body then M.body.nextVerify = 0 end return M.body ~= nil end
function M.GetState() return M.body and M.body:State() or { enabled = M.Enabled(), attached = false } end
function M.Status()
  if not M.body then return "utpp_body " .. M.VERSION .. " not attached" end
  return M.body:StatusLine() .. (M.guard and (" " .. M.guard:StatusLine()) or "")
    .. (M.veh and string.format(" vehicle=%s window=%s mounts=%d exits=%d unmounts=%d unmountChecks=%d entityHidden=%d",
      M.veh.phase, tostring(M.veh.window), M.veh.stats.mounts, M.veh.stats.exits, M.veh.stats.unmounts,
      M.body.stats.unmountChecks, M.body.stats.entityHidden) or "")
    .. (M.vpersp and (" " .. M.vpersp:StatusLine()) or "") -- (append-only)
    .. (M.repkeep and (" " .. M.repkeep:StatusLine()) or "") -- (append-only)
end

-- Read-only snapshot of player mesh components and garment item appearances. The class/rule is our
-- classifier; componentType/resourceField are the engine adapter values, not an inferred render plane.
function M.Dump()
  local b = M.body
  if not b then return 0 end
  local g = b.api.gender and b.api.gender() or nil
  local ok, comps = pcall(b.api.components, true)
  if not ok or type(comps) ~= "table" then b:Log("dump: components unavailable"); return 0 end
  b:Log(string.format("dump: gender=%s rep=%s components=%d", tostring(g), tostring(b.rep), #comps))
  local planes = b:PlaneIndex() -- one native registry read per Dump (the native logs the full text itself)
  b:Log("dump: itemPlane " .. (planes.why or (tostring(planes.header) .. " entries=" .. #planes.lines)))
  for _, c in ipairs(comps) do
    local cls, rule = M.Classify(g, c.name, c.hash, c.path)
    b:Log(string.format("  %-10s %-6s %s enabled=%s hash=%s componentType=%s resourceField=%s%s%s",
      cls, rule, tostring(c.name), tostring(c.enabled), tostring(M.NormHash(c.hash)), tostring(c.class),
      tostring(c.resourceField), M.PathNote(c, b.api), M.RenderMetaLine(c, planes)))
  end
  if b.api.garments then
    local okG, garments = pcall(b.api.garments)
    if okG and type(garments) == "table" then
      b:Log(string.format("dump: garment items=%d", #garments))
      for _, garment in ipairs(garments) do
        b:Log(string.format("  garment slot=%s key=%s appearance=%s", tostring(garment.slot),
          tostring(garment.key), tostring(garment.appearance)))
      end
    else b:Log("dump: garment items unavailable") end
  end
  return #comps
end

-- data table next to init.lua (CET: relative to the mod folder); tests may set UTPP_BODY_DATA_PATH
do
  local loaded, err = false, "not found"
  local candidates = {}
  if type(UTPP_BODY_DATA_PATH) == "string" then candidates[#candidates + 1] = UTPP_BODY_DATA_PATH end
  candidates[#candidates + 1] = "body_parts.lua"
  for _, p in ipairs(candidates) do
    local ok, tbl = pcall(dofile, p)
    if ok and type(tbl) == "table" then loaded, err = M.LoadData(tbl); if loaded then break end
    else err = tostring(tbl) end
  end
  if not loaded then
    local ok, tbl = pcall(require, "body_parts")
    if ok and type(tbl) == "table" then loaded, err = M.LoadData(tbl) end
  end
  M.dataError = (not loaded) and tostring(err) or nil
end

-- ================================================================ native objects from CET
-- CET resolves classes by their RTTI name. For these events the RTTI name differs from the redscript alias
-- (RED4ext.SDK Generated\game\*.hpp: NAME "gameActivateTPPRepresentationEvent", ALIAS "ActivateTPPRepresentationEvent");
-- tried only the alias and failed in game. Classes whose RTTI name equals
-- the alias work with <name>.new() (utpp_vehiclecam: vehicleRequestCameraPerspectiveEvent.new(), "queued on player: ok").
-- RTTI names first, the alias only as a fallback; the first method that works is reported (logged by the caller).
M.EVENT_NAMES = {
  activate = { "gameActivateTPPRepresentationEvent", "ActivateTPPRepresentationEvent" },
  deactivate = { "gameDeactivateTPPRepresentationEvent", "DeactivateTPPRepresentationEvent" },
  feature = { "gameAnimFeature_TPPRepresentation", "AnimFeature_TPPRepresentation" },
}
-- ("attempt to index local 'env' (a nil value)"): the CET mod sandbox has no _G, so
-- the old default (env falling back to _G) gave nil and the redscript bridge was never called. M.MakeEnv(src, fallback) ->
-- table that resolves a name lazily in src (live lookup), then in fallback (values captured at load). M.DefaultEnv() is
-- built once at load from the environment of this chunk (getfenv(1) in CET = LuaJIT / Lua 5.1), else _G, plus the
-- captured Game / NewObject / GetPlayer. Tests still pass an explicit env.
-- ("attempt to index upvalue 'G' (a nil value)" for both bridge routes): the
-- default still gave Game = nil in CET - the values captured at load are nil (CET provides Game & co. only after
-- the chunk ran) and getfenv(1) did not give a table holding Game (not whitelisted or a different table). Now the CET
-- globals are referenced BY NAME inside this chunk at call time (liveGlobals: the lookup goes through whatever
-- environment CET gave the chunk, at that moment), first; the getfenv source only for other names (class tables).
-- src / fallback of MakeEnv: a table or a function returning a table (evaluated at every lookup).
local function lookup(t, k)
  if type(t) == "function" then
    local ok, r = pcall(t)
    t = ok and r or nil
  end
  if type(t) ~= "table" then return nil end
  local ok, v = pcall(function() return t[k] end)
  if ok then return v end
  return nil
end
function M.MakeEnv(src, fallback)
  return setmetatable({}, { __index = function(_, k)
    local v = lookup(src, k)
    if v == nil then v = lookup(fallback, k) end
    return v
  end })
end
-- the names below are free variables of this chunk: resolved in the chunk's environment when called, never cached
local function liveGlobals()
  return { Game = Game, NewObject = NewObject, GetPlayer = GetPlayer, GetSingleton = GetSingleton }
end
local function loadEnvSource()
  if type(getfenv) == "function" then
    local ok, e = pcall(function() return getfenv(1) end) -- level 1 = this closure (pcall(getfenv, 1) would see pcall's env)
    if ok and type(e) == "table" then return e end
  end
  if type(_G) == "table" then return _G end
  return nil
end
local DEFAULT_ENV = M.MakeEnv(liveGlobals, loadEnvSource())
function M.DefaultEnv() return DEFAULT_ENV end

-- env: table with the CET globals (tests pass a mock); -> object, "how"; error lists every method tried
function M.CreateNative(names, env)
  env = env or DEFAULT_ENV
  local tried = {}
  for _, n in ipairs(names) do
    local tries = {
      { n .. ".new()", function() return env[n].new() end },
      { "NewObject(" .. n .. ")", function() return env.NewObject(n) end },
      { "NewObject(handle:" .. n .. ")", function() return env.NewObject("handle:" .. n) end },
    }
    for _, t in ipairs(tries) do
      local ok, o = pcall(t[2])
      if ok and o ~= nil then return o, t[1] end
      tried[#tried + 1] = t[1]
    end
  end
  error("cannot create " .. tostring(names[1]) .. " (tried " .. table.concat(tried, ", ") .. ")")
end

-- ================================================================ redscript bridge (scripts\utpp_body.reds)
-- CET could not create the representation events under any name in game;
-- redscript constructs them like vanilla (takeOverControlSystem.script). Routes, first that answers true wins:
--   1) Game.UTPPQueueTppRepresentation(on)   (script global)
--   2) Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):QueueTppRepresentation(on)
--   3) CET creation (M.CreateNative, RTTI name then alias) + QueueEvent on the player - kept for a game without the .reds
-- The bridge answers false without a player (and, for on = false, while the vehicle TPP camera owns the representation).
-- env: table with the CET globals (tests pass a mock). -> how (string); error lists every route and its answer.
M.bridgeHow = nil
short = function(v)
  local s = string.gsub(tostring(v), "^[^\n]-:%d+: ", "") -- drop the "file:line: " prefix of Lua errors
  return #s > 160 and (string.sub(s, 1, 157) .. "...") or s
end
function M.QueueRep(on, env)
  env = env or DEFAULT_ENV
  on = on and true or false
  local G, tried = env.Game, {}
  local function route(label, fn)
    local ok, res = pcall(fn)
    if ok and res == true then return true end
    tried[#tried + 1] = label .. (ok and ("=" .. tostring(res)) or (" error " .. short(res)))
    return false
  end
  local arg = on and "true" or "false"
  if route("Game.UTPPQueueTppRepresentation", function() return G.UTPPQueueTppRepresentation(on) end) then
    M.bridgeHow = "Game.UTPPQueueTppRepresentation(" .. arg .. ")"
    return M.bridgeHow
  end
  if route("UTPPBodyBridge", function()
    return G.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):QueueTppRepresentation(on)
  end) then
    M.bridgeHow = "UTPPBodyBridge:QueueTppRepresentation(" .. arg .. ")"
    return M.bridgeHow
  end
  local okN, ev, how = pcall(M.CreateNative, on and M.EVENT_NAMES.activate or M.EVENT_NAMES.deactivate, env)
  if okN then
    local okQ, errQ = pcall(function() G.GetPlayer():QueueEvent(ev) end)
    if okQ then M.bridgeHow = how; return how end
    tried[#tried + 1] = how .. " QueueEvent error " .. short(errQ)
  else
    tried[#tried + 1] = short(ev)
  end
  error("no route for the TPP representation (" .. table.concat(tried, "; ") .. ")", 0)
end

-- TppHead slot read by the bridge (no side effects) -> state, route; error when the bridge is not reachable
function M.RedsSlot(env)
  env = env or DEFAULT_ENV
  local G = env.Game
  local ok, s = pcall(function() return G.UTPPTppHeadSlotState() end)
  if ok and type(s) == "string" and s ~= "" then return s, "Game.UTPPTppHeadSlotState" end
  local ok2, s2 = pcall(function() return G.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):TppHeadSlotState() end)
  if ok2 and type(s2) == "string" and s2 ~= "" then return s2, "UTPPBodyBridge" end
  error("bridge not reachable (global " .. (ok and tostring(s) or ("error " .. short(s))) .. ", system "
    .. (ok2 and tostring(s2) or ("error " .. short(s2))) .. ") - utpp_body.reds installed?", 0)
end

-- ================================================================ in-game adapter (skipped by the offline tests)
M.wiring = { hashMethod = nil, pathMethod = nil, compsMethod = nil, errors = 0 }
local W = M.wiring

local function nameStr(n)
  if n == nil then return nil end
  if type(n) == "string" then return n end
  local ok, s = pcall(function() return Game.NameToString(n) end)
  if ok and type(s) == "string" then return s end
  ok, s = pcall(function() return n.value end)
  if ok and type(s) == "string" then return s end
  return tostring(n)
end

local MESH_CLASS = { entSkinnedMeshComponent = true, entGarmentSkinnedMeshComponent = true,
  entMorphTargetSkinnedMeshComponent = true, entMeshComponent = true, entPhysicalMeshComponent = true,
  entSkinnedClothComponent = true }

-- Optional reflected fields. CET/Codeware may omit them or throw on access; preserve that distinction
-- in diagnostics instead of turning an unavailable field into a misleading false value.
function M.ProbeRenderMetadata(component)
  local function field(read, isName)
    local ok, value = pcall(read)
    if not ok then return "unavailable:error" end
    if value == nil then return "unavailable:nil" end
    local okText, text = pcall(isName and nameStr or tostring, value)
    return okText and text or "unavailable:stringify"
  end
  return {
    castShadows = field(function() return component.castShadows end),
    castLocalShadows = field(function() return component.castLocalShadows end),
    skinningBindName = field(function() return component.skinning.bindName end, true),
    parentBindName = field(function() return component.parentTransform.bindName end, true),
    -- entSkinnedMeshComponent / entMorphTargetSkinnedMeshComponent (RED4ext.SDK SkinnedMeshComponent.hpp +0x238)
    renderPlaneParam = field(function() return component.renderingPlaneAnimationParam end, true),
  }
end

-- planes (optional) = M.PlaneIndex result: adds the native registry line of the component (M.PlaneNote).
function M.RenderMetaLine(c, planes)
  -- Normal component scans avoid reflective reads. Only a one-time unknown log or an explicit Dump
  -- requests them; an unknown log may probe its retained native component reference lazily.
  local meta = c.renderMeta or (c.ref and M.ProbeRenderMetadata(c.ref)) or {}
  return string.format(" castShadows=%s castLocalShadows=%s skinning.bindName=%s parentTransform.bindName=%s"
    .. " renderingPlaneAnimationParam=%s%s",
    tostring(meta.castShadows or "unavailable:adapter"), tostring(meta.castLocalShadows or "unavailable:adapter"),
    tostring(meta.skinningBindName or "unavailable:adapter"), tostring(meta.parentBindName or "unavailable:adapter"),
    tostring(meta.renderPlaneParam or "unavailable:adapter"), planes and M.PlaneNote(planes, c.name) or "")
end

-- ---------------------------------------------------------------- native render-plane registry (diagnostics)
-- Game.UTPPItemPlaneDump(max) (UTPP.dll, src\render\ItemPlane.cpp): first line = counters, then one line per registered
-- component (class, name, requested plane, current plane, owner is the player). The line format belongs to the native;
-- here a component is matched by its name as a whole token and the raw line is quoted, nothing is parsed.
M.PLANE_DUMP_MAX = 512
-- fn(max) -> text. -> text | nil, "unavailable:<why>" (never raises: missing native / old scripts / CET error)
function M.ReadPlaneDump(fn, max)
  if type(fn) ~= "function" then return nil, "unavailable:native" end
  local ok, text = pcall(fn, max or M.PLANE_DUMP_MAX)
  if not ok then return nil, "unavailable:error" end
  if type(text) ~= "string" then return nil, "unavailable:" .. type(text) end
  return text
end
-- -> { header = first line, lines = { component lines }, max = n, why = nil | "unavailable:<why>" }
function M.PlaneIndex(text, why, max)
  local idx = { lines = {}, max = max or M.PLANE_DUMP_MAX }
  if type(text) ~= "string" then idx.why = why or "unavailable:native"; return idx end
  for line in string.gmatch(text, "[^\r\n]+") do
    if idx.header == nil then idx.header = line else idx.lines[#idx.lines + 1] = line end
  end
  return idx
end
local PLANE_SEP = "[%s=\"'|,;:%(%)%[%]<>]"
local function hasToken(line, name)
  local init = 1
  while true do
    local s, e = string.find(line, name, init, true)
    if not s then return false end
    local before, after = string.sub(line, s - 1, s - 1), string.sub(line, e + 1, e + 1)
    if (s == 1 or string.find(before, PLANE_SEP)) and (after == "" or string.find(after, PLANE_SEP)) then return true end
    init = s + 1
  end
end
-- " itemPlane=[<registry line>]" | " itemPlane=not-registered" | " itemPlane=not-in-dump(max=<n>)" | " itemPlane=unavailable:*"
function M.PlaneNote(idx, name)
  if idx == nil then return "" end
  if idx.why then return " itemPlane=" .. idx.why end
  if name == nil or tostring(name) == "" then return " itemPlane=noname" end
  name = tostring(name)
  for _, line in ipairs(idx.lines) do
    if hasToken(line, name) then return " itemPlane=[" .. line .. "]" end
  end
  if #idx.lines >= idx.max then return string.format(" itemPlane=not-in-dump(max=%d)", idx.max) end
  return " itemPlane=not-registered"
end
-- api.planeDump(max) -> text (optional; GameApi: Game.UTPPItemPlaneDump)
function Body:PlaneIndex(max)
  max = max or M.PLANE_DUMP_MAX
  local text, why = M.ReadPlaneDump(self.api.planeDump, max)
  return M.PlaneIndex(text, why, max)
end
-- ---------------------------------------------------------------- render-plane diagnostics, read-only
-- Mesh depot path of a component for the unknown-part and Dump lines: the scan's own path (c.path) when it has one, else the
-- lazy api.meshPath(c) probe (GameApi; never in the per-scan hot path). " path=<depot path>" | " path=unresolved" (the route
-- answered without a path for this hash) | " path=unavailable:<why>" (no route worked) | "" (no probe in this api).
function M.PathNote(c, api)
  if c.path then return " path=" .. c.path end
  if api == nil or type(api.meshPath) ~= "function" then return "" end
  local ok, p, why = pcall(api.meshPath, c)
  if not ok then return " path=unavailable:error" end
  if type(p) == "string" and p ~= "" then return " path=" .. p end
  return " path=" .. tostring(why or "unresolved")
end
-- Slots always checked by the W lines (GameApi wornItems adds the AttachmentSlots.* / OutfitSlots.* TweakDB records): the
-- vanilla visual slots and the outfit-mod slots (absent records simply hold no item)
M.WORN_SLOTS = { "AttachmentSlots.Head", "AttachmentSlots.Eyes", "AttachmentSlots.Chest", "AttachmentSlots.Torso",
  "AttachmentSlots.Legs", "AttachmentSlots.Feet", "AttachmentSlots.Outfit", "AttachmentSlots.TppHead", "AttachmentSlots.RightArm",
  "AttachmentSlots.WeaponRight", "AttachmentSlots.WeaponLeft" }
for _, n in ipairs({ "AnkleLeft", "AnkleRight", "Back", "Balaclava", "BodyInner", "BodyMiddle", "BodyOuter", "BodyUnder", "EarLeft",
    "EarRight", "ElbowLeft", "ElbowRight", "EyeLeft", "EyeRight", "Eyes", "Feet", "FingernailsLeft", "FingernailsRight", "FingersLeft",
    "FingersRight", "Glasses", "HandLeft", "HandPropLeft", "HandPropRight", "HandRight", "Hands", "Head", "KneeLeft", "KneeRight",
    "LegsInner", "LegsMiddle", "LegsOuter", "Mask", "NecklaceLong", "NecklaceShort", "NecklaceTight", "Neckwear", "ShoulderLeft",
    "ShoulderRight", "ThighLeft", "ThighRight", "ToenailsLeft", "ToenailsRight", "ToesLeft", "ToesRight", "TorsoAux", "TorsoInner",
    "TorsoMiddle", "TorsoOuter", "TorsoUnder", "Waist", "Wreath", "WristLeft", "WristRight" }) do
  M.WORN_SLOTS[#M.WORN_SLOTS + 1] = "OutfitSlots." .. n
end
-- Appearance name that selects a first-person variant: "..._fpp" (vanilla FPP twin), "&FPP" (static) or the ArchiveXL dynamic
-- attribute "camera=fpp" (case-insensitive) -> 1, else 0; "?" when the name is unreadable
function M.FppVariant(app)
  if type(app) ~= "string" or app == "" then return "?" end
  local s = string.lower(app)
  if string.find(s, "_fpp", 1, true) or string.find(s, "&fpp", 1, true) or string.find(s, "camera=fpp", 1, true) then return 1 end
  return 0
end
-- w = { slot, item (TweakDB name), app } -> "W slot=<> item=<> app=<> fppVariant=<1|0|?>"
function M.WornLine(w)
  return string.format("W slot=%s item=%s app=%s fppVariant=%s", tostring(w.slot), tostring(w.item), tostring(w.app),
    tostring(M.FppVariant(w.app)))
end
-- Worn items of every occupied AttachmentSlots.* / OutfitSlots.* slot (src() -> list, info | error), one line each through
-- M.LogLine (CET console + UTPP log) after a header; -> the list | nil, why. No source = nothing printed (offline / no player).
function M.WornReport(src)
  if type(src) ~= "function" then return nil, "unavailable:source" end
  local ok, list, info = pcall(src)
  if not ok or type(list) ~= "table" then
    local why = "unavailable:" .. (ok and type(list) or "error")
    M.LogLine("[utpp_body] W " .. why .. (ok and "" or (" " .. short(list))))
    return nil, why
  end
  M.LogLine(string.format("[utpp_body] W items=%d %s", #list, tostring(info or "")))
  for _, w in ipairs(list) do M.LogLine("[utpp_body] " .. M.WornLine(w)) end
  return list
end
-- Console helper: print(GetMod("utpp_body").PlaneDump([max])) -> the dump text (or "unavailable:<why>"); every line is also
-- printed to the CET console (the native writes the same text to the UTPP log). Works without an attached body.
-- C3: then one W line per worn item (M.WornReport; worn = test source, default the GameApi's wornItems).
function M.PlaneDump(max, fn, worn)
  max = tonumber(max) or 64
  fn = fn or (M.body and M.body.api.planeDump) or function(n) return Game.UTPPItemPlaneDump(n) end
  worn = worn or (M.body and M.body.api.wornItems) or nil
  local text, why = M.ReadPlaneDump(fn, max)
  if text == nil then
    print("[utpp_body] itemPlane dump " .. why)
  else
    for line in string.gmatch(text, "[^\r\n]+") do print("[utpp_body] itemPlane " .. line) end
  end
  if worn then M.WornReport(worn) end
  return text or why
end

local function resHash(ref)
  if ref == nil then return nil end
  local tries = {
    { "codeware", function() return ResourceAsyncRef.GetHash(ref) end },
    { "field", function() return ref.hash end },
    { "tostring", function() return tostring(ref) end },
  }
  for _, t in ipairs(tries) do
    if W.hashMethod == nil or W.hashMethod == t[1] then
      local ok, v = pcall(t[2])
      local h = ok and M.NormHash(v) or nil
      if h then
        if W.hashMethod == nil then W.hashMethod = t[1]; M.LogLine("[utpp_body] mesh hash read via " .. t[1]) end
        return h
      end
    end
  end
  return nil
end

local function resPath(ref)
  if ref == nil or W.pathMethod == "none" then return nil end
  local ok, s = pcall(function() return ResRef.ToString(ResourceAsyncRef.GetPath(ref)) end)
  if ok and type(s) == "string" and string.find(s, "\\", 1, true) then W.pathMethod = "codeware"; return s end
  if W.pathMethod == nil then W.pathMethod = "none" end
  return nil
end

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

function M.LogLine(line)
  if M.core and M.core.sink then pcall(M.core.sink, line) return end
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
end

local function tdbStr(id)
  local ok, s = pcall(function() return TDBID.ToStringDEBUG(id) end)
  if ok and type(s) == "string" and s ~= "" then return s end
  return nil
end

function M.GameApi()
  local api = {}
  local function player() return Game.GetPlayer() end
  local slotTppHead = nil
  local headIds = nil

  api.log = function(msg) M.LogLine(msg) end
  api.playerKey = function()
    local p = player()
    if p == nil then return nil end
    local ok, h = pcall(function() return tostring(p:GetEntityID().hash) end)
    return ok and h or "player"
  end
  api.frozen = function()
    local p = player()
    if p == nil then return "noplayer" end
    local ok, photo = pcall(function() return Game.GetPhotoModeSystem():IsPhotoModeActive() end)
    if ok and photo then return "photomode" end
    return nil
  end
  api.gender = function()
    local ok, g = pcall(function() return nameStr(player():GetResolvedGenderName()) end)
    if ok and g == "Female" then return "female" elseif ok and g == "Male" then return "male" end
    return nil
  end
  local function tppHeadRecord()
    local g = api.gender()
    if g == "male" then return "Items.PlayerMaTppHead" elseif g == "female" then return "Items.PlayerWaTppHead" end
    return nil
  end
  -- ItemID of a record: CET ItemID.FromTDBID, else the gameItemID singleton
  local function itemId(rec)
    local tdb = TweakDBID.new(rec)
    local ok, id = pcall(function() return ItemID.FromTDBID(tdb) end)
    if ok and id ~= nil then return id, "ItemID.FromTDBID" end
    ok, id = pcall(function() return GetSingleton("gameItemID"):FromTDBID(tdb) end)
    if ok and id ~= nil then return id, "gameItemID:FromTDBID" end
    error("cannot build the ItemID of " .. rec)
  end
  M.ItemIdOf = itemId
  -- TppHead slot: "tpp" | "fpp" | "empty" | "spawning" (the TPP head of V's gender is being spawned) | "other"
  api.headSlot = function()
    slotTppHead = slotTppHead or TweakDBID.new("AttachmentSlots.TppHead")
    if headIds == nil then
      headIds = { fpp = TweakDBID.new("Items.PlayerFppHead"), wa = TweakDBID.new("Items.PlayerWaTppHead"),
                  ma = TweakDBID.new("Items.PlayerMaTppHead") }
    end
    local ts, p = Game.GetTransactionSystem(), player()
    local item = ts:GetItemInSlot(p, slotTppHead)
    local state = "other"
    if item == nil then state = "empty"
    else
      local tdb = ItemID.GetTDBID(item:GetItemID())
      local s = tdbStr(tdb)
      if s == "Items.PlayerFppHead" then state = "fpp"
      elseif s == "Items.PlayerWaTppHead" or s == "Items.PlayerMaTppHead" then state = "tpp"
      elseif s == nil then
        local function eq(a, b) return a.hash == b.hash and a.length == b.length end
        if eq(tdb, headIds.fpp) then state = "fpp" elseif eq(tdb, headIds.wa) or eq(tdb, headIds.ma) then state = "tpp" end
      end
    end
    if state == "fpp" or state == "empty" then
      -- vanilla TransactionSystem.IsSlotSpawningItem(obj, slotID, itemID): a TPP head already on its way
      local rec = tppHeadRecord()
      if rec ~= nil then
        local okS, spawning = pcall(function() return ts:IsSlotSpawningItem(p, slotTppHead, (itemId(rec))) end)
        if okS and spawning == true then return "spawning" end
      end
    end
    return state
  end
  api.repState = function()
    local s = api.headSlot()
    if s == "tpp" or s == "fpp" then return s end
    return "unknown"
  end
  -- head guard: the TPP head record of V's gender into AttachmentSlots.TppHead. Order as in the game's own code
  -- (inGameMenuGameController: TransactionSystem.GiveItem + AddItemToSlot), RemoveItemFromSlot(slot, true,
  -- true, true) first. an early version equipped "successfully" twice without any slot change; the result
  -- string now carries every step (ItemID method, inventory, remove, add, spawning, HasItemInSlot) to find out why.
  api.equipTppHead = function()
    local rec = tppHeadRecord()
    if rec == nil then return false, "gender unknown" end
    local ts, p = Game.GetTransactionSystem(), player()
    local slot = TweakDBID.new("AttachmentSlots.TppHead")
    local id, how = itemId(rec)
    -- our head item still on its way -> never remove / add again (that restarts or cancels the spawn)
    local okSp0, sp0 = pcall(function() return ts:IsSlotSpawningItem(p, slot, id) end)
    if okSp0 and sp0 == true then return true, rec .. " already spawning - not removed / added again" end
    local had = ts:HasItem(p, id)
    if not had then ts:GiveItem(p, id, 1) end
    local before = api.headSlot()
    local removed = "-"
    if before ~= "empty" and before ~= "spawning" then removed = tostring(ts:RemoveItemFromSlot(p, slot, true, true, true)) end
    local added = ts:AddItemToSlot(p, slot, id)
    local okSp, spawning = pcall(function() return ts:IsSlotSpawningItem(p, slot, id) end)
    local okIn, inSlot = pcall(function() return ts:HasItemInSlot(p, slot, id) end)
    return added ~= false, string.format("%s via %s, inventory %s, remove(%s)=%s, add=%s, spawning=%s, inSlot=%s", rec, how,
      had and "had it" or "given", before, removed, tostring(added), okSp and tostring(spawning) or "?",
      okIn and tostring(inSlot) or "?")
  end
  -- our TPP head item in AttachmentSlots.TppHead: "spawning" | "inslot" | "none" (vanilla natives
  -- IsSlotSpawningItem / HasItemInSlot, orphans.script) - the guard waits for the spawn instead of re-adding
  api.headItemState = function()
    local rec = tppHeadRecord()
    if rec == nil then return "unknown" end
    slotTppHead = slotTppHead or TweakDBID.new("AttachmentSlots.TppHead")
    local ts, p = Game.GetTransactionSystem(), player()
    local id = itemId(rec)
    if ts:IsSlotSpawningItem(p, slotTppHead, id) == true then return "spawning" end
    if ts:HasItemInSlot(p, slotTppHead, id) == true then return "inslot" end
    return "none"
  end
  -- hide / show the mesh components of the TPP head item (Items.Player?aTppHead, with its hair) in
  -- AttachmentSlots.TppHead; who = owner key ("guard" | "vehcam"): hidden while any owner holds it, exactly the
  -- components this call disabled are enabled again. Never touches an FPP head or another item.
  local headHidden, headHolders = nil, {}
  api.setHeadHidden = function(on, who)
    who = who or "api"
    if on then
      if headHidden ~= nil then headHolders[who] = true; return true, "already hidden" end
      if api.headSlot() ~= "tpp" then return false, "no TPP head in the slot" end
      slotTppHead = slotTppHead or TweakDBID.new("AttachmentSlots.TppHead")
      local item = Game.GetTransactionSystem():GetItemInSlot(player(), slotTppHead)
      if item == nil then return false, "no item object" end
      local list = {}
      for _, c in ipairs(item:GetComponents() or {}) do -- Codeware Entity.GetComponents()
        local okC, cls = pcall(function() return nameStr(c:GetClassName()) end)
        local okE, en = pcall(function() return c:IsEnabled() end)
        if okC and MESH_CLASS[cls] and okE and en == true and pcall(function() c:Toggle(false) end) then list[#list + 1] = c end
      end
      if #list == 0 then return false, "no enabled mesh component" end
      headHidden = list
      headHolders[who] = true
      return true, #list .. " mesh component(s) hidden"
    end
    headHolders[who] = nil
    if headHidden == nil then return true, "nothing hidden" end
    if next(headHolders) ~= nil then return true, "still held" end
    local n = 0
    for _, c in ipairs(headHidden) do if pcall(function() c:Toggle(true) end) then n = n + 1 end end
    headHidden = nil
    return true, n .. " mesh component(s) shown"
  end
  -- through the redscript bridge first (M.QueueRep), CET creation only as the last route
  api.activateRep = function() return M.QueueRep(true) end
  api.deactivateRep = function() return M.QueueRep(false) end
  -- head guard release: as vanilla TakeOverControlSystem.EnablePlayerTPPRepresenation(false) - never while the
  -- mounted vehicle's TPP camera owns the representation
  api.releaseRep = function()
    local p = player()
    local okV, tppVeh = pcall(function()
      local v = Game.GetMountedVehicle(p)
      return v ~= nil and v:GetCameraManager():IsTPPActive()
    end)
    if okV and tppVeh then return false, "vehicle TPP camera" end
    M.QueueRep(false)
    if api.feature then pcall(api.feature, false) end
    return true
  end
  -- through the redscript bridge first (scripts\utpp_body.reds UTPPApplyTppRepFeature), CET creation as before
  api.feature = function(on)
    on = on and true or false
    local okB, r = pcall(function() return Game.UTPPApplyTppRepFeature(on) end)
    if okB and r == true then return end
    local f = M.CreateNative(M.EVENT_NAMES.feature)
    f.IsActive = on
    AnimationControllerComponent.ApplyFeature(player(), "TPPRepresentation", f, 0)
  end
  -- wrapper variable utpp_foot_tpp on the player (scripts\utpp_body.reds UTPPSetFootTpp); false = no player
  api.footTpp = function(on)
    on = on and true or false
    local okG, r = pcall(function() return Game.UTPPSetFootTpp(on) end)
    if okG and r ~= nil then return r == true end
    return Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):SetFootTpp(on) == true
  end
  -- wrapper variable utpp_melee_tpp on the player (scripts\utpp_body.reds UTPPSetMeleeTpp); false = no player
  api.meleeTpp = function(on)
    on = on and true or false
    local okG, r = pcall(function() return Game.UTPPSetMeleeTpp(on) end)
    if okG and r ~= nil then return r == true end
    return Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):SetMeleeTpp(on) == true
  end
  -- exit keep latch + counters of the representation handlers (scripts\utpp_body.reds)
  api.setRepKeep = function(on, seconds)
    local okG, r = pcall(function() return Game.UTPPSetTppRepKeep(on and true or false, seconds or 0) end)
    if okG and r ~= nil then return r == true end
    return Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):SetTppRepKeep(on and true or false, seconds or 0) == true
  end
  -- native vehicle exit guard (UTPP.dll); true = hook live; an older DLL -> error -> false
  api.nativeGuard = function(on)
    local okG, r = pcall(function() return Game.UTPPSetVehicleExitRepGuard(on and true or false) end)
    return okG and r == true
  end
  api.repKeepDiag = function()
    local okG, d = pcall(function() return Game.UTPPTppRepKeepDiag() end)
    if okG and type(d) == "string" and d ~= "" then return d end
    return Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):TppRepKeepDiag()
  end
  api.gameOwnedReason = function()
    local p = player()
    local okV, tppVeh = pcall(function()
      local v = Game.GetMountedVehicle(p)
      return v ~= nil and v:GetCameraManager():IsTPPActive()
    end)
    if okV and tppVeh then return "vehicleTPP" end
    local okT, ctrl = pcall(function()
      return Game.GetScriptableSystemsContainer():Get("TakeOverControlSystem"):IsDeviceControlled()
    end)
    if okT and ctrl then return "takeover" end
    local okS, high = pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      return Game.GetBlackboardSystem():GetLocalInstanced(p:GetEntityID(), defs):GetInt(defs.HighLevel)
    end)
    if okS and type(high) == "number" and high >= 3 and high <= 5 then return "scene" end
    return nil
  end
  -- vehicle windows - PSM Vehicle (gamePSMVehicle: 0 Default, 1 Driving, 2 Combat, 3 Passenger,
  -- 4 Transition, 5 Turret, 6 DriverCombat, 7 Scene; orphans.script:6791) + Game.GetMountedVehicle (as utpp_vehiclecam)
  api.vehicle = function()
    local p = player()
    if p == nil then return nil end
    local out = { state = 0, mounted = false }
    pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      out.state = Game.GetBlackboardSystem():GetLocalInstanced(p:GetEntityID(), defs):GetInt(defs.Vehicle) or 0
    end)
    local okV, v = pcall(function() return Game.GetMountedVehicle(p) end)
    out.mounted = okV and v ~= nil
    return out
  end
  api.components = function(withRenderMeta)
    local list = player():GetComponents() -- Codeware Entity.GetComponents()
    local out = {}
    for _, c in ipairs(list or {}) do
      local okC, cls = pcall(function() return nameStr(c:GetClassName()) end)
      if okC and MESH_CLASS[cls] then
        local ref = nil
        if cls == "entMorphTargetSkinnedMeshComponent" then pcall(function() ref = c.morphResource end)
        else pcall(function() ref = c.mesh end) end
        local okN, name = pcall(function() return nameStr(c:GetName()) end)
        local okE, en = pcall(function() return c:IsEnabled() end)
        out[#out + 1] = { name = okN and name or nil, hash = resHash(ref), path = resPath(ref),
                          enabled = okE and en == true, ref = c, class = cls,
                          resourceField = cls == "entMorphTargetSkinnedMeshComponent" and "morphResource" or "mesh",
                          renderMeta = withRenderMeta and M.ProbeRenderMetadata(c) or nil }
      end
    end
    return out
  end
  api.setEnabled = function(comp, on) comp.ref:Toggle(on and true or false); return true end
  -- native render-plane registry (utpp.reds UTPPItemPlaneDump); callers guard it (M.ReadPlaneDump)
  api.planeDump = function(max) return Game.UTPPItemPlaneDump(max) end
  -- C3 (read-only, console diagnostics only): worn items of every occupied slot for M.PlaneDump's W lines. Slots =
  -- the AttachmentSlots.* / OutfitSlots.* records of TweakDB (OutfitSlots exist only with EquipmentEx) plus M.WORN_SLOTS (the
  -- vanilla clothing slots and the EquipmentEx outfit slots, in case the record enumeration is not available).
  api.wornItems = function()
    local names, seen, source = {}, {}, "fallback"
    local function add(n)
      if type(n) == "string" and not seen[n] and (string.find(n, "AttachmentSlots.", 1, true) == 1
          or string.find(n, "OutfitSlots.", 1, true) == 1) then
        seen[n] = true
        names[#names + 1] = n
      end
    end
    local okR, recs = pcall(function() return TweakDB:GetRecords("gamedataAttachmentSlot_Record") end)
    if okR and type(recs) == "table" then
      for _, r in ipairs(recs) do
        local okI, id = pcall(function() return r:GetID() end)
        if okI and id ~= nil then add(tdbStr(id)) end
      end
      if #names > 0 then source = "tweakdb" end
    end
    local listed = #names
    for _, n in ipairs(M.WORN_SLOTS) do add(n) end
    table.sort(names)
    local ts, p = Game.GetTransactionSystem(), player()
    local out = {}
    for _, n in ipairs(names) do
      pcall(function()
        local item = ts:GetItemInSlot(p, TweakDBID.new(n))
        if item == nil then return end
        local id = item:GetItemID()
        local okA, app = pcall(function() return nameStr(ts:GetItemAppearance(p, id)) end)
        local tdb = ItemID.GetTDBID(id)
        out[#out + 1] = { slot = n, item = tdbStr(tdb) or ("hash:" .. tostring(tdb.hash)), app = okA and app or nil }
      end)
    end
    return out, string.format("slots=%d (%s %d + list %d)", #names, source, listed, #names - listed)
  end
  -- C3: mesh depot path of a component for the unknown-part / Dump lines (lazy; resPath above stops trying after
  -- its first failure). Codeware ResRef.ToString resolves the hash through its path registry (paths the game or ArchiveXL
  -- loaded by name); -> path | nil, "unresolved" (a route answered without a path) | nil, "unavailable:<why>"
  api.meshPath = function(c)
    local ref = nil
    pcall(function()
      if c.class == "entMorphTargetSkinnedMeshComponent" then ref = c.ref.morphResource else ref = c.ref.mesh end
    end)
    local answered, errs = false, {}
    local routes = {
      { "ResourceAsyncRef.GetPath", function() return ResRef.ToString(ResourceAsyncRef.GetPath(ref)) end },
      -- the raw hash field (the read resHash uses in game: "mesh hash read via field")
      { "ResRef.FromHash(ref.hash)", function() return ResRef.ToString(ResRef.FromHash(ref.hash)) end },
      { "ResRef.FromHash", function()
        local h = c.hash and M.NormHash(c.hash)
        if not h then error("no hash") end
        return ResRef.ToString(ResRef.FromHash(loadstring("return " .. h .. "ULL")()))
      end },
    }
    for _, r in ipairs(routes) do
      local ok, s = pcall(r[2])
      if ok then
        answered = true
        if type(s) == "string" and s ~= "" and s ~= "None" then return s end
      else
        errs[#errs + 1] = r[1]
      end
    end
    if answered then return nil, "unresolved" end
    return nil, "unavailable:" .. table.concat(errs, "+")
  end
  M.GARMENT_SLOTS = M.GARMENT_SLOTS or { "AttachmentSlots.Legs", "AttachmentSlots.Feet" }
  api.garments = function()
    local ts = Game.GetTransactionSystem()
    local p = player()
    local out = {}
    for _, slot in ipairs(M.GARMENT_SLOTS) do
      pcall(function()
        local id = ts:GetActiveItemInSlot(p, TweakDBID.new(slot))
        if id == nil or not ItemID.IsValid(id) then return end
        local app = nameStr(ts:GetItemAppearance(p, id))
        if app == nil or app == "" or app == "None" then return end
        out[#out + 1] = { slot = slot, key = slot .. "|" .. tostring(tdbStr(ItemID.GetTDBID(id))), appearance = app, ref = id }
      end)
    end
    return out
  end
  api.setAppearance = function(g, name)
    Game.GetTransactionSystem():ChangeItemAppearanceByName(player(), g.ref, name)
    return true
  end
  api.setGarmentOffsets = function(on)
    if on then ArchiveXL.EnableGarmentOffsets() else ArchiveXL.DisableGarmentOffsets() end
    return true
  end
  return api
end

if type(registerForEvent) == "function" then
  local clock, announced = 0, false
  registerForEvent("onInit", function()
    if announced then return end
    announced = true
    local core = nil
    if type(GetMod) == "function" then
      local okC, c = pcall(GetMod, "utpp_core")
      if okC and type(c) == "table" then core = c end
    end
    local okR, errR = pcall(M.RegisterSettings, core)
    M.Attach(M.GameApi())
    M.LogLine(string.format("[utpp_body] onInit %s: settings %s, enabled=%s, headGuard=%s, data %s", M.VERSION,
      core and (okR and "via utpp_core" or ("FAILED " .. tostring(errR))) or "defaults (utpp_core missing)",
      tostring(M.Enabled()), tostring(M.HeadGuardEnabled()),
      M.data and (tostring(M.data.generator) .. ", " .. M.data.parts .. " parts") or ("MISSING: " .. tostring(M.dataError))))
  end)
  registerForEvent("onUpdate", function(delta)
    clock = clock + math.max(delta or 0, 0)
    if M.body == nil then return end
    M.clock = clock
    local ok, err = pcall(M.body.Step, M.body, clock)
    if not ok then
      W.errors = W.errors + 1
      if W.errors <= 3 then M.LogLine("[utpp_body] update error: " .. tostring(err)) end
    end
    local okV, errV = pcall(M.StepVehicle, clock)
    if not okV then
      W.errors = W.errors + 1
      if W.errors <= 3 then M.LogLine("[utpp_body] vehicle window error: " .. tostring(errV)) end
    end
    if M.guard then
      local okG, errG = pcall(M.guard.Step, M.guard, clock)
      if not okG then
        W.errors = W.errors + 1
        if W.errors <= 3 then M.LogLine("[utpp_body] head guard error: " .. tostring(errG)) end
      end
    end
    -- optional utpp_vehiclecam perspective hook, found at run time (GetMod exists only after load)
    if M.vcamHook.state == "pending" and type(GetMod) == "function" then
      local okH, errH = pcall(M.TryHookVehicleCam, clock, GetMod)
      if not okH then
        W.errors = W.errors + 1
        if W.errors <= 3 then M.LogLine("[utpp_body] vehicle camera hook error: " .. tostring(errH)) end
      end
    end
  end)
  registerForEvent("onShutdown", function()
    -- game exit: player gone -> the cached components / garments are dead objects, never touch them
    local okP, p = pcall(Game.GetPlayer)
    local alive = okP and p ~= nil and (pcall(function() return p:GetEntityID() end))
    if not alive then
      M.LogLine("[utpp_body] shutdown: player gone - game calls skipped")
      if M.body then pcall(M.body.ResetEntity, M.body) end
      return
    end
    if M.body then pcall(M.body.RestoreAll, M.body, clock, "shutdown") end
  end)
end

return M

-- UTPP utpp_perspective - automatic perspective decisions (third person TPP / first person FPP).
--
-- This module only DECIDES. It never switches a camera. Consumers (the UTPP camera module) read the decision:
--   local P = GetMod("utpp_perspective")
--   local token = P.Subscribe(function(decision, previous, mode) ... end)   -- called when decision.perspective changes
--   P.Desired()   -> { perspective = "TPP"|"FPP", tpp, reason, detail, rule, style = "instant"|"blend", blend (s),
--                      seq, t, restore, mode = "active"|"off", wantsTpp, locked, quick }
--   P.ToggleUser() / P.SetUserWantsTpp(bool) / P.SetQuickFpp(down) / P.Unsubscribe(token) / print(P.Status())
-- "FPP" means: the UTPP on-foot third-person camera is off and the game camera owns the view (first person, or the
-- vehicle / scene / device camera of the game).
--
-- Modes ("Use UTPP automatic perspective" in the UTPP tab):
--   active - UTPP drives: subscribers are notified;
--   off    - automatic rules off (switch off): only hard rules (outside the body, mirror, vehicle, device
--            takeover) and the user's choice.
--
-- Rules (highest priority first): outside body (Johnny, braindance, cyberspace; locks the toggle) > mirror > vehicle /
-- mounted turret > device takeover > scene > takedown / body carry > workspot > combat > weapon / aiming / gadget / arm
-- cyberware > unarmed zoom handoff (utpp_camera) > interaction (device zoom, terminal, elevator, forcing doors, personal link, breach) > ladders / swimming >
-- scanner > quick FPP hold > user preference. Each rule has an enter and an exit delay (latch with hysteresis); automatic
-- switches are at least minGap apart except urgent rules (cuts). Photo mode and menus freeze the decision (timers paused).
-- After a load the decision is held until the session is stable (10 polls or 12 s) and then re-emitted (restore).
--
-- Game API from the vanilla scripts.
--
-- Rules:
--   * vehicle enter / exit (PSM Vehicle Transition 4, or MountedToVehicle before a vehicle state) keeps third person like
--     utpp_vehiclecam keepTpp (17.6): entering from TPP stays TPP until a seat state (Driving 1, Passenger
--     3, ...) hands over to the game's vehicle camera; leaving a seat returns to the user's choice 0.4 s after the exit
--     starts. A transition stuck for vehicleKeepMax s (6) falls back to the game camera. Setting
--     "Third person on vehicle enter / exit" (vehicleKeepTpp), off = old behaviour (FPP for the transition);
--   * workspot ignored while in / mounted to a vehicle (the seat is a workspot: it delayed every exit by 1 s with a blend)
--     and latched only after workspotEnter s (0.6);
--   * weapons: default policy "aim" (TPP with a drawn weapon, user preference; was "drawn"), and
--     "aiming" (PSM UpperBody Aim) counts only with an item in WeaponRight (it was raised by dodge / slide with empty hands).
--
-- per-situation policies as settings (UTPP tab): perspective.combat TPP (default) /
--   FPP, perspective.weaponDrawn TPP (default) / FPP, perspective.aiming TPP (default, over-the-shoulder aiming) / FPP. The old
--   perspective.weapon (drawn / aim / never) is migrated into the new keys once (core "initial" value) and kept as a
--   legacy cfg field. Status() shows the active policies.
--
-- Scoped weapons keep their own aiming policy. A scope / sniper view cannot be
--   rendered over the shoulder, so with a scoped RANGED weapon in WeaponRight the aiming rule uses the setting
--   perspective.aimingScoped (fpp default | tpp) instead of perspective.aiming (gadgets / arm cyberware keep using aiming).
--   Scoped = (1) a REAL scope part in the weapon's AttachmentSlots.Scope (item:GetItemData():GetItemParts() -> InnerItemData
--   slot + item id, ItemID.IsValid, part ItemType Prt_Scope / ShortScope / LongScope / TechSniperScope / PowerSniperScope;
--   HasPartInSlot is NOT used: it is true for empty slots), (2) the weapon record ItemType is Wea_SniperRifle
--   (Wea_PrecisionRifle only with a real scope part), (3) stat ScopeFOV > 0 (zoomTransitions reads it per weapon; ignored
--   for pistols / revolvers / SMGs / shotguns / LMGs / HMGs). Reads are cached per
--   weapon (entity id) and refreshed when WeaponRight holds another item. One log line per change "aiming scoped
--   weapon=<record> -> fpp|tpp (<reason>)"; Status() shows scoped= and aimingScoped=.
--
-- Handles: ExitGame crashed (access violation right after the guarded onShutdown handlers) when game structs were held in Lua locals:
--   item data, InnerItemData parts, item records, TweakDBID userdata:
--   * HANDLES: nothing but plain Lua values (boolean / number / string) survives a poll. The scope cache holds
--     { key (entity hash string), slot, fov, part (item type name), partName (record name) }; item / item data / parts /
--     records / TweakDBIDs / blackboards are locals of readObs and are set to nil before it returns. onShutdown drops the
--     cache, stops the polling and runs a full GC while the engine can still release the handles; onUpdate without a
--     player also drops the cache;
--   * POLICY: a SHORT scope (Prt_ShortScope = red dot / 1.5x, and the generic Prt_Scope) keeps over-the-shoulder aiming;
--     only Prt_LongScope / Prt_TechSniperScope / Prt_PowerSniperScope (and Wea_SniperRifle) count as scoped. Setting
--     perspective.shortScopeFpp (default off) makes short scopes count as well. The cache holds the part TYPE, the
--     setting is applied on every poll (live). Reason text "scope slot (long)" / "scope slot (short)". The ScopeFOV rule
--     is not applied for weapons whose real scope part is a short scope (the FOV comes from that scope).
--
-- Melee block: the perspective must not switch to FPP while blocking with fists:
--   blocking with fists): holding "aim" with a MELEE weapon or FISTS equipped is a BLOCK, not aiming - the perspective stays
--   TPP (policy aiming=fpp applies to ranged weapons only). Melee = the item in WeaponRight is of a melee type (empty slot =
--   fists; gamedataItemType Wea_Fists, Wea_Katana, Wea_Knife, Wea_OneHandedClub, Wea_TwoHandedClub, Wea_Hammer, Wea_Machete,
--   Wea_Sword, Wea_Chainsword, Wea_Axe, Wea_LongBlade, Wea_ShortBlade, Wea_Melee, Cyb_MantisBlades, Cyb_StrongArms,
--   Cyb_NanoWires; weapon.script IsBlade / IsBlunt) OR PSM MeleeWeapon is a block / targeting state (5-10: Parried, Hold,
--   ChargedHold, Block, Targeting, Deflect; gamePSMMeleeWeapon). Either one turns the aiming rule off (gadgets and arm
--   cyberware are separate rules and unchanged). Setting perspective.meleeBlockTpp (default on, off = the earlier
--   behaviour); one log line per change "aiming ignored: melee block (...)". Status() shows meleeBlock=.
--
-- Unarmed zoom: rule "zoom" (reason "unarmed zoom", detail "camera handoff"):
--   utpp_camera zooms over the shoulder while RMB is held with empty hands; the wheel step past its last step asks
--   for first person so the game's own (Kiroshi) zoom is shown: GetMod("utpp_perspective").RequestFppZoom(on, source).
--   While requested the rule forces FPP (hard: also with the automatic rules off; urgent; instant both ways); cleared on
--   RMB release -> back to the previous decision (user TPP). A request is a user action (no minGap wait). The flag lives
--   in the engine state (st.zoomFpp; obs.zoomFpp works too) and is dropped by a session reset. Log "unarmed zoom FPP
--   request ON|OFF (<source>)" and the decision "FPP <- unarmed zoom (camera handoff) instant"; Status() zoomFpp=.
--
-- Vehicles. Nothing changes on foot:
--   * driver seat (PSM Vehicle 1) and driver combat (6): while utpp_camera holds the ride (obs.camHeld, the decision is
--     ignored there) and the game's vehicle camera is third person (obs.vehActive = VehicleCameraManager active
--     perspective 1-6, or obs.vehTpp = IsTPPActive when the enum is unreadable), the vehicle rule gives no first-person
--     reason: the decision is "TPP <- vehicle (vehicle:N (game TPP camera))" and masks the lower rules like the vehicle rule
--     did. "vehicle:N" (FPP) is returned when the game camera really is first person, when it is unknown, and whenever the
--     camera does not hold the ride (a TPP decision would make utpp_camera take the view from the vehicle camera then).
--     Desired() re-checks the hold live (M.liveCamHeld), so the frame in which the hold ends never sees that TPP;
--   * a return to TPP whose losing first-person rule was the vehicle rule that the camera hold ignored, while the hold is
--     still on, only updates the state (no notification, decision.switchSuppressed); log "perspective switch suppressed
--     (vehicle hold, rep owned)". Watchdog: if third person is not active (obs.tppActive: utpp_camera TPP signal and the
--     utpp_body representation) vehicleUnmountWatch s (0.2) after the unmount, the switch is issued (subscribers notified);
--   * perspective choices while mounted are for this ride only: perspective.userWantsTpp is never written while mounted,
--     and the choice from before the mount returns at the unmount (or a session reset);
--   * Game.UTPPSetVehicleTppAnim(bool) (optional native, pcall, nil-safe): in the driver seat it follows the view - false
--     when first person is chosen in the ride (game vehicle camera FPP, or an FPP decision when the camera is unreadable;
--     right after seating only when first person lasts VEH_ANIM_FPP_WAIT s, while the entry TPP request may be pending),
--     true when third person returns. Reset (no call) at the unmount.
local M = { VERSION = "0.2.7" }

local function copy(t) local r = {} for k, v in pairs(t) do r[k] = v end return r end
local function inRange(v, a, b) return type(v) == "number" and v >= a and v <= b end

-- ================================================================ pure decision engine
local P = {}
M.Engine = P

P.DEFAULTS = {
  auto = true,              -- false: only hard rules + the user's choice
  scene = true, sceneTier = 3, sceneExit = 1.5, sceneBlend = 0.6,
  vehicleKeepTpp = true, vehicleKeepMax = 6,
  vehicleUnmountWatch = 0.2, -- s after the unmount before a suppressed TPP switch is issued (third person not active)
  takedown = true,
  workspot = true, workspotEnter = 0.6, workspotMaxHold = 300,
  -- per-situation policy: "tpp" | "fpp". combat: FPP or TPP (default);
  -- weaponDrawn: a weapon in hand; aiming: aiming a weapon / throwing a gadget / arm cyberware (FPP for now).
  -- Legacy: boolean combat (true = fpp) and weapon = "drawn" (both fpp) | "aim" | "never" (aiming tpp) still work.
  combat = "tpp", weaponDrawn = "tpp", aiming = "tpp", aimingScoped = "fpp", shortScopeFpp = false, combatCut = true, combatExit = 1.25,
  interaction = true, interactionEnter = 0.1, interactionExit = 1.0,
  scanner = false, traversal = false,
  toggleOverrides = false,
  animate = true, blend = 0.8,
  minGap = 0.5,
  sessionGap = 0.75, settleTicks = 10, settleMax = 12,
  remember = true, defaultTpp = true, log = true,
  meleeBlockTpp = true,     -- aim with a melee weapon / fists = block, never FPP (see header)
}

-- policy helpers: true when the situation switches to FPP
local function isFpp(v) return v == "fpp" or v == true end
function P.CombatFpp(c) return isFpp(c.combat) end
function P.DrawnFpp(c) return isFpp(c.weaponDrawn) or c.weapon == "drawn" end
function P.AimingFpp(c)
  if c.weapon == "never" then return false end
  return not (c.aiming == "tpp" or c.aiming == false)
end
-- scoped weapons use aimingScoped (fpp unless "tpp" / false); legacy weapon = "never" does not touch it
function P.AimingScopedFpp(c)
  return not (c.aimingScoped == "tpp" or c.aimingScoped == false)
end
-- melee block: gamedataItemType names of melee weapons / fists / melee cyberware (weapon.script IsBlade / IsBlunt)
P.MELEE_TYPES = {
  Wea_Fists = true, Wea_Melee = true, Wea_Katana = true, Wea_Knife = true, Wea_OneHandedClub = true, Wea_TwoHandedClub = true,
  Wea_Hammer = true, Wea_Machete = true, Wea_Sword = true, Wea_Chainsword = true, Wea_Axe = true, Wea_LongBlade = true,
  Wea_ShortBlade = true, Cyb_MantisBlades = true, Cyb_StrongArms = true, Cyb_NanoWires = true,
}
-- Detail text when the melee weapon class or a PSM MeleeWeapon block / targeting state (5-10) is present (aiming itself is
-- not checked here), else nil. obs: weaponType (gamedataItemType name; "" = empty WeaponRight = fists; nil = unknown),
-- meleeWeapon (item tag MeleeWeapon, optional), meleeState (PSM MeleeWeapon int). Off with cfg.meleeBlockTpp == false.
function P.MeleeBlock(o, c)
  if not o or (c and c.meleeBlockTpp == false) then return nil end
  local parts = {}
  if o.weaponType == "" then parts[#parts + 1] = "fists"
  elseif type(o.weaponType) == "string" and P.MELEE_TYPES[o.weaponType] then parts[#parts + 1] = "weapon=" .. o.weaponType
  elseif o.meleeWeapon == true then parts[#parts + 1] = "melee weapon" end
  if inRange(o.meleeState, 5, 10) then parts[#parts + 1] = "PSM MeleeWeapon=" .. o.meleeState end
  if #parts == 0 then return nil end
  return table.concat(parts, ", ")
end
-- scope detection. HasPartInSlot(AttachmentSlots.Scope) is TRUE for weapons without a scope (Overture
-- iron sights, Kyubi) = it reports the slot, not an installed part. Now the real part of the Scope slot is resolved
-- (GetItemParts -> InnerItemData slot / item id, ItemID.IsValid) and its item type must be a scope (RPGManager.IsScopeAttachment).
-- Only Wea_SniperRifle counts by type (Wea_PrecisionRifle has iron sights: scoped only with a real scope part).
P.SCOPED_TYPES = { Wea_SniperRifle = true }
-- scope part type -> kind. "short" = red dot / 1.5x (over-the-shoulder aiming stays); "long" = real zoom. The generic
-- Prt_Scope is not documented in the 2.31 scripts (only listed next to the others in RPGManager.IsScopeAttachment) and
-- cannot be told apart offline: it is treated like a short scope (does not force FPP unless shortScopeFpp is on).
P.SCOPE_PART_TYPES = { Prt_Scope = "short", Prt_ShortScope = "short", Prt_LongScope = "long", Prt_TechSniperScope = "long",
                       Prt_PowerSniperScope = "long" }
-- ScopeFOV is ignored for weapon types that never carry a scope (sanity guard against a baseline stat value)
P.NO_SCOPE_FOV_TYPES = { Wea_Handgun = true, Wea_Revolver = true, Wea_SubmachineGun = true, Wea_Shotgun = true,
                         Wea_ShotgunDual = true, Wea_LightMachineGun = true, Wea_HeavyMachineGun = true }
-- "short" | "long" for a scope part type name, else nil
function P.ScopeKind(t) return type(t) == "string" and P.SCOPE_PART_TYPES[t] or nil end
-- true when the part counts as "scoped": long scopes always, short scopes only with shortFpp == true (perspective.shortScopeFpp)
function P.IsScopePartType(t, shortFpp)
  local k = P.ScopeKind(t)
  return k == "long" or (k == "short" and shortFpp == true)
end
-- Reason text when the weapon in WeaponRight is a scoped RANGED weapon, else nil. obs: scopeSlot (bool, a REAL scope part
-- is attached to AttachmentSlots.Scope), weaponType (as in MeleeBlock; "" = fists, never scoped), scopeFov (stat ScopeFOV).
-- scopeKind ("long" | "short", the kind of that part; default long). Order of the reasons: scope slot ("scope slot (long)" /
-- "(short)"), sniper type, ScopeFOV (not with a real short scope part: scopeKind "short" without scopeSlot = a short scope
-- the policy does not count, its ScopeFOV is not a reason). Melee types are never scoped.
function P.Scoped(o)
  if not o or o.weaponType == "" then return nil end
  if type(o.weaponType) == "string" and P.MELEE_TYPES[o.weaponType] then return nil end
  if o.scopeSlot == true then return "scope slot (" .. (o.scopeKind == "short" and "short" or "long") .. ")" end
  if type(o.weaponType) == "string" and P.SCOPED_TYPES[o.weaponType] then return "sniper type" end
  if type(o.scopeFov) == "number" and o.scopeFov > 0 and o.scopeKind ~= "short"
      and not (type(o.weaponType) == "string" and P.NO_SCOPE_FOV_TYPES[o.weaponType]) then
    return "ScopeFOV"
  end
  return nil
end
-- Aiming policy for the current observation: scoped weapon -> aimingScoped, else aiming. Returns fpp (bool), scoped reason.
function P.AimFppFor(o, c)
  local sc = P.Scoped(o)
  if sc then return P.AimingScopedFpp(c), sc end
  return P.AimingFpp(c), nil
end
function P.Policies(c)
  local function n(b) return b and "FPP" or "TPP" end
  return string.format("combat=%s weaponDrawn=%s aiming=%s", n(P.CombatFpp(c)), n(P.DrawnFpp(c)), n(P.AimingFpp(c)))
end

-- raw(obs, cfg) -> detail string when the situation is present, else nil/false.
-- style: "instant" | "blend" | "combat" (instant when cfg.combatCut, else blend). exitStyle: style of the return.
P.RULES = {
  { id = "outside", hard = true, lock = true, urgent = true, style = "instant", enter = 0, exit = 0.3,
    raw = function(o)
      if o.replacer then return "replacer" elseif o.braindance then return "braindance"
      elseif o.cyberspace then return "cyberspace" end
    end },
  { id = "mirror", hard = true, urgent = true, style = "instant", enter = 0, exit = 0.5,
    raw = function(o) return o.mirror and "mirror" end },
  { id = "vehicle", hard = true, urgent = true, style = "instant", exitStyle = "instant", enter = 0, exit = 0.4,
    raw = function(o, c, st)
      local v = o.vehicle or 0
      if v == 5 then return "turret" end
      -- enter / exit transition: kept in third person (rule vehicle_hold) when allowed, else the game camera at once
      if P.VehicleTransition(o) and st and st.vehHold then return nil end
      -- driver seat / driver combat held by utpp_camera with the game's vehicle camera in third person
      if P.VehicleGameTpp(o) then return nil end
      if v ~= 0 then return "vehicle:" .. v elseif o.mounted then return "mounted" end
    end },
  -- keepTpp: an enter / exit transition that started in TPP (entering) or from a seat while the user wants TPP
  -- (exiting) stays TPP; only a transition stuck for vehicleKeepMax s hands over to the game camera.
  { id = "vehicle_hold", reason = "vehicle", hard = true, urgent = true, style = "instant", exitStyle = "instant",
    enterKey = "vehicleKeepMax", exit = 0,
    raw = function(o, c, st)
      if st and st.vehHold and P.VehicleTransition(o) then
        return ((o.vehicle or 0) == 4 and "transition:4" or "mounted") .. " (kept " .. st.vehHold .. ")"
      end
    end },
  { id = "takeover", hard = true, urgent = true, style = "instant", exitStyle = "instant", enter = 0, exit = 0.4,
    raw = function(o) return o.controllingDevice and "device control" end },
  { id = "scene", style = "blend", blendKey = "sceneBlend", enter = 0, exitKey = "sceneExit",
    raw = function(o, c)
      if c.scene and inRange(o.high, c.sceneTier, 5) then return "SceneTier" .. o.high end
    end },
  { id = "takedown", urgent = true, style = "instant", enter = 0, exit = 0.8,
    raw = function(o, c)
      if not c.takedown then return nil end
      if (o.takedown or 0) ~= 0 then return "takedown:" .. o.takedown end
      if (o.carry or 0) ~= 0 then return "carry:" .. o.carry end
    end },
  -- a vehicle seat is a workspot too (PSM IsInWorkspot while mounted) - ignored there; enter delay = exit grace
  { id = "workspot", style = "blend", enterKey = "workspotEnter", exit = 1.0, maxHoldKey = "workspotMaxHold",
    overridable = true,
    raw = function(o, c)
      if (o.vehicle or 0) ~= 0 or o.mounted then return nil end
      return c.workspot and o.workspot and "workspot"
    end },
  { id = "combat", style = "combat", enter = 0, exitKey = "combatExit", overridable = true,
    raw = function(o, c) return P.CombatFpp(c) and o.combat == 1 and "InCombat" end },
  { id = "weapon", style = "combat", enter = 0, exitKey = "combatExit", overridable = true,
    raw = function(o, c)
      if P.DrawnFpp(c) and o.weaponDrawn then return "weapon drawn" end
      -- UpperBody Aim without an item in the hand (dodge / slide) is not aiming a weapon
      -- aim with a melee weapon / fists is a block, not aiming (setting meleeBlockTpp)
      -- aiming a scoped ranged weapon follows aimingScoped instead of aiming
      local aimFpp = P.AimingFpp(c)
      if o.aiming and o.weaponDrawn and not P.MeleeBlock(o, c) then
        local sc = P.Scoped(o)
        if sc then
          if P.AimingScopedFpp(c) then return "aiming (scoped: " .. sc .. ")" end
        elseif aimFpp then
          return "aiming"
        end
      end
      if not aimFpp then return nil end
      if inRange(o.gadget, 1, 6) then return "gadget" end
      if inRange(o.lhcw, 2, 10) then return "arm cyberware" end
    end },
  -- first person handoff of utpp_camera's unarmed zoom (RequestFppZoom); not overridable, never a lock
  { id = "zoom", reason = "unarmed zoom", hard = true, urgent = true, style = "instant", exitStyle = "instant", enter = 0,
    exit = 0,
    raw = function(o, c, st) if (st and st.zoomFpp) or o.zoomFpp then return "camera handoff" end end },
  { id = "interaction", style = "blend", enterKey = "interactionEnter", exitKey = "interactionExit", overridable = true,
    raw = function(o, c)
      if not c.interaction then return nil end
      if o.uiZoom then return "device zoom" elseif o.terminal then return "terminal" elseif o.elevator then return "elevator"
      elseif o.forceDoor then return "forcing door" elseif o.personalLink then return "personal link"
      elseif o.minigame then return "breach" end
    end },
  { id = "traversal", style = "blend", enter = 0.2, exit = 0.8, overridable = true,
    raw = function(o, c)
      if not c.traversal then return nil end
      if (o.swim or 0) ~= 0 or o.high == 6 then return "swimming" end
      if inRange(o.loco, 10, 13) then return "ladder" end
    end },
  { id = "scanner", style = "instant", exitStyle = "instant", enter = 0, exit = 0, overridable = true,
    raw = function(o, c) return c.scanner and o.vision == 1 and "scanner" end },
}
P.byId = {}
for i, r in ipairs(P.RULES) do r.priority = i; P.byId[r.id] = r end

-- entering / leaving a vehicle: PSM Vehicle Transition (4), or MountedToVehicle before any vehicle state
function P.VehicleTransition(o)
  local v = o.vehicle or 0
  return v == 4 or (v == 0 and o.mounted == true)
end

-- the game's vehicle camera (VehicleCameraManager active perspective: 0 = FPP, 1-3 TPP, 4-6 DriverCombat TPP; read by
-- the wiring in vehicle states 1 / 6): true = third person, false = first person, nil = unknown. obs.vehActive (number) first,
-- obs.vehTpp (IsTPPActive) when the enum could not be converted.
function P.VehicleCamTpp(o)
  if not o then return nil end
  if type(o.vehActive) == "number" then return o.vehActive ~= 0 end
  if type(o.vehTpp) == "boolean" then return o.vehTpp end
  return nil
end
-- detail text when the driver seat (1) / driver combat (6) is shown by the game's third-person vehicle camera while
-- utpp_camera holds the ride (obs.camHeld == true), else nil. Without the hold the vehicle rule is unchanged (see header).
function P.VehicleGameTpp(o)
  local v = o and o.vehicle or 0
  if (v == 1 or v == 6) and o.camHeld == true and P.VehicleCamTpp(o) == true then
    return "vehicle:" .. v .. " (game TPP camera)"
  end
  return nil
end

-- mounted = PSM Vehicle ~= 0 or MountedToVehicle. On the mount edge the user's choice is saved; on the unmount edge it
-- returns (choices made while mounted are for that ride only) and the watchdog clock starts.
local function updateMount(st, now, o)
  local m = (o.vehicle or 0) ~= 0 or o.mounted == true
  if m and not st.vehMounted then
    st.vehMounted, st.vehIntent, st.unmountAt = true, st.wantsTpp, nil
  elseif not m and st.vehMounted then
    st.vehMounted, st.unmountAt = false, now
    local intent = st.vehIntent
    st.vehIntent = nil
    if intent ~= nil and intent ~= st.wantsTpp then
      st.intentRestored = { from = st.wantsTpp, to = intent }
      st.wantsTpp = intent
    end
  end
end

-- vehicle hold (keepTpp). st.vehHold = "enter" | "exit" | nil, decided once at the start of each transition:
-- entering from on foot keeps TPP only when the current decision is TPP (FPP forced by a rule or chosen by the user is
-- respected, like utpp_vehiclecam); leaving a seat returns to the user's choice (other rules still apply by priority).
-- review fix: the exit follows the choice that holds after the unmount (the one from before the mount, see updateMount),
-- so a toggle made in the ride never decides the exit only to be undone at the unmount (double switch). Not mounted or
-- no stored choice: st.wantsTpp.
function P.ExitWantsTpp(st)
  if st.vehMounted and st.vehIntent ~= nil then return st.vehIntent end
  return st.wantsTpp
end
local function updateVehicleHold(st, now, o)
  if P.VehicleTransition(o) then
    if st.vehTransAt == nil then
      st.vehTransAt, st.vehHold = now, nil
      if st.cfg.vehicleKeepTpp ~= false then
        if st.vehSeat then
          if P.ExitWantsTpp(st) and not st.quick then st.vehHold = "exit" end
        elseif st.current.tpp then
          st.vehHold = "enter"
        end
      end
    end
  else
    st.vehTransAt, st.vehHold = nil, nil
    st.vehSeat = (o.vehicle or 0) ~= 0
  end
end

local function decision(tpp, reason, extra)
  local d = { perspective = tpp and "TPP" or "FPP", tpp = tpp and true or false, reason = reason, style = "instant", blend = 0 }
  if extra then for k, v in pairs(extra) do d[k] = v end end
  return d
end

function P.New(cfg, opts)
  local st = { cfg = cfg or copy(P.DEFAULTS), rules = {}, wantsTpp = true, quick = false, suppressed = {},
               current = decision(false, "init", { seq = 0, t = 0 }), lastSwitch = -1e9, seq = 0,
               session = "init", gap = 0, settle = 0, settleStart = nil, frozenAt = nil, userEvent = false,
               restore = false, locked = false, now = nil }
  for _, r in ipairs(P.RULES) do st.rules[r.id] = { latched = false } end
  if opts and opts.ready then st.session = "ready" end
  if opts and opts.wantsTpp ~= nil then st.wantsTpp = opts.wantsTpp and true or false end
  return st
end

function P.ResetSession(st)
  for _, r in ipairs(P.RULES) do st.rules[r.id] = { latched = false } end
  st.suppressed, st.quick, st.locked = {}, false, false
  st.settle, st.settleStart, st.frozenAt, st.restore = 0, nil, nil, false
  st.vehTransAt, st.vehHold, st.vehSeat = nil, nil, false
  st.zoomFpp = false -- the camera's unarmed zoom request does not survive a load / death / fast travel
  -- a load while mounted ends the ride: the choice from before the mount returns, the suppressed switch is dropped
  if st.vehIntent ~= nil then st.wantsTpp = st.vehIntent end
  st.vehMounted, st.vehIntent, st.unmountAt, st.switchOwed, st.vehFppIgnored, st.vehGameTpp = false, nil, nil, nil, nil, nil
end

local function cfgNum(c, key, fallback)
  if key and type(c[key]) == "number" then return c[key] end
  return fallback or 0
end

-- latch with hysteresis: raw must hold for `enter` s to latch and be absent for `exit` s to release;
-- maxHold (s) releases a latch whose raw condition stays on too long (stuck states) until raw clears once.
function P.Latch(st, rule, now, obs)
  local c, r = st.cfg, st.rules[rule.id]
  local detail = nil
  if rule.hard or c.auto ~= false then detail = rule.raw(obs, c, st) end
  if detail then
    r.detail, r.clear = detail, nil
    if not r.on then r.on = now end
    local maxHold = rule.maxHoldKey and c[rule.maxHoldKey] or nil
    if r.expired then
      r.latched = false
    elseif maxHold and maxHold > 0 and now - r.on >= maxHold then
      r.latched, r.expired = false, true
    elseif not r.latched and now - r.on >= cfgNum(c, rule.enterKey, rule.enter) then
      r.latched = true
    end
  else
    r.on, r.expired = nil, false
    if r.latched then
      r.clear = r.clear or now
      if now - r.clear >= cfgNum(c, rule.exitKey, rule.exit) then r.latched, r.clear = false, nil end
    end
  end
  if not r.latched then st.suppressed[rule.id] = nil end
  return r.latched
end

local function winner(st)
  for _, rule in ipairs(P.RULES) do
    if st.rules[rule.id].latched and not st.suppressed[rule.id] then return rule end
  end
  return nil
end

local function styleOf(st, target, rule, prev)
  local c = st.cfg
  local s, blend = "blend", c.blend
  if st.restore then return "instant", 0 end
  if not target.tpp then
    if rule then
      s = rule.style
      if s == "combat" then s = c.combatCut and "instant" or "blend" end
      if rule.blendKey then blend = cfgNum(c, rule.blendKey, c.blend) end
    elseif target.reason == "quick_fpp" then
      s = "instant"
    end
  else
    local pr = prev.rule and P.byId[prev.rule] or nil
    if prev.reason == "quick_fpp" or (pr and pr.exitStyle == "instant") then s = "instant" end
  end
  if s == "blend" and not c.animate then s = "instant" end
  if s == "instant" then blend = 0 end
  return s, blend
end

-- obs: { player, menu, photo, loading, vehicle, mounted, high, combat, swim, workspot, takedown, carry, loco, vision,
--        aiming, weaponDrawn, gadget, lhcw, uiZoom, controllingDevice, elevator, forceDoor, personalLink, minigame,
--        terminal, replacer, braindance, cyberspace, mirror }
-- returns decision, changed (perspective changed or session restore -> consumers must (re)apply)
function P.Step(st, now, obs)
  local c = st.cfg
  local dt = st.now and math.max(0, now - st.now) or 0
  st.now = now
  obs = obs or {}
  -- session (load / death / fast travel: the player disappears)
  if not obs.player then
    st.gap = st.gap + dt
    if st.session ~= "lost" and st.gap >= c.sessionGap then
      P.ResetSession(st)
      st.session = "lost"
    end
    return st.current, false
  end
  st.gap = 0
  if st.session ~= "ready" then
    st.settleStart = st.settleStart or now
    if not obs.menu and not obs.loading then st.settle = st.settle + 1 end
    if st.settle >= c.settleTicks or now - st.settleStart >= c.settleMax then
      st.session, st.restore = "ready", true
    else
      return st.current, false
    end
  end
  -- freeze (photo mode, menus): decision kept, timers paused
  local frozen = (obs.photo and "photo") or (obs.menu and "menu") or nil
  if frozen then
    st.frozenAt = st.frozenAt or now
    st.current.frozen = frozen
    st.userEvent = false
    return st.current, false
  end
  if st.frozenAt then
    local shift = now - st.frozenAt
    for _, r in pairs(st.rules) do
      if r.on then r.on = r.on + shift end
      if r.clear then r.clear = r.clear + shift end
    end
    st.lastSwitch = st.lastSwitch + shift
    st.frozenAt = nil
  end
  st.current.frozen = nil
  updateVehicleHold(st, now, obs)
  updateMount(st, now, obs)
  st.vehGameTpp = P.VehicleGameTpp(obs)
  for _, rule in ipairs(P.RULES) do P.Latch(st, rule, now, obs) end
  if st.vehGameTpp then -- the game's TPP vehicle camera shows the seat now: the vehicle latch ends at once
    local r = st.rules.vehicle
    r.latched, r.clear, r.on = false, nil, nil
  end
  local rule = winner(st)
  -- below the vehicle rule nothing may force first person while the game's TPP vehicle camera shows the seat (the
  -- vehicle rule masked them before, they still apply after the ride)
  if st.vehGameTpp and rule and rule.priority > P.byId.vehicle.priority then rule = nil end
  st.locked = rule ~= nil and rule.lock == true
  local target
  if rule then
    target = decision(false, rule.reason or rule.id, { rule = rule.id, detail = st.rules[rule.id].detail })
  elseif st.vehGameTpp then
    target = decision(true, "vehicle", { detail = st.vehGameTpp, vehView = obs.vehicle })
  elseif st.quick then
    target = decision(false, "quick_fpp")
  else
    -- mounted (enter / exit hold): the choice from before the mount, the one that also holds after the unmount
    target = decision(P.ExitWantsTpp(st), "user")
  end
  local cur = st.current
  -- the current first person comes from the vehicle rule while utpp_camera held the vehicle (ignored there)
  if not cur.tpp and cur.rule == "vehicle" then
    if obs.camHeld == true then st.vehFppIgnored = true end
  else
    st.vehFppIgnored = nil
  end
  local switching = target.tpp ~= cur.tpp
  if switching and not st.userEvent and not st.restore and now - st.lastSwitch < c.minGap then
    local urgent = (rule and (rule.urgent or (rule.style == "combat" and c.combatCut))) or target.vehView ~= nil
    if not urgent then
      cur.pending = target.reason
      st.userEvent = false
      return cur, false
    end
  end
  local changed = switching or st.restore
  -- back to TPP while the hold that ignored the vehicle FPP is still on: state only, no switch
  local suppressed = switching and target.tpp and st.vehFppIgnored == true and obs.camHeld == true and not st.restore
  if suppressed then changed = false end
  if switching or st.restore or target.reason ~= cur.reason or target.detail ~= cur.detail then
    if switching or st.restore then
      target.style, target.blend = styleOf(st, target, rule, cur)
      st.lastSwitch = now
    else
      target.style, target.blend = cur.style, cur.blend
    end
    if suppressed then
      target.switchSuppressed, st.switchOwed = true, true
    elseif switching then
      st.switchOwed = nil
    end
    st.seq = st.seq + 1
    target.seq, target.t, target.restore = st.seq, now, st.restore or nil
    st.current = target
  else
    cur.pending = nil
  end
  -- watchdog: a suppressed TPP switch is issued when third person is not active vehicleUnmountWatch s after the unmount
  st.watch = nil
  if st.switchOwed and st.unmountAt and now - st.unmountAt >= cfgNum(c, "vehicleUnmountWatch", 0.2) - 1e-6 then
    st.switchOwed = nil
    if st.current.tpp and not changed then
      if obs.tppActive ~= true then
        changed, st.watch = true, "issued"
      else
        st.watch = "ok"
      end
    end
  end
  st.restore, st.userEvent = false, false
  return st.current, changed
end

-- toggle what the user sees. Returns ok, note.
function P.Toggle(st)
  local cur = st.current
  if st.locked then return false, "locked:" .. tostring(cur.rule) end
  st.userEvent = true
  if cur.tpp then
    st.wantsTpp = false
    return true, "FPP"
  end
  if cur.rule then
    local rule = P.byId[cur.rule]
    if st.cfg.toggleOverrides and rule and rule.overridable then
      st.wantsTpp = true
      for _, r in ipairs(P.RULES) do
        if r.overridable and st.rules[r.id].latched then st.suppressed[r.id] = true end
      end
      return true, "override:" .. cur.rule
    end
    st.wantsTpp = not st.wantsTpp
    return true, (st.wantsTpp and "deferred:" or "deferred-cancelled:") .. cur.rule
  end
  if cur.reason == "quick_fpp" then return false, "quick_fpp held" end
  st.wantsTpp = true
  return true, "TPP"
end

function P.SetQuick(st, down)
  down = down and true or false
  if st.quick == down then return false end
  st.quick, st.userEvent = down, true
  return true
end

-- ================================================================ module state, API
M.cfg = copy(P.DEFAULTS)
M.st = P.New(M.cfg)
M.listeners = {}
M.history = {}
M.logTail = {}
M.core = nil
M.mode = "active"
M.forceMode = nil      -- tests / console: "active" | "off"
M.stats = { steps = 0, changes = 0, notified = 0, listenerErrors = 0, toggles = 0 }
local nextToken = 0

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

function M.Log(msg)
  if M.cfg.log == false then return end
  local line = "[utpp_perspective] " .. tostring(msg)
  M.logTail[#M.logTail + 1] = line
  if #M.logTail > 80 then table.remove(M.logTail, 1) end
  if M.sink then M.sink(line) return end
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
end

function M.Mode()
  if M.forceMode then return M.forceMode end
  local core = M.core
  if core then
    local okE, eff = pcall(core.Effective, "perspective.impl")
    if okE and eff == true then return "active" end
    return "off"
  end
  return "active"
end

function M.Subscribe(fn)
  assert(type(fn) == "function", "utpp_perspective.Subscribe: function expected")
  nextToken = nextToken + 1
  M.listeners[#M.listeners + 1] = { token = nextToken, fn = fn }
  return nextToken
end

function M.Unsubscribe(token)
  for i, l in ipairs(M.listeners) do
    if l.token == token then table.remove(M.listeners, i) return true end
  end
  return false
end

function M.Desired()
  local d = copy(M.st.current)
  -- the TPP of the game's vehicle camera is valid only while utpp_camera holds the ride; the hold is checked live, so
  -- the frame in which it ends (before the next poll) reads the vehicle first person, never a TPP the camera would take
  if d.vehView ~= nil and M.liveCamHeld then
    local okH, held = pcall(M.liveCamHeld)
    if okH and held == false then
      d.perspective, d.tpp, d.reason, d.rule, d.detail = "FPP", false, "vehicle", "vehicle", "vehicle:" .. tostring(d.vehView)
      d.style, d.blend, d.vehView, d.live = "instant", 0, nil, "camera hold ended"
    end
  end
  d.mode = M.mode
  -- wantsTpp = the choice for the exit / after the ride (read by utpp_camera VehicleExitWanted); rideWantsTpp = in memory
  d.wantsTpp, d.locked, d.quick = P.ExitWantsTpp(M.st), M.st.locked, M.st.quick
  d.rideWantsTpp = M.st.wantsTpp
  return d
end

local function notify(dec, prev)
  for _, l in ipairs(M.listeners) do
    local d = copy(dec)
    d.mode = M.mode
    local ok, err = pcall(l.fn, d, copy(prev), M.mode)
    M.stats.notified = M.stats.notified + 1
    if not ok then
      M.stats.listenerErrors = M.stats.listenerErrors + 1
      if M.stats.listenerErrors <= 5 then M.Log("listener error: " .. tostring(err)) end
    end
  end
end

-- driver-seat animation set: Game.UTPPSetVehicleTppAnim(on) (optional native; a missing one is logged once).
-- M.vehAnim = last value asked for in this ride (nil = none yet), reset at the unmount (the script side clears it there).
M.VEH_ANIM_FPP_WAIT = 1.0
local function setVehicleAnim(on, why)
  M.vehAnim = on
  M.stats.vehAnimCalls = (M.stats.vehAnimCalls or 0) + 1
  local ok, res = pcall(function() return Game.UTPPSetVehicleTppAnim(on) end)
  if ok and res ~= false then
    M.Log(string.format("vehicle TPP animation -> %s (%s)", on and "on" or "off", why))
  elseif not M.vehAnimMissing then
    M.vehAnimMissing = true
    M.Log("vehicle TPP animation: Game.UTPPSetVehicleTppAnim unavailable (" .. tostring(res) .. ")")
  end
end
function M.VehicleAnimStep(now, o, dec)
  local v = (o and o.player and o.vehicle) or 0
  if not (o and o.player and (v ~= 0 or o.mounted == true)) then
    M.vehAnim, M.vehAnimFppSince = nil, nil
    return
  end
  if v ~= 1 and v ~= 6 then return end
  local view = P.VehicleCamTpp(o)
  if view == nil then view = dec ~= nil and dec.tpp == true end
  if view then
    M.vehAnimFppSince = nil
    if M.vehAnim ~= true then setVehicleAnim(true, "third person in the driver seat") end
  elseif M.vehAnim == nil then
    -- right after seating the entry's TPP request may still be pending: first person must last VEH_ANIM_FPP_WAIT s
    M.vehAnimFppSince = M.vehAnimFppSince or now
    if now - M.vehAnimFppSince >= M.VEH_ANIM_FPP_WAIT - 1e-6 then
      setVehicleAnim(false, string.format("first person %.1f s after seating", M.VEH_ANIM_FPP_WAIT))
    end
  elseif M.vehAnim ~= false then
    setVehicleAnim(false, "first person chosen in the ride")
  end
end

-- now (s), obs (see P.Step)
function M.Update(now, obs)
  M.mode = M.Mode()
  M.cfg.auto = M.mode ~= "off"
  M.lastNow, M.lastObs = now, obs
  -- one log line per change of the melee block decision (aim held with melee / fists while the aiming policy is FPP)
  local mb = obs and obs.player and obs.aiming and P.AimingFpp(M.cfg) and P.MeleeBlock(obs, M.cfg) or nil
  if mb ~= M.meleeBlockKey then
    M.meleeBlockKey = mb
    if mb then M.Log(string.format("t=%.2f aiming ignored: melee block (%s)", now, mb)) end
  end
  -- one log line per change of the scoped-weapon aiming decision (aim held with a scoped ranged weapon)
  local sk = nil
  if obs and obs.player and obs.aiming and obs.weaponDrawn and not P.MeleeBlock(obs, M.cfg) then
    local fpp, sc = P.AimFppFor(obs, M.cfg)
    if sc then
      sk = string.format("weapon=%s -> %s (%s)", tostring(obs.weaponRecord or obs.weaponType or "?"), fpp and "fpp" or "tpp", sc)
    end
  end
  if sk ~= M.scopedKey then
    M.scopedKey = sk
    if sk then M.Log("aiming scoped " .. sk) end
  end
  local prev = M.st.current
  local dec, changed = P.Step(M.st, now, obs)
  M.stats.steps = M.stats.steps + 1
  local st = M.st
  if st.intentRestored then -- the choice made in the ride is dropped at the unmount
    local r = st.intentRestored
    st.intentRestored = nil
    M.Log(string.format("t=%.2f unmount: perspective choice of the ride dropped (%s -> %s, the choice from before the mount)",
      now, r.from and "TPP" or "FPP", r.to and "TPP" or "FPP"))
  end
  if dec ~= prev then
    M.history[#M.history + 1] = { t = now, perspective = dec.perspective, reason = dec.reason, detail = dec.detail,
                                  style = dec.style, changed = changed, mode = M.mode }
    if #M.history > 50 then table.remove(M.history, 1) end
    local shown = changed or dec.switchSuppressed
    M.Log(string.format("t=%.2f %s%s <- %s%s%s%s [%s]%s", now, shown and "" or "(reason) ", dec.perspective,
      tostring(dec.reason), dec.detail and (" (" .. dec.detail .. ")") or "",
      shown and (" " .. dec.style .. (dec.blend > 0 and string.format(" %.2fs", dec.blend) or "")) or "",
      dec.restore and " restore" or "", M.mode,
      dec.switchSuppressed and " - perspective switch suppressed (vehicle hold, rep owned)" or ""))
    if dec.switchSuppressed then M.stats.suppressed = (M.stats.suppressed or 0) + 1 end
  end
  if st.watch == "issued" then
    M.stats.watchdog = (M.stats.watchdog or 0) + 1
    M.Log(string.format("t=%.2f perspective switch watchdog: third person not active %.2f s after the unmount - switch issued",
      now, cfgNum(M.cfg, "vehicleUnmountWatch", 0.2)))
  elseif st.watch == "ok" then
    M.Log(string.format("t=%.2f perspective switch watchdog: third person active after the unmount - no switch needed", now))
  end
  st.watch = nil
  if changed then
    M.stats.changes = M.stats.changes + 1
    notify(dec, prev)
  end
  M.VehicleAnimStep(now, obs, dec)
  return dec, changed
end

local function reapply()
  if M.lastNow and M.lastObs then return M.Update(M.lastNow, M.lastObs) end
  return nil
end

local function persist()
  -- a choice made while mounted is for this ride only (never stored; the earlier choice returns at the unmount)
  if M.st.vehMounted then
    M.stats.rideChoices = (M.stats.rideChoices or 0) + 1
    M.Log("perspective choice while mounted: not stored (this ride only, the choice from before the mount returns at the unmount)")
    return
  end
  if M.core and M.cfg.remember then pcall(M.core.Set, "perspective.userWantsTpp", M.st.wantsTpp) end
end

function M.ToggleUser(source)
  M.stats.toggles = M.stats.toggles + 1
  local before = M.st.wantsTpp
  local ok, note = P.Toggle(M.st)
  M.Log(string.format("toggle (%s): %s%s", tostring(source or "api"), ok and "" or "refused ", tostring(note)))
  if M.st.wantsTpp ~= before then persist() end
  reapply()
  return ok, note
end

function M.SetUserWantsTpp(b, source)
  b = b and true or false
  local before = M.st.wantsTpp
  M.st.wantsTpp, M.st.userEvent = b, true
  if b ~= before then
    M.Log(string.format("user preference %s (%s)", b and "TPP" or "FPP", tostring(source or "api")))
    persist()
  end
  reapply()
  return true
end

function M.SetQuickFpp(down)
  if P.SetQuick(M.st, down) then reapply() end
  return M.st.quick
end

-- utpp_camera's unarmed zoom asks for first person past its last third-person step (on) and releases it when RMB
-- is let go (off). A user action: applied at once (no minGap). Returns true.
function M.RequestFppZoom(on, source)
  on = on and true or false
  local st = M.st
  if (st.zoomFpp == true) == on then return true end
  st.zoomFpp, st.userEvent = on, true
  M.stats.zoomRequests = (M.stats.zoomRequests or 0) + 1
  M.Log(string.format("unarmed zoom FPP request %s (%s)", on and "ON" or "OFF", tostring(source or "api")))
  reapply()
  return true
end
function M.ZoomFppRequested() return M.st.zoomFpp == true end

function M.Explain()
  local parts = {}
  for _, rule in ipairs(P.RULES) do
    local r = M.st.rules[rule.id]
    if r.latched or r.on then
      parts[#parts + 1] = string.format("%s%s%s(%s)", rule.id, r.latched and "*" or "", M.st.suppressed[rule.id] and "~" or "",
        tostring(r.detail))
    end
  end
  return table.concat(parts, " ")
end

-- "no" or "yes(<record>: <reason>)" for the weapon of the last poll
function M.ScopedText()
  local sc = P.Scoped(M.lastObs)
  if not sc then return "no" end
  return string.format("yes(%s: %s)", tostring(M.lastObs.weaponRecord or M.lastObs.weaponType or "?"), sc)
end

-- scope reads cached per weapon. key = weapon identity (entity id string); readers.slot() -> bool|nil (nil = failed
-- read, not cached; true = a scope part of ANY kind sits in the Scope slot), second value = part item type name,
-- third = part record name; readers.fov() -> number|nil. Returns slot, fov, part. A different key (another item in
-- WeaponRight) re-reads. The cache holds ONLY plain values (boolean / number / string): anything else a reader
-- returns is dropped, never a game object (handles released after the engine tore the world down crashed ExitGame).
local function plain(v)
  local t = type(v)
  if t == "boolean" or t == "number" or t == "string" then return v end
  return nil
end
function M.ScopeInfo(key, readers)
  local c = M.scopeCache
  if c and key ~= nil and c.key == key then return c.slot, c.fov, c.part end
  local slot, part, partName, fov = nil, nil, nil, nil
  if readers.slot then slot, part, partName = readers.slot() end -- (not "and / or": a false result must stay false)
  if readers.fov then fov = readers.fov() end
  slot, part, partName, fov = plain(slot), plain(part), plain(partName), plain(fov)
  M.scopeReads = (M.scopeReads or 0) + 1
  if type(key) == "string" and slot ~= nil then
    M.scopeCache = { key = key, slot = slot, fov = fov, part = part, partName = partName }
  else
    M.scopeCache = nil
  end
  return slot, fov, part
end

-- drop everything a poll could have kept from the game (cache, last diagnostic). Game object handles are never kept
-- in module state; this is the single place that is called on shutdown / player gone / reload.
function M.ReleaseHandles()
  M.scopeCache = nil
  M.scopeDiagLine = nil
end

-- Public API used by utpp_reload_gate (aim-at-entry of the Nue reload branch): keep the name in sync with the gate,
-- test_utpp_perspective.lua checks every perspective.<Name>() the gate calls. True while the player aims right now
-- (PSM UpperBody Aim); live reader set up below, last observation as the fallback.
function M.IsAiming()
  if M.liveAiming then
    local ok, v = pcall(M.liveAiming)
    if ok and v ~= nil then return v == true end
  end
  return M.lastObs ~= nil and M.lastObs.aiming == true
end

function M.Status()
  local d = M.st.current
  return string.format("utpp_perspective %s mode=%s desired=%s reason=%s%s style=%s wantsTpp=%s quick=%s locked=%s "
    .. "session=%s rules=[%s] steps=%d changes=%d notified=%d listeners=%d "
    .. "policies=[%s]", M.VERSION, M.mode, d.perspective, tostring(d.reason), d.detail and (":" .. d.detail) or "",
    tostring(d.style), tostring(M.st.wantsTpp), tostring(M.st.quick), tostring(M.st.locked), M.st.session, M.Explain(),
    M.stats.steps, M.stats.changes, M.stats.notified, #M.listeners, P.Policies(M.cfg))
    .. string.format(" meleeBlock=%s(%s)", M.cfg.meleeBlockTpp ~= false and "on" or "off", M.meleeBlockKey or "-")
    .. string.format(" zoomFpp=%s(%d)", M.st.zoomFpp == true and "on" or "off", M.stats.zoomRequests or 0)
    .. string.format(" scoped=%s aimingScoped=%s shortScopeFpp=%s", M.ScopedText(), P.AimingScopedFpp(M.cfg) and "fpp" or "tpp",
      M.cfg.shortScopeFpp == true and "on" or "off")
    .. string.format(" vehicle=[mounted=%s intent=%s gameTpp=%s owed=%s suppressed=%d watchdog=%d anim=%s]",
      tostring(M.st.vehMounted == true), M.st.vehIntent == nil and "-" or (M.st.vehIntent and "TPP" or "FPP"),
      tostring(M.st.vehGameTpp or "-"), tostring(M.st.switchOwed == true), M.stats.suppressed or 0, M.stats.watchdog or 0,
      M.vehAnim == nil and "-" or (M.vehAnim and "on" or "off"))
end

-- ---------------------------------------------------------------- settings (UTPP tab via utpp_core)
M.SETTINGS = {
  { key = "perspective.impl", type = "bool", default = true, label = "Use UTPP automatic perspective",
    desc = "Switches between first and third person by situation." },
  { key = "perspective.startInLastView", cfg = "remember", type = "bool", default = true, label = "Start in the last used view",
    desc = "On: the game starts in the view you used last. Off: the game always starts in third person (default: on)." },
  -- the mod always starts in third person; this key is no longer a menu option (developer level, the stored
  -- value is ignored: see M.StartWantsTpp) and stays registered only so old settings files keep loading
  { key = "perspective.defaultTpp", cfg = "defaultTpp", type = "bool", default = true, label = "Start in third person (unused)",
    desc = "Unused: the mod always starts in third person." },
  { key = "perspective.scene", cfg = "scene", type = "bool", default = true, label = "First person in scenes and dialogues",
    desc = "Scenes from the selected tier upwards use the game's first-person view; afterwards the view returns." },
  { key = "perspective.sceneTier", cfg = "sceneTier", type = "int", default = 3, min = 1, max = 5, label = "Scene tier for first person (1-5)",
    desc = "Lowest scene tier (1-5) that switches to first person. 3 = dialogues that block movement." },
  { key = "perspective.combat", cfg = "combat", type = "choice", default = "tpp", label = "Perspective in combat",
    desc = "Third person: combat stays in third person (default). First person: combat cuts to first person. "
      .. "Weapons are set separately below.",
    choices = { { "tpp", "Third person" }, { "fpp", "First person" } } },
  { key = "perspective.weaponDrawn", cfg = "weaponDrawn", type = "choice", default = "tpp",
    label = "Perspective with a weapon drawn",
    desc = "Third person: a weapon in hand stays in third person (default). First person: first person whenever a weapon "
      .. "is in hand.",
    choices = { { "tpp", "Third person" }, { "fpp", "First person" } },
    initial = function(stored) return stored == "drawn" and "fpp" or nil end, legacy = "perspective.weapon" },
  { key = "perspective.aiming", cfg = "aiming", type = "choice", default = "tpp", label = "Perspective while aiming",
    desc = "Aiming a weapon, throwing a grenade or using arm cyberware. Third person (default): over-the-shoulder aiming. "
      .. "First person: the game's first-person aiming.",
    choices = { { "fpp", "First person" }, { "tpp", "Third person" } },
    -- the registration default is "tpp" (user's setting): the old weapon policies that meant first person keep it
    initial = function(stored)
      if stored == "never" then return "tpp" end
      if stored == "aim" or stored == "drawn" then return "fpp" end
      return nil
    end, legacy = "perspective.weapon" },
  { key = "perspective.aimingScoped", cfg = "aimingScoped", type = "choice", default = "fpp",
    label = "Perspective while aiming with a scope",
    desc = "Applies to ranged weapons with a scope installed and to sniper / precision rifles, instead of 'Perspective while "
      .. "aiming'. First person: the scope view cannot be shown over the shoulder. Third person: the scope picture is not "
      .. "rendered. Default: first person.",
    choices = { { "fpp", "First person" }, { "tpp", "Third person" } } },
  { key = "perspective.shortScopeFpp", cfg = "shortScopeFpp", type = "bool", default = false,
    label = "Short scopes count as scoped (FPP)",
    desc = "Off (default): a short scope (red dot / 1.5x, e.g. on the Overture revolver) keeps over-the-shoulder aiming; only "
      .. "long, tech and power sniper scopes and sniper rifles use 'Perspective while aiming with a scope'. On: short "
      .. "scopes use that setting too." },
  { key = "perspective.meleeBlockTpp", cfg = "meleeBlockTpp", type = "bool", default = true,
    label = "Third person when blocking (melee)",
    desc = "Holding aim with a melee weapon or fists is a block, not aiming: the view stays in third person even when "
      .. "'Perspective while aiming' is First person (that setting then applies to ranged weapons only). Off: blocking counts "
      .. "as aiming." },
  { key = "perspective.vehicleKeepTpp", cfg = "vehicleKeepTpp", type = "bool", default = true,
    label = "Third person on vehicle enter / exit",
    desc = "Entering from third person stays in third person until seated (then the vehicle camera); leaving a seat "
      .. "returns to third person as the exit starts. Off: the game camera for the whole transition." },
  { key = "perspective.combatCut", cfg = "combatCut", type = "bool", default = false, label = "Instant switch for combat / weapons",
    desc = "Combat and weapons switch instantly instead of the animated transition." },
  { key = "perspective.combatExit", cfg = "combatExit", type = "float", default = 1.25, min = 0, max = 5, step = 0.25,
    label = "Return delay after combat (s)", desc = "Time without combat / weapon before third person returns." },
  { key = "perspective.takedown", cfg = "takedown", type = "bool", default = false, label = "First person for takedowns",
    desc = "Grapples, takedowns and carrying bodies." },
  { key = "perspective.workspot", cfg = "workspot", type = "bool", default = false, label = "First person in workspots",
    desc = "Sitting, leaning and other workspots (released after 5 minutes if the state gets stuck)." },
  { key = "perspective.interaction", cfg = "interaction", type = "bool", default = false,
    label = "First person at terminals and devices",
    desc = "Device zoom, terminals in use, elevators, forcing doors, personal link and breach." },
  { key = "perspective.interactionExit", cfg = "interactionExit", type = "float", default = 1.0, min = 0, max = 5, step = 0.25,
    label = "Return delay after interactions (s)", desc = "Time after the interaction before third person returns." },
  { key = "perspective.traversal", cfg = "traversal", type = "bool", default = false, label = "First person on ladders and swimming",
    desc = "Off: ladders and swimming stay in third person." },
  { key = "perspective.scanner", cfg = "scanner", type = "bool", default = false, label = "First person while scanning",
    desc = "Instant first person while the scanner is open." },
  { key = "perspective.toggleOverrides", cfg = "toggleOverrides", type = "bool", default = false,
    label = "Toggle overrides automatic first person",
    desc = "Toggling to third person during combat, weapons, workspots, interactions, ladders or the scanner keeps third "
      .. "person until that situation ends. Off: the switch waits until the situation ends." },
  { key = "perspective.sceneExit", cfg = "sceneExit", type = "float", default = 1.5, min = 0, max = 5, step = 0.25,
    label = "Return delay after scenes (s)", desc = "Time after a scene before third person returns." },
  { key = "perspective.animate", cfg = "animate", type = "bool", default = true, label = "Animated transitions",
    desc = "Off: every switch is an instant cut." },
  { key = "perspective.blend", cfg = "blend", type = "float", default = 0.8, min = 0.2, max = 2.5, step = 0.1,
    label = "Transition time (s)", desc = "Length of animated transitions (scenes use 0.6 s)." },
  { key = "perspective.minGap", cfg = "minGap", type = "float", default = 0.5, min = 0, max = 3, step = 0.1,
    label = "Min. time between auto switches (s)", desc = "Prevents flicker; cuts for combat and vehicles ignore it." },
  { key = "perspective.log", cfg = "log", type = "bool", default = true, label = "Log perspective decisions",
    desc = "Writes decisions to the UTPP log." },
}

function M.AttachCore(core)
  if type(core) ~= "table" or type(core.Register) ~= "function" then return false, "utpp_core not available" end
  M.core = core
  core.RegisterSection("perspective", "Perspective (automatic)", { owner = "utpp_perspective" })
  for _, s in ipairs(M.SETTINGS) do
    local def = copy(s)
    def.section, def.cfg = "perspective", nil
    -- legacy migration (perspective.weapon drawn / aim / never -> weaponDrawn / aiming): initial value when nothing stored
    if type(s.initial) == "function" then
      local st = core.settings and core.settings.stored
      def.initial = s.legacy and st and s.initial(st[s.legacy]) or nil
    end
    def.legacy = nil
    if s.cfg then
      local field = s.cfg
      def.apply = function(v) M.cfg[field] = v end
    end
    core.Register(def)
  end
  -- hidden (no widget): the remembered choice
  core.Register({ key = "perspective.userWantsTpp", type = "bool", default = true, label = "Remembered perspective (TPP)" })
  if M.cfg.remember then
    M.st.wantsTpp = core.Get("perspective.userWantsTpp") ~= false
  else
    M.st.wantsTpp = true -- always third person (the stored perspective.defaultTpp is ignored)
  end
  M.mode = M.Mode()
  return true
end

-- ================================================================ in-game wiring (skipped by the offline tests)
if type(registerForEvent) == "function" then
  local clock, pollTimer, announced = 0, 0, false
  local POLL = 0.1
  M.reads = { failed = {} }
  M.sink = function(line)
    print(line)
    if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
  end

  local function readFailed(name, err)
    if M.reads.failed[name] then return end
    M.reads.failed[name] = true
    M.Log("read " .. name .. " FAILED (input treated as absent): " .. tostring(err))
  end

  local function try(name, fn)
    local ok, v = pcall(fn)
    if not ok then readFailed(name, v) return nil end
    return v
  end

  -- vehicle inputs. Other UTPP modules through GetMod (plain Lua tables, never kept in the module state).
  local function modApi(name)
    if type(GetMod) ~= "function" then return nil end
    local ok, m = pcall(GetMod, name)
    if ok and type(m) == "table" then return m end
    return nil
  end
  local function boolCall(m, fname)
    if m == nil or type(m[fname]) ~= "function" then return nil end
    local ok, v = pcall(m[fname])
    if ok and type(v) == "boolean" then return v end
    return nil
  end
  -- utpp_camera hold / TPP signal and the utpp_body representation (only while mounted or a suppressed switch is owed)
  local function readUtpp(o)
    local cam = modApi("utpp_camera")
    o.camHeld, o.camTpp = boolCall(cam, "VehicleHeld"), boolCall(cam, "TppSignalState")
    local body = modApi("utpp_body")
    if body and type(body.GetState) == "function" then
      local ok, s = pcall(body.GetState)
      if ok and type(s) == "table" and type(s.rep) == "string" then o.bodyRep = s.rep end
    end
    if o.camTpp ~= nil then o.tppActive = o.camTpp == true and o.bodyRep ~= "fpp" end
  end
  M.liveCamHeld = function() return boolCall(modApi("utpp_camera"), "VehicleHeld") end
  -- vehicleCameraPerspective as a number (EnumInt, else the enum name), nil when unreadable
  local PERSP_IDX = { FPP = 0, TPPClose = 1, TPPMedium = 2, TPPFar = 3, DriverCombatClose = 4, DriverCombatMedium = 5,
                      DriverCombatFar = 6 }
  local function perspInt(raw)
    if type(raw) == "number" then return raw end
    if raw == nil then return nil end
    local okE, n = pcall(function() return EnumInt(raw) end)
    if okE and type(n) == "number" then return n end
    local okS, s = pcall(function() return Game.EnumValueToString("vehicleCameraPerspective", raw) end)
    local text = (okS and type(s) == "string" and s ~= "") and s or tostring(raw)
    local best, bestLen = nil, 0
    for name, i in pairs(PERSP_IDX) do
      if string.find(text, name, 1, true) and #name > bestLen then best, bestLen = i, #name end
    end
    return best
  end
  -- the game's vehicle camera in the driver seat: o.vehActive (0-6) or o.vehTpp (IsTPPActive); handles stay locals
  local function readVehicleCam(player, o)
    local ok, err = pcall(function()
      local veh = Game.GetMountedVehicle(player)
      if veh == nil then return end
      local mgr = veh:GetCameraManager()
      veh = nil
      if mgr == nil then return end
      local okA, raw = pcall(function() return mgr:GetActivePerspective() end)
      if okA then o.vehActive = perspInt(raw) end
      raw = nil
      if o.vehActive == nil then
        local okT, b = pcall(function() return mgr:IsTPPActive() end)
        if okT and type(b) == "boolean" then o.vehTpp = b end
      end
      mgr = nil
    end)
    if not ok then readFailed("vehicle camera (GetMountedVehicle)", err) end
  end

  local function readObs(player)
    local o = { player = true }
    local defs = GetAllBlackboardDefs()
    local bbs = Game.GetBlackboardSystem()
    local D = defs.PlayerStateMachine
    local psm = try("PSM blackboard", function() return bbs:GetLocalInstanced(player:GetEntityID(), D) end)
    if psm ~= nil then
      local function gi(n) return try("PSM." .. n, function() return psm:GetInt(D[n]) end) end
      local function gb(n) return try("PSM." .. n, function() return psm:GetBool(D[n]) end) == true end
      o.vehicle, o.high, o.combat, o.swim = gi("Vehicle"), gi("HighLevel"), gi("Combat"), gi("Swimming")
      o.workspot = (gi("IsInWorkspot") or 0) ~= 0
      o.takedown, o.carry, o.loco, o.vision = gi("Takedown"), gi("BodyCarrying"), gi("LocomotionDetailed"), gi("Vision")
      o.aiming = gi("UpperBody") == 6
      o.meleeState = gi("MeleeWeapon")
      o.gadget, o.lhcw = gi("CombatGadget"), gi("LeftHandCyberware")
      o.mounted, o.uiZoom, o.controllingDevice = gb("MountedToVehicle"), gb("IsUIZoomDevice"), gb("IsControllingDevice")
      o.elevator, o.forceDoor = gb("IsPlayerInsideElevator"), gb("IsForceOpeningDoor")
      o.personalLink, o.minigame = gb("IsInteractingViaPersonalLink"), gb("IsInMinigame")
    end
    -- vehicle inputs (nothing extra is read on foot)
    if (o.vehicle or 0) ~= 0 or o.mounted == true or M.st.switchOwed then readUtpp(o) end
    if o.vehicle == 1 or o.vehicle == 6 then readVehicleCam(player, o) end
    o.menu = try("UI_System.IsInMenu", function() return bbs:Get(defs.UI_System):GetBool(defs.UI_System.IsInMenu) end) == true
      or try("IsGamePaused", function() return Game.GetSystemRequestsHandler():IsGamePaused() end) == true
    o.photo = try("PhotoMode", function() return Game.GetPhotoModeSystem():IsPhotoModeActive() end) == true
    o.braindance = try("Braindance.IsActive", function() return bbs:Get(defs.Braindance):GetBool(defs.Braindance.IsActive) end) == true
    o.replacer = try("UI_PlayerStats.isReplacer",
      function() return bbs:Get(defs.UI_PlayerStats):GetBool(defs.UI_PlayerStats.isReplacer) end) == true
    o.cyberspace = try("CyberspacePresence",
      function() return Game.GetStatusEffectSystem():HasStatusEffectWithTag(player:GetEntityID(), "CyberspacePresence") end) == true
    -- WeaponRight item (nil = empty slot = fists, or a failed read: weaponType stays nil then)
    local slotOk, item = false, nil
    do
      local okS, v = pcall(function()
        return Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new("AttachmentSlots.WeaponRight"))
      end)
      if okS then slotOk, item = true, v else readFailed("WeaponRight slot", v) end
    end
    o.weaponDrawn = item ~= nil
    -- weapon class for the melee block: gamedataItemType name of the item (record), "" = empty slot (fists)
    if slotOk and item == nil then
      o.weaponType = ""
    elseif item ~= nil then
      o.weaponType = try("weapon type", function()
        local id = item:GetItemID()
        local rec = TweakDBInterface.GetWeaponItemRecord(ItemID.GetTDBID(id))
        if rec == nil then return nil end
        local t = rec:ItemType():Type()
        if type(t) == "string" then return t end
        local okV, name = pcall(function() return t.value end)
        if okV and type(name) == "string" then return name end
        return tostring(t)
      end)
    end
    -- scope detection (cached per weapon entity id, see M.ScopeInfo); melee / fists skip it. Only plain values
    -- (strings / numbers / booleans) leave this block; every game struct (item data, parts, records, ids) is a local that
    -- is set to nil before readObs returns.
    if item ~= nil and not (type(o.weaponType) == "string" and P.MELEE_TYPES[o.weaponType]) then
      local key = try("weapon entity id", function()
        local e = item:GetEntityID()
        local okH, h = pcall(function() return e.hash end)
        if okH and h ~= nil then return tostring(h) end
        return tostring(e)
      end)
      local recName = try("weapon record name", function()
        return TDBID.ToStringDEBUG(ItemID.GetTDBID(item:GetItemID()))
      end)
      local function enumName(t)
        if type(t) == "string" then return t end
        local okV, name = pcall(function() return t.value end)
        if okV and type(name) == "string" then return name end
        return tostring(t)
      end
      -- (record name, item type name) of an item id as plain strings, nil when the id is not valid
      local function partInfo(itemId)
        if itemId == nil or not ItemID.IsValid(itemId) then return nil end
        local tdb = ItemID.GetTDBID(itemId)
        local rec = TweakDBInterface.GetItemRecord(tdb)
        local t = rec ~= nil and enumName(rec:ItemType():Type()) or "?"
        local name = TDBID.ToStringDEBUG(tdb)
        tdb, rec = nil, nil
        return { name = name, type = t }
      end
      local diag = nil
      -- true when a scope part of any kind sits in the Scope slot (the policy is applied below, live), part type, part name
      local function readSlot()
        local scopeSlotId = TweakDBID.new("AttachmentSlots.Scope")
        -- primary: the parts of the weapon's item data; the part sitting in the Scope slot must be a real scope
        local data = try("weapon item data", function() return item:GetItemData() end)
        local parts = data ~= nil and try("item parts", function()
          local r = data:GetItemParts()
          if type(r) ~= "table" then r = data:GetItemParts({}) end
          return r
        end) or nil
        if type(parts) == "table" then
          local found, ptype, nParts = nil, nil, #parts
          for _, part in ipairs(parts) do
            local pslot = try("part slot", function() return InnerItemData.GetSlotID(part) end)
            local isScopeSlot = pslot ~= nil and (pslot == scopeSlotId
              or select(2, pcall(function() return TDBID.ToStringDEBUG(pslot) end)) == "AttachmentSlots.Scope")
            pslot = nil
            if isScopeSlot then
              local pi = try("scope part", function() return partInfo(InnerItemData.GetItemID(part)) end)
              if pi ~= nil then found, ptype = pi.name, pi.type end
            end
            part = nil
          end
          parts, data, scopeSlotId = nil, nil, nil
          diag = string.format("scope part=%s type=%s (GetItemParts, %d parts)", tostring(found or "none"), tostring(ptype or "-"), nParts)
          return P.ScopeKind(ptype) ~= nil, ptype, found
        end
        -- fallback (item parts unreadable): the item in the Scope slot of the weapon, validated the same way
        local w = try("scope slot (transaction)", function() return Game.GetTransactionSystem():GetItemInSlot(item, scopeSlotId) end)
        scopeSlotId = nil
        if w == nil then
          diag = "scope part=none (GetItemInSlot nil)"
          local hadData = data ~= nil
          data, parts = nil, nil
          if hadData then return false end
          return nil
        end
        local pi = try("scope part (transaction)", function() return partInfo(w:GetItemID()) end)
        w, data, parts = nil, nil, nil
        diag = string.format("scope part=%s type=%s (GetItemInSlot)", tostring(pi and pi.name or "none"), tostring(pi and pi.type or "-"))
        if pi == nil then return false end
        return P.ScopeKind(pi.type) ~= nil, pi.type, pi.name
      end
      local slot, fov, ptype = M.ScopeInfo(key, {
        slot = readSlot,
        fov = function()
          return try("ScopeFOV", function() return Game.GetStatsSystem():GetStatValue(item:GetEntityID(), "ScopeFOV") end)
        end,
      })
      -- the cache keeps the part type; the shortScopeFpp setting is applied on every poll
      local kind = slot == true and (P.ScopeKind(ptype) or "long") or P.ScopeKind(ptype)
      o.scopeKind = kind
      o.scopeSlot = slot
      if slot == true and kind == "short" and not (M.cfg and M.cfg.shortScopeFpp == true) then o.scopeSlot = false end
      o.scopeFov, o.weaponRecord = fov, recName
      if diag ~= nil then -- once per fresh read (cache), the data to judge a wrong scope decision
        local line = string.format("scope read weapon=%s type=%s %s ScopeFOV=%s", tostring(recName), tostring(o.weaponType), diag, tostring(fov))
        if M.scopeDiagLine ~= line then
          M.scopeDiagLine = line
          M.Log(line)
        end
      end
    end
    -- no game struct stays referenced after this poll (see the header)
    item = nil
    o.terminal = try("UIGameData.InteractionData", function()
      local d = defs.UIGameData
      local v = bbs:Get(d):GetVariant(d.InteractionData)
      if v == nil then return false end
      local okF, data = pcall(FromVariant, v)
      if okF and data ~= nil then v = data end
      return v.terminalInteractionActive == true
    end) == true
    -- mirror / character customization: no verified signal yet (input stays absent)
    psm, bbs, defs, D, player = nil, nil, nil, nil, nil -- no handle outlives the poll
    -- the DLL's upper-body mask state "aim" (bit 2) fires on PSM UpperBody Aim even with EMPTY hands (RMB
    -- unarmed), and the game graph has no upper body for that -> T-pose arms. Until the DLL checks the weapon itself,
    -- allow the aim mask only while an item is in WeaponRight: states 7 (melee+aim+exit) with a weapon, 5 without.
    if slotOk then
      local want = o.weaponDrawn and 7 or 5
      if want ~= M.aimMaskBits then
        local okM, res = pcall(function() return Game.UTPPSetMMUpperMaskStates(want) end)
        if okM and res ~= false then
          M.aimMaskBits = want
          M.Log(string.format("MM upper-body mask states -> %d (%s)", want, o.weaponDrawn and "weapon in hand: aim mask allowed" or "empty hands: aim mask off"))
        elseif M.aimMaskBits ~= "failed" then
          M.aimMaskBits = "failed"
          M.Log("MM upper-body mask states: UTPPSetMMUpperMaskStates unavailable (" .. tostring(res) .. ")")
        end
      end
    end
    return o
  end

  M.liveAiming = function()
    local player = Game.GetPlayer()
    if player == nil then return nil end
    local D = GetAllBlackboardDefs().PlayerStateMachine
    local psm = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), D)
    if psm == nil then return nil end
    return psm:GetInt(D.UpperBody) == 6
  end

  registerForEvent("onInit", function()
    M.shuttingDown = false
    if announced then return end
    announced = true
    local core = nil
    if type(GetMod) == "function" then
      local ok, c = pcall(GetMod, "utpp_core")
      if ok and type(c) == "table" then core = c end
    end
    local okA, err = false, "utpp_core not found (settings not persisted, defaults used)"
    if core then okA, err = pcall(M.AttachCore, core) end
    if not okA then M.st.wantsTpp = true end -- no settings: always third person
    M.mode = M.Mode()
    M.Log(string.format("onInit %s: mode=%s settings=%s wantsTpp=%s", M.VERSION, M.mode,
      okA and "utpp_core" or ("defaults (" .. tostring(err) .. ")"), tostring(M.st.wantsTpp)))
  end)

  -- game exit. Drop every cached handle / struct (only plain values are kept anyway), stop polling (the player is
  -- gone, game calls would only touch torn-down systems) and collect now, while Lua still owns the wrappers.
  registerForEvent("onShutdown", function()
    M.shuttingDown = true
    pcall(M.ReleaseHandles)
    pcall(M.Log, "onShutdown: scope cache and cached handles released, polling stopped")
    pcall(collectgarbage, "collect")
  end)

  registerForEvent("onUpdate", function(delta)
    if M.shuttingDown then return end
    local dt = math.max(delta or 0, 0)
    clock = clock + dt
    pollTimer = pollTimer + dt
    if pollTimer < POLL then return end
    pollTimer = 0
    local ok, err = pcall(function()
      local okP, player = pcall(function() return Game.GetPlayer() end)
      if not okP or player == nil then
        M.ReleaseHandles() -- no player = the world is going away (load / exit): hold nothing from it
        M.Update(clock, { player = false })
        return
      end
      local o = readObs(player)
      M.Update(clock, o)
    end)
    if not ok then
      M.stats.updateErrors = (M.stats.updateErrors or 0) + 1
      if M.stats.updateErrors <= 3 then M.Log("update error: " .. tostring(err)) end
    end
  end)
end

return M

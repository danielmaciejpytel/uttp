-- UTPP utpp_perspective 0.2.5 CET load test: the init.lua is loaded in a CET-like environment (registerForEvent present,
-- GetMod defined only AFTER the file was loaded), with a mock game (PSM blackboard, UI / braindance / replacer
-- blackboards, photo mode, status effects, transaction and camera systems). Checks: events registered at load, one onInit
-- status line (also to Game.UTPPLogLine), no update errors, decisions from the blackboard, subscribers notified, settings
-- via utpp_core, failing reads logged once.
local results = { pass = 0, fail = 0 }
local outPrint = print
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  outPrint(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local here = string.match(debug.getinfo(1, "S").source, "^@(.*)$") or ""
local root = string.match(here, "^(.*)[/\\]Source[/\\]tools[/\\]lua%-tests[/\\]") or "."
local PERSP = UTPP_PERSPECTIVE_PATH or (root .. "\\Source\\plugin\\UTPP\\cet\\utpp_perspective\\init.lua")
local CORE = UTPP_CORE_PATH or (root .. "\\Source\\plugin\\UTPP\\cet\\utpp_core\\init.lua")

local saved = { print = print, registerForEvent = registerForEvent, registerInput = registerInput, GetMod = GetMod, Game = Game,
                GetAllBlackboardDefs = GetAllBlackboardDefs, TweakDBID = TweakDBID, Transform = Transform, FromVariant = FromVariant,
                TweakDBInterface = TweakDBInterface, ItemID = ItemID, TDBID = TDBID, InnerItemData = InnerItemData }
local function restore() for k, v in pairs(saved) do _G[k] = v end end

-- ---------------------------------------------------------------- mock game
-- 0.2.5: every object the mock game hands out (player, blackboards, item, item data, parts, systems) carries the MOCK
-- metatable, so a walk over the module state can prove that no "game handle" outlives a poll.
local MOCK = {}
local function mock(t) return setmetatable(t, MOCK) end
-- walks tables (keys + values) and function upvalues from the roots; returns the paths of mock game objects / userdata found
local function findGameRefs(roots)
  local seen, found = {}, {}
  local function visit(v, path)
    local t = type(v)
    if t == "userdata" then found[#found + 1] = path .. "<userdata>" return end
    if t ~= "table" and t ~= "function" then return end
    if seen[v] then return end
    seen[v] = true
    if t == "table" then
      if getmetatable(v) == MOCK then found[#found + 1] = path return end
      for k, x in pairs(v) do
        visit(k, path .. "<key>")
        visit(x, path .. "." .. tostring(k))
      end
    else
      local i = 1
      while true do
        local n, x = debug.getupvalue(v, i)
        if not n then break end
        visit(x, path .. "^" .. n)
        i = i + 1
      end
    end
  end
  for name, r in pairs(roots) do visit(r, name) end
  return found
end
local function makeGame(world)
  local defs = { PlayerStateMachine = {}, UI_System = { IsInMenu = "IsInMenu" }, Braindance = { IsActive = "IsActive" },
                 UI_PlayerStats = { isReplacer = "isReplacer" }, UIGameData = { InteractionData = "InteractionData" } }
  for _, n in ipairs({ "Vehicle", "HighLevel", "Combat", "Swimming", "IsInWorkspot", "Takedown", "BodyCarrying",
    "LocomotionDetailed", "Vision", "UpperBody", "CombatGadget", "LeftHandCyberware", "MountedToVehicle", "IsUIZoomDevice",
    "IsControllingDevice", "IsPlayerInsideElevator", "IsForceOpeningDoor", "IsInteractingViaPersonalLink", "IsInMinigame",
    "MeleeWeapon" }) do
    defs.PlayerStateMachine[n] = n
  end
  GetAllBlackboardDefs = function() return defs end
  TweakDBID = { new = function(s) return s end }
  FromVariant = function(v) return v end
  -- 0.2.3: weapon class reads (item -> ItemID -> weapon record -> ItemType():Type() as an enum-like table with .value)
  ItemID = { GetTDBID = function(id) return id end, IsValid = function(id) return id ~= nil and id.valid ~= false end }
  -- 0.2.4: parts of the weapon item data (world.parts = { { slot = "AttachmentSlots.Scope", id = { name =, ptype =, valid = } } })
  InnerItemData = { GetSlotID = function(part) return part.slot end, GetItemID = function(part) return part.id end }
  TDBID = { ToStringDEBUG = function(id) return id.name or id.type end }
  TweakDBInterface = { GetWeaponItemRecord = function(id)
    return { ItemType = function() return { Type = function() return { value = id.type } end } end }
  end, GetItemRecord = function(id)
    return { ItemType = function() return { Type = function() return { value = id.ptype } end } end }
  end }
  local psm = mock({
    GetInt = function(_, id) return world.psm[id] or 0 end,
    GetBool = function(_, id) return world.psm[id] == true end,
  })
  local function bbFor(tbl) return mock({
    GetBool = function(_, id) return tbl[id] == true end,
    GetVariant = function(_, id) return tbl[id] end,
  }) end
  local player = mock({
    GetEntityID = function() return 1 end,
    GetWorldPosition = function() return { x = 0, y = 0, z = 0 } end,
  })
  local logged = {}
  Game = {
    UTPPLogLine = function(msg) logged[#logged + 1] = msg end,
    GetPlayer = function() world.playerCalls = (world.playerCalls or 0) + 1 if world.noPlayer then return nil end return player end,
    GetBlackboardSystem = function() return mock({
      GetLocalInstanced = function() return psm end,
      Get = function(_, d)
        if d == defs.UI_System then return bbFor(world.ui) elseif d == defs.Braindance then return bbFor(world.bd)
        elseif d == defs.UI_PlayerStats then return bbFor(world.stats) elseif d == defs.UIGameData then return bbFor(world.gamedata) end
      end,
    }) end,
    GetSystemRequestsHandler = function() return { IsGamePaused = function() return false end } end,
    GetPhotoModeSystem = function() return { IsPhotoModeActive = function() return world.photo == true end } end,
    GetStatusEffectSystem = function() return { HasStatusEffectWithTag = function(_, _, tag)
      if world.statusBroken then error("no such function") end
      return world.tags[tag] == true end } end,
    GetTransactionSystem = function() return mock({ GetItemInSlot = function(_, _, slot)
      if slot == "AttachmentSlots.WeaponRight" and world.weapon then
        -- 0.2.4: item object with entity id, item data (HasPartInSlot) for the scope slot, record name for the log line
        return mock({
          GetItemID = function() return mock({ type = world.weaponType or "Wea_Handgun", name = world.weaponName }) end,
          GetEntityID = function() return mock({ hash = world.weaponEntity or 1 }) end,
          GetItemData = function() return mock({
            HasPartInSlot = function() return true end, -- the cycle 28 false positive: true for an empty slot (must not be used)
            GetItemParts = function()
              world.slotReads = (world.slotReads or 0) + 1
              local parts = world.parts or {}
              for _, part in ipairs(parts) do
                mock(part)
                if type(part.id) == "table" then mock(part.id) end
              end
              return mock(parts)
            end }) end,
        })
      end
      return nil end }) end,
    GetStatsSystem = function() return mock({ GetStatValue = function(_, _, stat)
      return stat == "ScopeFOV" and (world.scopeFov or 0) or 0 end }) end,
    GetCameraSystem = function() return { GetActiveCameraWorldTransform = function()
      return true, { position = world.cam } end } end,
  }
  return logged
end

local function loadInCet(getModAfterLoad, world)
  local handlers, lines = {}, {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  registerInput = nil
  GetMod = nil
  local logged = makeGame(world)
  print = function(...)
    local parts = {}
    for i = 1, select("#", ...) do parts[#parts + 1] = tostring(select(i, ...)) end
    lines[#lines + 1] = table.concat(parts, " ")
  end
  local ok, mod = pcall(dofile, PERSP)
  GetMod = getModAfterLoad
  if ok and handlers.onInit then
    local okI, errI = pcall(handlers.onInit)
    if not okI then lines[#lines + 1] = "onInit raised: " .. tostring(errI) end
  end
  return ok, mod, handlers, lines, logged
end
local function run(handlers, frames)
  for _ = 1, frames do
    local ok, err = pcall(handlers.onUpdate, 1 / 60)
    if not ok then return err end
  end
end
local function count(lines, needle)
  local n = 0
  for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end
local function newWorld()
  return { psm = {}, ui = {}, bd = {}, stats = {}, gamedata = {}, tags = {}, cam = { x = 0.05, y = 0, z = 1.6 } }
end

-- 1) no utpp_core, no player at first
do
  local world = newWorld()
  world.noPlayer = true
  local ok, mod, handlers, lines, logged = loadInCet(function() return nil end, world)
  check("loads in a CET-like environment", ok and type(mod) == "table" and mod.VERSION == "0.2.7", ok and nil or tostring(mod))
  check("onInit / onUpdate registered although GetMod is defined only after load",
    type(handlers.onInit) == "function" and type(handlers.onUpdate) == "function")
  check("one onInit line: mode active, defaults (utpp_core missing)",
    count(lines, "[utpp_perspective] onInit 0.2.7: mode=active settings=defaults") == 1, table.concat(lines, " | "))
  check("onInit line also sent to Game.UTPPLogLine", count(logged, "onInit 0.2.7") == 1)
  local err = run(handlers, 120)
  check("no player: no update errors", err == nil and count(lines, "update error") == 0, err)
  world.noPlayer = false
  err = run(handlers, 120)
  check("player present: no update errors, session restored to TPP", err == nil and mod.Desired().perspective == "TPP"
    and mod.Desired().restore == true, mod.Status())
  check("polled at 10 Hz (not every frame)", mod.stats.steps >= 20 and mod.stats.steps <= 50, mod.stats.steps)
  world.psm.Combat = 1
  run(handlers, 12)
  check("PSM Combat=1, default policy combat=TPP -> stays TPP", mod.Desired().perspective == "TPP")
  mod.cfg.combat = "fpp"
  run(handlers, 12)
  check("PSM Combat=1, policy combat=FPP -> FPP (combat)", mod.Desired().perspective == "FPP" and mod.Desired().reason == "combat")
  mod.cfg.combat = "tpp"
  world.psm.Combat = 0
  world.psm.Vehicle = 5
  run(handlers, 12)
  check("PSM Vehicle=5 -> mounted turret", mod.Desired().detail == "turret")
  world.psm.Vehicle = 0
  world.stats.isReplacer = true
  run(handlers, 12)
  check("UI_PlayerStats.isReplacer -> outside (locked)", mod.Desired().reason == "outside" and mod.Desired().locked == true)
  world.stats.isReplacer = false
  world.weapon = true
  run(handlers, 60)
  check("WeaponRight slot occupied, default policy weaponDrawn=TPP -> stays TPP", mod.Desired().perspective == "TPP")
  mod.cfg.aiming = "fpp" -- the aiming rules below are tested with the FPP-while-aiming policy (the fallback default is tpp)
  world.psm.UpperBody = 6
  run(handlers, 12)
  check("WeaponRight + UpperBody Aim -> weapon (aiming)", mod.Desired().detail == "aiming")
  world.psm.UpperBody = 0
  mod.cfg.weaponDrawn = "fpp"
  run(handlers, 12)
  check("policy weaponDrawn=FPP: WeaponRight slot occupied -> weapon drawn", mod.Desired().detail == "weapon drawn")
  mod.cfg.weaponDrawn = "tpp"
  -- 0.2.3 melee block: aim with a melee weapon / fists (weapon record type or PSM MeleeWeapon 5-10) never switches to FPP
  run(handlers, 120)
  check("melee block: ranged weapon, back to TPP after the aim", mod.Desired().perspective == "TPP", mod.Status())
  world.psm.UpperBody, world.weaponType = 6, "Wea_Katana"
  run(handlers, 30)
  check("melee block: katana + aim -> stays TPP, one log line", mod.Desired().perspective == "TPP"
    and count(lines, "aiming ignored: melee block (weapon=Wea_Katana)") == 1 and string.find(mod.Status(), "meleeBlock=on(weapon=Wea_Katana)", 1, true) ~= nil,
    mod.Status())
  world.psm.UpperBody = 0
  run(handlers, 12)
  world.weaponType, world.psm.UpperBody = "Wea_Fists", 6
  run(handlers, 30)
  check("melee block: fists (Wea_Fists item) + aim -> stays TPP, one log line", mod.Desired().perspective == "TPP"
    and count(lines, "aiming ignored: melee block (weapon=Wea_Fists)") == 1, mod.Status())
  mod.cfg.meleeBlockTpp = false
  run(handlers, 12)
  check("melee block: setting off -> fists + aim is aiming again (FPP, old behaviour)", mod.Desired().detail == "aiming"
    and mod.Desired().perspective == "FPP", mod.Status())
  mod.cfg.meleeBlockTpp = true
  world.psm.UpperBody = 0
  run(handlers, 120)
  world.weaponType, world.psm.UpperBody = "Wea_Handgun", 6
  run(handlers, 12)
  check("melee block: pistol + aim -> FPP (aiming)", mod.Desired().detail == "aiming" and mod.Desired().perspective == "FPP")
  world.psm.UpperBody = 0
  run(handlers, 120)
  world.psm.MeleeWeapon, world.psm.UpperBody = 8, 6
  run(handlers, 30)
  check("melee block: pistol, PSM MeleeWeapon=8 (block state) + aim -> stays TPP, one log line", mod.Desired().perspective == "TPP"
    and count(lines, "aiming ignored: melee block (PSM MeleeWeapon=8)") == 1, mod.Status())
  world.psm.UpperBody, world.psm.MeleeWeapon, world.weaponType = 0, 0, nil
  run(handlers, 12)
  -- 0.2.4 scoped weapons: aimingScoped (default fpp) replaces aiming; reads cached per weapon entity, refreshed on a new item
  world.weapon, world.weaponType, world.weaponName, world.weaponEntity = true, "Wea_Handgun", "Items.Preset_Pistol", 11
  mod.cfg.aiming, mod.cfg.aimingScoped = "tpp", "fpp"
  world.psm.UpperBody = 6
  run(handlers, 12)
  check("scoped: pistol without a scope + aiming=tpp -> stays TPP, no scoped log", mod.Desired().perspective == "TPP"
    and count(lines, "aiming scoped") == 0 and string.find(mod.Status(), "scoped=no aimingScoped=fpp", 1, true) ~= nil, mod.Status())
  world.psm.UpperBody = 0
  run(handlers, 120)
  -- false positives of cycle 28 (HasPartInSlot is true here in the mock, as in the game for an empty slot)
  local function aimTpp(name, entity, wtype, wname, parts)
    world.weaponType, world.weaponName, world.weaponEntity, world.parts = wtype, wname, entity, parts
    world.psm.UpperBody = 6
    run(handlers, 30)
    local ok = mod.Desired().perspective == "TPP" and count(lines, "aiming scoped weapon=" .. wname) == 0
      and string.find(mod.Status(), "scoped=no", 1, true) ~= nil
    check("scoped false positive: " .. name .. " + aiming=tpp -> stays TPP, no scoped log", ok, mod.Status())
    world.psm.UpperBody = 0
    run(handlers, 120)
  end
  aimTpp("revolver (Overture), Scope slot present but empty (invalid part id)", 21, "Wea_Revolver", "Items.Preset_Overture_Kerry",
    { { slot = "AttachmentSlots.Scope", id = { valid = false } } })
  aimTpp("precision rifle (Kyubi iron sights), no parts at all", 22, "Wea_PrecisionRifle", "Items.Preset_Kyubi_Myers", {})
  aimTpp("precision rifle (Kyubi), a scope RAIL in the Scope slot (not a scope type)", 23, "Wea_PrecisionRifle", "Items.Preset_Kyubi_Myers",
    { { slot = "AttachmentSlots.Scope", id = { name = "Items.Rail1", ptype = "Prt_ScopeRail" } } })
  aimTpp("pistol, a scope part sitting in ANOTHER slot (Muzzle)", 24, "Wea_Handgun", "Items.Preset_Pistol",
    { { slot = "AttachmentSlots.PowerModule", id = { name = "Items.Scope1", ptype = "Prt_ShortScope" } } })
  check("scoped false positive: diagnostic line names the part", count(lines, "scope read weapon=Items.Preset_Kyubi_Myers type=Wea_PrecisionRifle scope part=none") >= 1
    and count(lines, "scope part=Items.Rail1 type=Prt_ScopeRail") == 1, table.concat(lines, " | "))
  -- 0.2.5 policy: a SHORT scope keeps third-person aiming (Overture revolver with Items.w_att_scope_short_07, ScopeFOV 26)
  world.scopeFov = 26
  aimTpp("revolver (Overture), real SHORT scope (Prt_ShortScope, ScopeFOV 26)", 31, "Wea_Revolver", "Items.Preset_Overture_Kerry",
    { { slot = "AttachmentSlots.Scope", id = { name = "Items.w_att_scope_short_07", ptype = "Prt_ShortScope" } } })
  aimTpp("assault rifle, real SHORT scope, ScopeFOV 26 (the FOV comes from the short scope)", 32, "Wea_AssaultRifle", "Items.Preset_Rifle",
    { { slot = "AttachmentSlots.Scope", id = { name = "Items.w_att_scope_short_01", ptype = "Prt_ShortScope" } } })
  aimTpp("rifle, generic Prt_Scope part (treated like a short scope)", 33, "Wea_AssaultRifle", "Items.Preset_Rifle",
    { { slot = "AttachmentSlots.Scope", id = { name = "Items.w_att_scope_generic", ptype = "Prt_Scope" } } })
  check("short scope: diagnostic line names the Overture scope", count(lines, "scope read weapon=Items.Preset_Overture_Kerry type=Wea_Revolver scope part=Items.w_att_scope_short_07 type=Prt_ShortScope") == 1)
  -- the setting is applied live on top of the cached part type (same weapon entity, no re-read)
  world.weaponType, world.weaponName, world.weaponEntity, world.slotReads = "Wea_Revolver", "Items.Preset_Overture_Kerry", 34, 0
  world.parts = { { slot = "AttachmentSlots.Scope", id = { name = "Items.w_att_scope_short_07", ptype = "Prt_ShortScope" } } }
  world.psm.UpperBody = 6
  run(handlers, 30)
  check("short scope: default shortScopeFpp=false -> TPP, no scoped log", mod.cfg.shortScopeFpp == false and mod.Desired().perspective == "TPP"
    and count(lines, "aiming scoped weapon=Items.Preset_Overture_Kerry") == 0 and world.slotReads == 1)
  mod.cfg.shortScopeFpp = true
  run(handlers, 30)
  check("short scope: shortScopeFpp=true (live, cached part type) -> FPP, log (scope slot (short)), no re-read",
    mod.Desired().perspective == "FPP" and world.slotReads == 1
    and count(lines, "[utpp_perspective] aiming scoped weapon=Items.Preset_Overture_Kerry -> fpp (scope slot (short))") == 1
    and string.find(mod.Status(), "shortScopeFpp=on", 1, true) ~= nil, mod.Status())
  mod.cfg.shortScopeFpp = false
  world.psm.UpperBody = 0
  run(handlers, 120)
  -- LONG scope on the precision rifle (Kyubi): scoped
  world.scopeFov = 12
  world.weaponType, world.weaponName, world.weaponEntity, world.slotReads = "Wea_PrecisionRifle", "Items.Preset_Kyubi_Myers", 35, 0
  world.parts = { { slot = "AttachmentSlots.Scope", id = { name = "Items.w_att_scope_long_01", ptype = "Prt_LongScope" } } }
  world.psm.UpperBody = 6
  run(handlers, 30)
  check("long scope: Kyubi with Prt_LongScope -> FPP, log (scope slot (long))", mod.Desired().perspective == "FPP"
    and count(lines, "[utpp_perspective] aiming scoped weapon=Items.Preset_Kyubi_Myers -> fpp (scope slot (long))") == 1, mod.Status())
  world.psm.UpperBody = 0
  run(handlers, 120)
  world.scopeFov = nil
  world.weaponType, world.weaponName = "Wea_Handgun", "Items.Preset_Pistol"
  world.parts = { { slot = "AttachmentSlots.Scope", id = { name = "Items.Scope1", ptype = "Prt_LongScope" } } }
  world.weaponEntity, world.slotReads = 12, 0
  world.psm.UpperBody = 6
  run(handlers, 30)
  check("scoped: long scope in the slot + aiming=tpp, aimingScoped=fpp -> FPP, one log line (scope slot (long))",
    mod.Desired().perspective == "FPP" and mod.Desired().detail == "aiming (scoped: scope slot (long))"
    and count(lines, "[utpp_perspective] aiming scoped weapon=Items.Preset_Pistol -> fpp (scope slot (long))") == 1
    and string.find(mod.Status(), "scoped=yes(Items.Preset_Pistol: scope slot (long)) aimingScoped=fpp", 1, true) ~= nil, mod.Status())
  -- the walker itself: finds a mock in a table value, a table key and a closure upvalue, and ignores plain data
  do
    local planted = mock({})
    local up = function() return planted end
    check("handles: the walker detects planted game objects (value / upvalue) and passes plain data",
      #findGameRefs({ a = { b = planted } }) == 1 and #findGameRefs({ f = up }) == 1 and #findGameRefs({ a = { b = "x", c = 1, d = true } }) == 0)
  end
  -- 0.2.5 handles: the cache is plain data, the module state holds no game object (mock tagged) at any point
  do
    local c = mod.scopeCache
    local plainOk = type(c) == "table" and c.key ~= nil
    for _, v in pairs(c or {}) do
      local t = type(v)
      if not (t == "string" or t == "number" or t == "boolean") then plainOk = false end
    end
    check("handles: scope cache entry holds only plain values (key / slot / fov / part / partName)", plainOk
      and c.part == "Prt_LongScope" and c.partName == "Items.Scope1" and c.slot == true, c and c.part)
    local refs = findGameRefs({ mod = mod, onUpdate = handlers.onUpdate, onInit = handlers.onInit, onShutdown = handlers.onShutdown })
    check("handles: no mock game object / userdata reachable from the module state or its handlers while aiming scoped",
      #refs == 0, table.concat(refs, ", "))
    check("handles: lastObs holds only plain values", (function()
      for _, v in pairs(mod.lastObs or {}) do
        local t = type(v)
        if not (t == "string" or t == "number" or t == "boolean") then return false end
      end
      return true end)())
  end
  check("scoped: slot read once for the same weapon entity (cached over 30 polls)", world.slotReads == 1, tostring(world.slotReads))
  world.parts = nil
  run(handlers, 12)
  check("scoped: cache holds until the weapon changes (scope removal not re-read for the same entity)", world.slotReads == 1
    and mod.Desired().perspective == "FPP")
  world.weaponEntity = 13
  run(handlers, 120)
  check("scoped: new weapon entity -> re-read, no scope any more -> TPP (aiming=tpp)", world.slotReads == 2
    and mod.Desired().perspective == "TPP" and string.find(mod.Status(), "scoped=no", 1, true) ~= nil, mod.Status())
  mod.cfg.aimingScoped = "tpp"
  world.psm.UpperBody = 0
  run(handlers, 120)
  world.weaponType, world.weaponName, world.weaponEntity, world.scopeFov = "Wea_SniperRifle", "Items.Preset_Sniper", 14, nil
  world.parts = nil
  mod.cfg.aiming, mod.cfg.aimingScoped = "fpp", "tpp"
  world.psm.UpperBody = 6
  run(handlers, 30)
  check("scoped: sniper type, aiming=fpp but aimingScoped=tpp -> stays TPP, log (sniper type) -> tpp", mod.Desired().perspective == "TPP"
    and count(lines, "[utpp_perspective] aiming scoped weapon=Items.Preset_Sniper -> tpp (sniper type)") == 1, mod.Status())
  world.psm.UpperBody = 0
  run(handlers, 120)
  world.weaponType, world.weaponName, world.weaponEntity, world.scopeFov = "Wea_AssaultRifle", "Items.Preset_Rifle", 15, 35
  mod.cfg.aiming, mod.cfg.aimingScoped = "tpp", "fpp"
  world.psm.UpperBody = 6
  run(handlers, 30)
  check("scoped: ScopeFOV > 0 (rifle, no slot part) + aimingScoped=fpp -> FPP, log (ScopeFOV)", mod.Desired().perspective == "FPP"
    and count(lines, "[utpp_perspective] aiming scoped weapon=Items.Preset_Rifle -> fpp (ScopeFOV)") == 1, mod.Status())
  world.psm.UpperBody, world.scopeFov = 0, 0
  run(handlers, 120)
  -- game exit: set up a long-scoped precision rifle first so the cache holds a part type
  world.weaponType, world.weaponName, world.weaponEntity, world.scopeFov, world.slotReads = "Wea_PrecisionRifle", "Items.Preset_Kyubi_Myers", 40, 12, 0
  world.parts = { { slot = "AttachmentSlots.Scope", id = { name = "Items.Scope1", ptype = "Prt_LongScope" } } }
  mod.cfg.aiming, mod.cfg.aimingScoped = "tpp", "fpp"
  world.psm.UpperBody = 6
  run(handlers, 30)
  -- game exit: onShutdown drops the cache, stops polling and the state still holds no game object
  do
    local calls = world.playerCalls
    local pre = mod.Desired().perspective
    check("onShutdown handler registered", type(handlers.onShutdown) == "function")
    local okS, errS = pcall(handlers.onShutdown)
    local refs = findGameRefs({ mod = mod, onUpdate = handlers.onUpdate, onInit = handlers.onInit, onShutdown = handlers.onShutdown })
    check("onShutdown: no error, scope cache dropped, no game object in the module state", okS and mod.scopeCache == nil
      and mod.scopeDiagLine == nil and #refs == 0 and count(lines, "onShutdown: scope cache and cached handles released") == 1,
      tostring(errS) .. " " .. table.concat(refs, ", "))
    world.psm.UpperBody = 0
    run(handlers, 60)
    check("onShutdown: polling stopped (no game call, no new decision, no update error)", world.playerCalls == calls
      and mod.Desired().perspective == pre and count(lines, "update error") == 0, tostring(world.playerCalls) .. "/" .. tostring(calls))
    check("onShutdown: a second call is harmless", pcall(handlers.onShutdown) and mod.scopeCache == nil)
    handlers.onInit() -- CET reload / new session: onInit re-arms the polling
    world.psm.UpperBody = 6
    run(handlers, 30)
    check("onInit after onShutdown: polling resumes, cache refilled with plain data again", world.playerCalls > calls
      and mod.scopeCache ~= nil and mod.scopeCache.part == "Prt_LongScope" and #findGameRefs({ mod = mod }) == 0)
    -- the player vanishes (load / exit): the cache is dropped at once, nothing is read from the world
    world.noPlayer = true
    run(handlers, 12)
    check("player gone: scope cache dropped on the next poll, no game object left", mod.scopeCache == nil
      and #findGameRefs({ mod = mod }) == 0 and count(lines, "update error") == 0)
    world.noPlayer = false
    world.psm.UpperBody = 0
    run(handlers, 120)
  end
  mod.cfg.aiming, mod.cfg.aimingScoped, world.weaponEntity, world.weaponType, world.weaponName = "fpp", "fpp", 1, nil, nil
  world.weapon = false
  world.gamedata.InteractionData = { terminalInteractionActive = true }
  run(handlers, 120)
  check("UIGameData terminalInteractionActive -> terminal", mod.Desired().detail == "terminal", mod.Status())
  world.gamedata.InteractionData = nil
  world.tags.CyberspacePresence = true
  run(handlers, 12)
  check("CyberspacePresence tag -> outside (cyberspace)", mod.Desired().detail == "cyberspace")
  world.tags.CyberspacePresence = false
  world.statusBroken = true
  run(handlers, 120)
  check("failing read logged once, other inputs still work", count(lines, "read CyberspacePresence FAILED") == 1
    and mod.Desired().perspective == "TPP" and count(lines, "update error") == 0, mod.Status())
  world.ui.IsInMenu = true
  world.psm.Combat = 1
  run(handlers, 30)
  check("menu freezes the decision", mod.Desired().perspective == "TPP" and mod.Desired().frozen == "menu")
  restore()
end

-- 2) utpp_core present: active mode, settings via utpp_core, subscribers notified
do
  local file = os.tmpname()
  registerForEvent, registerInput = nil, nil
  local core = dofile(CORE)
  core.SETTINGS_FILE = file
  core.sink = function() end
  restore()
  local world = newWorld()
  world.cam = { x = 0, y = -1.6, z = 1.7 }
  local ok, mod, handlers, lines = loadInCet(function(n)
    if n == "utpp_core" then return core end
  end, world)
  check("utpp_core: onInit reports active mode and settings via utpp_core",
    ok and count(lines, "onInit 0.2.7: mode=active settings=utpp_core") == 1, table.concat(lines, " | "))
  check("settings section registered in utpp_core", core.settings.sections.perspective ~= nil)
  mod.cfg.combat = "fpp"
  mod.cfg.combatCut = true -- the instant cut is the case under test (the registration default is off since 2026-10-20)
  local got = 0
  mod.Subscribe(function() got = got + 1 end)
  run(handlers, 120)
  check("active: decision TPP computed, subscriber notified", mod.Desired().perspective == "TPP" and got >= 1
    and mod.Desired().mode == "active", tostring(got))
  local before = got
  world.psm.Combat = 1
  run(handlers, 150)
  check("active: combat -> FPP, subscriber notified and decision logged", mod.Desired().perspective == "FPP" and got == before + 1
    and count(lines, "FPP <- combat (InCombat) instant [active]") == 1, table.concat(lines, " | "))
  check("no update errors in active mode", count(lines, "update error") == 0)
  restore()
  os.remove(file)
end

-- 3) 0.2.7 vehicles: the game's vehicle camera (GetMountedVehicle -> GetCameraManager), utpp_camera hold / TPP signal and
-- utpp_body representation through GetMod, the live hold check of Desired(), Game.UTPPSetVehicleTppAnim; nothing read on foot
do
  local world = newWorld()
  local cam = { held = false, tpp = true, getModCalls = 0 }
  local camMod = { VehicleHeld = function() return cam.held end, TppSignalState = function() return cam.tpp end }
  local bodyMod = { GetState = function() return { rep = cam.rep or "tpp" } end }
  local ok, mod, handlers, lines = loadInCet(function(n)
    cam.getModCalls = cam.getModCalls + 1
    if n == "utpp_camera" then return camMod elseif n == "utpp_body" then return bodyMod end
  end, world)
  local anim = {}
  Game.UTPPSetVehicleTppAnim = function(on) anim[#anim + 1] = on end
  check("vehicle: loads with the vehicle reads", ok and type(mod.liveCamHeld) == "function")
  run(handlers, 120)
  local calls0 = cam.getModCalls
  run(handlers, 60)
  check("vehicle: on foot nothing extra is read (no GetMod / vehicle reads per poll)", cam.getModCalls == calls0
    and mod.Desired().perspective == "TPP", tostring(cam.getModCalls - calls0))
  -- driver seat, camera holds, vehicle camera TPPMedium as a number
  local active = 2
  local vehicle = mock({ GetCameraManager = function() return mock({
    GetActivePerspective = function() return active end,
    IsTPPActive = function() return cam.vehTpp end }) end })
  Game.GetMountedVehicle = function() return vehicle end
  world.psm.MountedToVehicle, world.psm.Vehicle, cam.held = true, 1, true
  run(handlers, 12)
  check("vehicle: driver seat + hold + vehicle camera 2 -> TPP (game TPP camera)", mod.Desired().perspective == "TPP"
    and mod.Desired().detail == "vehicle:1 (game TPP camera)" and mod.lastObs.vehActive == 2 and mod.lastObs.camHeld == true,
    mod.Status())
  check("vehicle: Game.UTPPSetVehicleTppAnim(true) for the TPP view", anim[#anim] == true and #anim == 1, #anim)
  active = "DriverCombatMedium" -- enum name text (EnumInt unavailable)
  world.psm.Vehicle = 6
  run(handlers, 12)
  check("vehicle: driver combat, perspective by name (DriverCombatMedium = 5) -> TPP", mod.lastObs.vehActive == 5
    and mod.Desired().detail == "vehicle:6 (game TPP camera)", tostring(mod.lastObs.vehActive))
  active = 0
  run(handlers, 12)
  check("vehicle: vehicle camera FPP (0) -> 'vehicle:6', anim false", mod.Desired().detail == "vehicle:6" and anim[#anim] == false)
  active, cam.vehTpp = {}, true -- unconvertible enum: IsTPPActive decides
  run(handlers, 12)
  check("vehicle: enum unconvertible, IsTPPActive true -> TPP", mod.lastObs.vehActive == nil and mod.lastObs.vehTpp == true
    and mod.Desired().perspective == "TPP")
  -- the hold ends between two polls: Desired() reads the vehicle first person at once
  cam.held = false
  check("vehicle: hold ended before the next poll -> Desired() FPP 'vehicle:6' (live)", mod.Desired().perspective == "FPP"
    and mod.Desired().detail == "vehicle:6")
  run(handlers, 12)
  check("vehicle: next poll without the hold -> 'vehicle:6' decided", mod.st.current.detail == "vehicle:6")
  -- handles: vehicle / camera manager mocks never outlive a poll
  local refs = findGameRefs({ mod = mod, onUpdate = handlers.onUpdate })
  check("vehicle: no vehicle / camera manager object kept in the module state", #refs == 0, table.concat(refs, ", "))
  -- a failing vehicle read is logged once, no update error
  Game.GetMountedVehicle = nil
  cam.held = true
  run(handlers, 30)
  check("vehicle: GetMountedVehicle missing -> logged once, no update error, 'vehicle:6'",
    count(lines, "read vehicle camera (GetMountedVehicle) FAILED") == 1 and count(lines, "update error") == 0
    and mod.Desired().detail == "vehicle:6", table.concat(lines, " | "))
  -- unmount: the anim state resets, on foot again nothing is read
  world.psm.MountedToVehicle, world.psm.Vehicle, cam.held = false, 0, false
  run(handlers, 60)
  calls0 = cam.getModCalls
  run(handlers, 30)
  check("vehicle: after the unmount TPP again, nothing read on foot", mod.Desired().perspective == "TPP" and mod.vehAnim == nil
    and cam.getModCalls == calls0 and count(lines, "update error") == 0, mod.Status())
  restore()
end

print = saved.print
outPrint(string.format("utpp_perspective CET load test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_perspective CET load test failed")

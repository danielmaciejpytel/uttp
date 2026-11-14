-- Offline tests of the UTPP CET module utpp_body (Phase 5N step 3b): body part classifier on the generated vanilla table
-- (female + male V), representation ownership (L1), part toggles with exact restore (L2), garment appearances (L3).
-- Globals set by the runner: UTPP_BODY_PATH (init.lua), UTPP_BODY_DATA_PATH (body_parts.lua), UTPP_CORE_PATH (optional).
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local savedRegister = registerForEvent
registerForEvent = nil -- no CET wiring in the offline test
local M = dofile(UTPP_BODY_PATH)
registerForEvent = savedRegister
local DATA = dofile(UTPP_BODY_DATA_PATH)

check("module loads with its data table", type(M) == "table" and M.data ~= nil and M.data.parts == #DATA.parts,
      M.dataError)
check("table has both genders and every class", M.data.counts["female.fppOnly"] and M.data.counts["female.tppOnly"]
      and M.data.counts["male.fppOnly"] and M.data.counts["male.shared"] and M.data.counts["female.entity"]
      and M.data.counts["male.head"] and true or false)

-- hash of a part in the generated table (the generator checks the table against the extracted vanilla data)
local function H(gender, name, cls, pathPart)
  for _, p in ipairs(DATA.parts) do
    if p[1] == gender and p[3] == name and (cls == nil or p[2] == cls)
       and (pathPart == nil or (p[5] and string.find(p[5], pathPart, 1, true))) then return p[4] end
  end
  error("part not in table: " .. gender .. " " .. name)
end
local function P(gender, name, cls)
  for _, p in ipairs(DATA.parts) do if p[1] == gender and p[3] == name and (cls == nil or p[2] == cls) then return p[5] end end
end

-- ---------------------------------------------------------------- classifier
local C = M.Classify
check("female FPP arms / torso / neck / nails -> fppOnly",
  C("female", "left_arm", H("female", "left_arm")) == "fppOnly" and C("female", "right_arm", H("female", "right_arm")) == "fppOnly"
  and C("female", "t0_000_pwa_fpp__torso", H("female", "t0_000_pwa_fpp__torso")) == "fppOnly"
  and C("female", "n0_000_pwa_fpp__neck0160", H("female", "n0_000_pwa_fpp__neck0160")) == "fppOnly"
  and C("female", "a0_000_pwa_fpp__nails_l", H("female", "a0_000_pwa_fpp__nails_l")) == "fppOnly")
check("female TPP torso (morphtarget) / hq arms -> tppOnly",
  C("female", "t0_000_pwa_base__full", H("female", "t0_000_pwa_base__full")) == "tppOnly"
  and C("female", "a0_001_pwa_base_hq__full", H("female", "a0_001_pwa_base_hq__full")) == "tppOnly")
check("female underwear variant -> tppVariant (never forced on)",
  C("female", "l1_047_pwa_shorts__panties7731", H("female", "l1_047_pwa_shorts__panties7731")) == "tppVariant")
check("female shared legs (cs_flat) -> shared", C("female", "l0_000_pwa_base__cs_flat", H("female", "l0_000_pwa_base__cs_flat")) == "shared")
check("male torso / arms shared, neck fppOnly",
  C("male", "t0_000_pma_base__full", H("male", "t0_000_pma_base__full")) == "shared"
  and C("male", "a0_000_ma_base__full_ag_hq1491", H("male", "a0_000_ma_base__full_ag_hq1491")) == "shared"
  and C("male", "n0_000_pma_fpp__neck2814", H("male", "n0_000_pma_fpp__neck2814")) == "fppOnly")
check("entity components owned by the game (also with an FPP mesh hash)",
  C("female", "body", H("female", "t0_000_pwa_fpp__torso")) == "entity"
  and C("female", "n0_000_pwa_base__full", H("female", "n0_000_pwa_fpp__neck0160")) == "entity"
  and C("male", "torso", nil) == "entity")
check("TPP head components -> head", (function()
  for _, p in ipairs(DATA.parts) do if p[1] == "female" and p[2] == "head" then return C("female", p[3], p[4]) == "head" end end
end)())
local mono = "a0_002_wa__monowire_whip_l_cableless"
local monoF = H("female", mono, "fppOnly")
local monoT = H("female", mono, "tppOnly")
check("same component name, FPP vs TPP mesh -> resolved by hash", C("female", mono, monoF) == "fppOnly" and C("female", mono, monoT) == "tppOnly")
check("same name without a hash -> ambiguous (untouched)", C("female", mono, nil) == "ambiguous")
check("hash only (renamed component) -> class by hash", C("female", "renamed_by_a_mod", H("female", "left_arm")) == "fppOnly")
check("hash shared by fppOnly and ambiguous parts -> ambiguous", C("female", "renamed", H("female", "right_arm")) == "ambiguous")
check("name only (hash unreadable) -> class by unique name", C("female", "left_arm", nil) == "fppOnly")
check("unknown part -> unknown", C("female", "t2_002_jacket__custom", "123456789") == "unknown")
check("path fallback for an unknown \\fpp\\ resource", C("female", "x", "1", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\new_part.mesh") == "fppOnly")
check("female-only name for male V -> unknown", C("male", "left_arm", nil) == "unknown")
check("unknown gender -> unknown", C(nil, "left_arm", nil) == "unknown")
check("hash normalisation: '123ULL', numbers, big numbers, zero",
  M.NormHash("15042179484460848946ULL") == "15042179484460848946" and M.NormHash(12345) == "12345"
  and M.NormHash(2 ^ 60) == nil and M.NormHash("0") == nil and M.NormHash(nil) == nil)
check("ToTPPName: static &FPP, dynamic camera=fpp, no marker",
  M.ToTPPName("l1_001_pants&FPP") == "l1_001_pants&TPP" and M.ToFPPName("l1_001_pants&TPP") == "l1_001_pants&FPP"
  and M.ToTPPName("pants!camera=fpp+gender=w%abc") == "pants!camera=tpp+gender=w%abc"
  and M.ToTPPName("pants!gender=w") == "pants!gender=w" and M.ToTPPName("plain_app") == "plain_app"
  and M.ToFPPName(M.ToTPPName("pants!gender=w+camera=fpp")) == "pants!gender=w+camera=fpp")

-- ---------------------------------------------------------------- mock game
local function comp(gender, name, cls, enabled, pathPart)
  return { name = name, hash = H(gender, name, cls, pathPart), enabled = enabled }
end
local function femaleSet()
  return {
    comp("female", "left_arm", nil, true), comp("female", "right_arm", nil, true),
    comp("female", "t0_000_pwa_fpp__torso", nil, true), comp("female", "n0_000_pwa_fpp__neck0160", nil, true),
    comp("female", "a0_000_pwa_fpp__nails_l", nil, true),
    comp("female", "t0_000_pwa_base__full", nil, false), comp("female", "a0_001_pwa_base_hq__full", nil, true),
    comp("female", "l0_000_pwa_base__cs_flat", nil, true), comp("female", "l1_047_pwa_shorts__panties7731", nil, false),
    { name = "body", hash = H("female", "t0_000_pwa_fpp__torso"), enabled = false },
    { name = "n0_000_pwa_base__full", hash = H("female", "n0_000_pwa_fpp__neck0160"), enabled = false },
    comp("female", "a0_006_wa__launcher_holstered_right", nil, true),
    { name = "t2_002_jacket__custom", hash = "987654321", enabled = true },
    { name = "mod_body_part", hash = "123123123", enabled = true },
  }
end
local function maleSet()
  return {
    comp("male", "t0_000_pma_base__full", nil, true), comp("male", "a0_000_ma_base__full_ag_hq1491", nil, true),
    comp("male", "n0_000_pma_fpp__neck2814", nil, true), comp("male", "a0_001_ma__personal_link_default6377", nil, false),
    { name = "legs", hash = nil, enabled = false }, { name = "t1_unknown_shirt", hash = "55555", enabled = true },
  }
end
local function snapshot(list) local s = {} for i, c in ipairs(list) do s[i] = c.name .. "=" .. tostring(c.enabled) end return table.concat(s, ",") end
local function byName(list, n) for _, c in ipairs(list) do if c.name == n then return c end end end

local function mock(gender, comps, opts)
  opts = opts or {}
  local g = { gender = gender, rep = opts.rep or "fpp", comps = comps, q = {}, act = 0, deact = 0, feat = {}, toggles = 0,
              reason = nil, frozenR = nil, key = "p1", logs = {}, garments = opts.garments or {}, appCalls = 0,
              offsets = true, offsetCalls = 0, cfg = { enabled = true, hideFpp = true, showTpp = true, garments = true,
              garmentOffsets = false } }
  local api = {}
  api.gender = function() return g.gender end
  api.repState = function() return g.rep end
  api.activateRep = function() g.act = g.act + 1; g.q[#g.q + 1] = "act" end
  api.deactivateRep = function() g.deact = g.deact + 1; g.q[#g.q + 1] = "deact" end
  api.feature = function(on) g.feat[#g.feat + 1] = on end
  -- 0.1.11 wrapper variable utpp_foot_tpp: every send recorded with the number of activations/deactivations queued before it
  g.foot, g.noFoot = {}, false
  api.footTpp = function(on)
    if g.noFoot then return false end
    g.foot[#g.foot + 1] = { v = on and 1 or 0, act = g.act, deact = g.deact, feat = #g.feat, key = g.key }
    return true
  end
  api.gameOwnedReason = function() return g.reason end
  api.frozen = function() return g.frozenR end
  api.playerKey = function() return g.key end
  api.components = function() return g.comps end
  api.setEnabled = function(c, on) c.enabled = on; g.toggles = g.toggles + 1; return true end
  api.garments = function() return g.garments end
  api.setAppearance = function(gm, name) gm.appearance = name; g.appCalls = g.appCalls + 1; return true end
  api.setGarmentOffsets = function(on) g.offsets = on; g.offsetCalls = g.offsetCalls + 1; return true end
  api.log = function(m) g.logs[#g.logs + 1] = m end
  -- the game processes queued events between frames
  g.process = function()
    for _, e in ipairs(g.q) do
      if e == "act" and not g.ignoreActivate then g.rep = "tpp" elseif e == "deact" then g.rep = "fpp" end
    end
    g.q = {}
  end
  g.body = M.NewBody(api, function() return g.cfg end)
  g.t = 0
  g.run = function(seconds, dt)
    dt = dt or (1 / 60)
    local n = math.floor(seconds / dt + 0.5)
    for _ = 1, n do g.t = g.t + dt; g.body:Step(g.t); g.process() end
  end
  g.count = function(needle)
    local k = 0
    for _, l in ipairs(g.logs) do if string.find(l, needle, 1, true) then k = k + 1 end end
    return k
  end
  return g
end

-- ---------------------------------------------------------------- female: full cycle
do
  local g = mock("female", femaleSet())
  local before = snapshot(g.comps)
  g.run(0.5)
  check("idle: nothing activated or toggled while TPP is not requested", g.act == 0 and g.toggles == 0 and g.deact == 0)
  g.body:SetWanted(true, "test")
  g.run(0.1)
  check("female: one activation, owner = utpp", g.act == 1 and g.body.owner == "utpp" and g.body.rep == "tpp", g.act)
  check("female: FPP parts hidden", byName(g.comps, "left_arm").enabled == false and byName(g.comps, "right_arm").enabled == false
    and byName(g.comps, "t0_000_pwa_fpp__torso").enabled == false and byName(g.comps, "n0_000_pwa_fpp__neck0160").enabled == false
    and byName(g.comps, "a0_000_pwa_fpp__nails_l").enabled == false)
  check("female: TPP torso enabled, already-visible TPP arms untouched",
    byName(g.comps, "t0_000_pwa_base__full").enabled == true and byName(g.comps, "a0_001_pwa_base_hq__full").enabled == true
    and g.body.stats.shown == 1 and g.body.stats.hidden == 5, g.body.stats.hidden)
  check("female: shared / variant / entity / ambiguous / unknown untouched",
    byName(g.comps, "l0_000_pwa_base__cs_flat").enabled == true and byName(g.comps, "l1_047_pwa_shorts__panties7731").enabled == false
    and byName(g.comps, "body").enabled == false and byName(g.comps, "n0_000_pwa_base__full").enabled == false
    and byName(g.comps, "a0_006_wa__launcher_holstered_right").enabled == true and byName(g.comps, "t2_002_jacket__custom").enabled == true)
  local toggles = g.toggles
  g.run(3.0)
  g.body:SetWanted(true, "again")
  g.run(1.0)
  check("idempotent: repeated requests and watchdog verifications change nothing", g.toggles == toggles and g.act == 1, g.toggles)
  check("unknown parts logged once each", g.count("unknown part left untouched: t2_002_jacket__custom") == 1
    and g.count("unknown part left untouched: mod_body_part") == 1 and g.body.stats.unknown == 2)
  check("feature TPPRepresentation set on activation", g.feat[1] == true)
  -- the game re-enables a hidden FPP arm (e.g. appearance refresh): the watchdog hides it again
  byName(g.comps, "left_arm").enabled = true
  g.run(0.6)
  check("watchdog re-hides a part the game re-enabled", byName(g.comps, "left_arm").enabled == false and g.body.stats.reapplied == 1)
  g.body:SetWanted(false, "test")
  g.run(0.1)
  check("female: back to FPP restores exactly the initial state", snapshot(g.comps) == before, snapshot(g.comps))
  check("female: deactivated once by its owner, then idle", g.deact == 1 and g.body.owner == nil and g.rep == "fpp" and g.body.rep == "idle")
  g.body:SetWanted(false, "again")
  g.run(1.0)
  check("idempotent release: no extra deactivation", g.deact == 1)
  -- 100 cycles
  for _ = 1, 100 do g.body:SetWanted(true, "loop"); g.run(0.1); g.body:SetWanted(false, "loop"); g.run(0.05) end
  check("100 TPP/FPP cycles: state identical, one activation and deactivation per cycle",
    snapshot(g.comps) == before and g.act == 101 and g.deact == 101, g.act .. "/" .. g.deact)
end

-- Read-only evidence for a dress and a separate garment shadow component: do not classify by name
-- or toggle either component. Dump includes the adapter's actual component type/resource field and
-- the appearance of the item slots the module already knows how to inspect.
do
  local dress = { name = "t1_104_pwa_dress__golden_fur", hash = "10608969622358489589", enabled = true,
    class = "entGarmentSkinnedMeshComponent", resourceField = "mesh", path = "base\\dress\\golden_fur.mesh",
    renderMeta = M.ProbeRenderMetadata({ castShadows = "Never", castLocalShadows = "Always",
      skinning = { bindName = "shadow" }, parentTransform = { bindName = "Root" } }) }
  local shadow = { name = "t2_pwa_dress__hanako_shadow", hash = "16804755618972636502", enabled = true,
    class = "entSkinnedMeshComponent", resourceField = "mesh", path = "base\\dress\\hanako_shadow.mesh",
    ref = { castShadows = "Always", castLocalShadows = false, skinning = { bindName = "shadow" },
      parentTransform = { bindName = "Spine3" } } }
  local g = mock("female", { dress, shadow }, { garments = {
    { slot = "AttachmentSlots.Legs", key = "AttachmentSlots.Legs|dress_fixture", appearance = "dress_fixture&TPP" },
  } })
  g.body:SetWanted(true, "diagnostic")
  g.run(0.2)
  local savedBody = M.body
  M.body = g.body
  local count = M.Dump()
  M.body = savedBody
  check("dress diagnostic: unknown garment and shadow remain enabled", dress.enabled and shadow.enabled and g.toggles == 0)
  check("dress diagnostic: unknown log includes existing component metadata and resource path",
    g.count("unknown part left untouched: t1_104_pwa_dress__golden_fur") == 1
    and g.count("componentType=entGarmentSkinnedMeshComponent resourceField=mesh path=base\\dress\\golden_fur.mesh") >= 1)
  check("dress diagnostic: Dump identifies shadow component without toggling it",
    count == 2 and g.count("t2_pwa_dress__hanako_shadow enabled=true hash=16804755618972636502 componentType=entSkinnedMeshComponent resourceField=mesh path=base\\dress\\hanako_shadow.mesh") == 1)
  check("dress diagnostic: Dump reports already-supported garment appearance",
    g.count("garment slot=AttachmentSlots.Legs key=AttachmentSlots.Legs|dress_fixture appearance=dress_fixture&TPP") == 1)
  check("dress diagnostic: reflected shadow and bind metadata appear without classification change",
    g.count("castShadows=Never castLocalShadows=Always skinning.bindName=shadow parentTransform.bindName=Root") >= 2)
  check("dress diagnostic: unknown log lazily probes native component reference",
    g.count("castShadows=Always castLocalShadows=false skinning.bindName=shadow parentTransform.bindName=Spine3") >= 2)
  local absent = M.ProbeRenderMetadata({ castShadows = false })
  check("dress diagnostic: false is distinct from absent reflected fields",
    absent.castShadows == "false" and absent.castLocalShadows == "unavailable:nil"
    and string.find(absent.skinningBindName, "unavailable:", 1, true) == 1)
  local thrown = M.ProbeRenderMetadata(setmetatable({}, { __index = function() error("reflection failed") end }))
  check("dress diagnostic: reflected access errors are reported as unavailable",
    thrown.castShadows == "unavailable:error" and thrown.parentBindName == "unavailable:error")
end

-- 0.1.10 render-plane diagnostics: renderingPlaneAnimationParam + the native registry line (Game.UTPPItemPlaneDump)
do
  local withParam = M.ProbeRenderMetadata({ renderingPlaneAnimationParam = "renderPlaneLeftArm" })
  local noParam = M.ProbeRenderMetadata({ castShadows = "Always" })
  local thrownP = M.ProbeRenderMetadata(setmetatable({}, { __index = function() error("reflection failed") end }))
  check("plane diagnostic: renderingPlaneAnimationParam read, absent and failing reads are told apart",
    withParam.renderPlaneParam == "renderPlaneLeftArm" and noParam.renderPlaneParam == "unavailable:nil"
    and thrownP.renderPlaneParam == "unavailable:error")
  check("plane diagnostic: RenderMetaLine prints the param after the existing fields, no registry note without an index",
    M.RenderMetaLine({ renderMeta = withParam }) == " castShadows=unavailable:nil castLocalShadows=unavailable:nil"
      .. " skinning.bindName=unavailable:error parentTransform.bindName=unavailable:error"
      .. " renderingPlaneAnimationParam=renderPlaneLeftArm", M.RenderMetaLine({ renderMeta = withParam }))

  -- the registry text: header + component lines (the native owns the format; matched by the name as a whole token)
  local DUMP = "itemplane registry=3 reapplied=7 bgInBc=0 weaponOutBc=1\n"
    .. "entSkinnedMeshComponent a0_001_pwa_base_hq__full_shadow req=2 cur=0 player=1\n"
    .. "entSkinnedMeshComponent a0_001_pwa_base_hq__full req=2 cur=0 player=1\n"
    .. "entGarmentSkinnedMeshComponent t2_002_jacket__custom req=2 cur=2 player=1"
  local function planeMock(planeDump)
    local arm = { name = "a0_001_pwa_base_hq__full", hash = "111", enabled = true, class = "entSkinnedMeshComponent",
      resourceField = "mesh", renderMeta = M.ProbeRenderMetadata({ renderingPlaneAnimationParam = "renderPlaneLeftArm" }) }
    local jacket = { name = "t2_002_jacket__custom", hash = "222", enabled = true, class = "entGarmentSkinnedMeshComponent",
      resourceField = "mesh", renderMeta = M.ProbeRenderMetadata({ renderingPlaneAnimationParam = "renderPlane" }) }
    local legs = { name = "l1_unbound_pants", hash = "333", enabled = true, class = "entSkinnedMeshComponent",
      resourceField = "mesh", renderMeta = M.ProbeRenderMetadata({}) }
    local g = mock("female", { arm, jacket, legs })
    g.calls = 0
    g.body.api.planeDump = planeDump and function(max) g.calls = g.calls + 1; g.max = max; return planeDump(max) end or nil
    return g
  end

  -- native present: unknown lines and Dump carry the registry line; one registry read per scan and per Dump
  local g = planeMock(function() return DUMP end)
  g.body:SetWanted(true, "plane")
  g.run(0.2)
  check("plane diagnostic: unknown line quotes the exact registry line (whole-token match, not the _shadow line)",
    g.count("unknown part left untouched: a0_001_pwa_base_hq__full ") == 0 -- the hq arm is tppOnly, not unknown
      and g.count("renderingPlaneAnimationParam=renderPlane itemPlane=[entGarmentSkinnedMeshComponent t2_002_jacket__custom req=2 cur=2 player=1]") == 1,
    table.concat(g.logs, " | "))
  check("plane diagnostic: a part outside the registry -> not-registered",
    g.count("l1_unbound_pants hash=333") == 1 and g.count("renderingPlaneAnimationParam=unavailable:nil itemPlane=not-registered") == 1)
  check("plane diagnostic: two unknown parts in one scan -> one registry read", g.calls == 1, g.calls)
  local savedBody = M.body
  M.body = g.body
  g.calls = 0
  M.Dump()
  M.body = savedBody
  check("plane diagnostic: Dump reads the registry exactly once", g.calls == 1 and g.max == M.PLANE_DUMP_MAX, g.calls)
  check("plane diagnostic: Dump logs the registry header and annotates every component",
    g.count("dump: itemPlane itemplane registry=3 reapplied=7 bgInBc=0 weaponOutBc=1 entries=3") == 1
      and g.count("renderingPlaneAnimationParam=renderPlaneLeftArm itemPlane=[entSkinnedMeshComponent a0_001_pwa_base_hq__full req=2 cur=0 player=1]") == 1
      and g.count("itemPlane=[entGarmentSkinnedMeshComponent t2_002_jacket__custom req=2 cur=2 player=1]") == 2
      and g.count("itemPlane=not-registered") == 2, table.concat(g.logs, " | "))

  -- native missing / raising / wrong type: never an error, the reason is printed instead
  for _, case in ipairs({ { nil, "unavailable:native" }, { function() error("no such function UTPPItemPlaneDump") end,
      "unavailable:error" }, { function() return nil end, "unavailable:nil" } }) do
    local gm = planeMock(case[1])
    local okRun, err = pcall(function() gm.body:SetWanted(true, "plane"); gm.run(0.2) end)
    local savedB = M.body
    M.body = gm.body
    local okDump, n = pcall(M.Dump)
    M.body = savedB
    check("plane diagnostic: registry " .. case[2] .. " -> no error, reason on the unknown and Dump lines",
      okRun and okDump and n == 3 and gm.count("itemPlane=" .. case[2]) >= 2 + 3
        and gm.count("dump: itemPlane " .. case[2]) == 1, tostring(err) .. " " .. tostring(n))
  end
  local idx = M.PlaneIndex("hdr\nx\ny", nil, 2)
  check("plane diagnostic: a full dump (max lines) does not claim not-registered",
    M.PlaneNote(idx, "zzz") == " itemPlane=not-in-dump(max=2)" and M.PlaneNote(idx, "x") == " itemPlane=[x]")

  -- console helper: prints / returns the text, guarded
  local printed = {}
  local savedPrint = print
  print = function(s) printed[#printed + 1] = s end
  local text = M.PlaneDump(8, function(max) return "hdr max=" .. max .. "\nline1" end)
  local why = M.PlaneDump(nil, function() error("native missing") end)
  print = savedPrint
  check("plane diagnostic: PlaneDump returns the text and prints every line",
    text == "hdr max=8\nline1" and printed[1] == "[utpp_body] itemPlane hdr max=8" and printed[2] == "[utpp_body] itemPlane line1",
    table.concat(printed, " | "))
  check("plane diagnostic: PlaneDump with a failing native returns the reason, no error",
    why == "unavailable:error" and printed[3] == "[utpp_body] itemPlane dump unavailable:error")
end

-- 2026-11-01 C3 (render-plane round 2): W lines per worn item (fppVariant), lazy mesh depot path on unknown / Dump lines
do
  local FV = M.FppVariant
  check("C3 fppVariant: _fpp / &FPP / camera=fpp (any case) -> 1",
    FV("h1_001_wa_hat__cowboy_fpp") == 1 and FV("t2_002_jacket__custom&FPP") == 1 and FV("t2_x!camera=fpp+gender=w%12") == 1
      and FV("T2_X!Camera=FPP") == 1 and FV("l1_009_skirt&fpp") == 1)
  check("C3 fppVariant: TPP / plain names -> 0, unreadable -> ?",
    FV("t2_002_jacket__custom&TPP") == 0 and FV("s1_002_shoes!camera=tpp+gender=w%12") == 0 and FV("s1_003_socks_plain") == 0
      and FV("fppfan_jacket") == 0 and FV(nil) == "?" and FV("") == "?")
  check("C3 W line format", M.WornLine({ slot = "OutfitSlots.FingernailsLeft", item = "Items.Nails_01", app = "nails_01!camera=fpp" })
    == "W slot=OutfitSlots.FingernailsLeft item=Items.Nails_01 app=nails_01!camera=fpp fppVariant=1")
  check("C3 slot list: vanilla AttachmentSlots and the EquipmentEx OutfitSlots (FingernailsLeft, TorsoOuter, Hands)", (function()
    local s = {}
    for _, n in ipairs(M.WORN_SLOTS) do s[n] = true end
    return s["AttachmentSlots.Chest"] and s["AttachmentSlots.TppHead"] and s["OutfitSlots.FingernailsLeft"]
      and s["OutfitSlots.TorsoOuter"] and s["OutfitSlots.Hands"] and #M.WORN_SLOTS == 11 + 54
  end)() == true)

  -- PlaneDump + worn source: the dump lines, then a header and one W line per item (UTPP log route); return value unchanged
  local printed = {}
  local savedPrint, savedCore = print, M.core
  print = function(s) printed[#printed + 1] = s end
  M.core = nil
  local worn = function()
    return { { slot = "AttachmentSlots.Chest", item = "Items.Jacket_01", app = "t2_002_jacket__custom&TPP" },
             { slot = "OutfitSlots.FingernailsLeft", item = "Items.Nails_01", app = "nails_01!camera=fpp+gender=w" } }, "slots=65 (fallback 0 + list 65)"
  end
  local text = M.PlaneDump(4, function() return "hdr\nline1" end, worn)
  local why = M.PlaneDump(4, function() error("native missing") end, function() error("no transaction system") end)
  print, M.core = savedPrint, savedCore
  check("C3 PlaneDump: dump lines, then 'W items=2' and one W line per worn item; returns the dump text",
    text == "hdr\nline1" and printed[1] == "[utpp_body] itemPlane hdr" and printed[2] == "[utpp_body] itemPlane line1"
      and printed[3] == "[utpp_body] W items=2 slots=65 (fallback 0 + list 65)"
      and printed[4] == "[utpp_body] W slot=AttachmentSlots.Chest item=Items.Jacket_01 app=t2_002_jacket__custom&TPP fppVariant=0"
      and printed[5] == "[utpp_body] W slot=OutfitSlots.FingernailsLeft item=Items.Nails_01 app=nails_01!camera=fpp+gender=w fppVariant=1",
    table.concat(printed, " | "))
  check("C3 PlaneDump: failing native and failing worn source -> reasons, no error",
    why == "unavailable:error" and printed[6] == "[utpp_body] itemPlane dump unavailable:error"
      and string.find(printed[7] or "", "[utpp_body] W unavailable:error no transaction system", 1, true) == 1, table.concat(printed, " | "))

  -- mesh depot path: the scan path wins; otherwise the lazy probe (path / unresolved / unavailable); no probe -> nothing
  local PN = M.PathNote
  check("C3 PathNote: scan path, probe path, unresolved, unavailable, no probe",
    PN({ path = "base\\a.mesh" }, { meshPath = function() return "other" end }) == " path=base\\a.mesh"
      and PN({}, { meshPath = function() return "base\\evb\\shell.mesh" end }) == " path=base\\evb\\shell.mesh"
      and PN({}, { meshPath = function() return nil, "unresolved" end }) == " path=unresolved"
      and PN({}, { meshPath = function() return nil, "unavailable:ResRef.FromHash" end }) == " path=unavailable:ResRef.FromHash"
      and PN({}, { meshPath = function() error("boom") end }) == " path=unavailable:error"
      and PN({}, {}) == "" and PN({}, nil) == "")
  -- unknown part without a scan path (an EVB shell): the probe runs once, when the part is logged, not on every scan
  local shell = { name = "evb_shell_torso", hash = "424242", enabled = true, class = "entSkinnedMeshComponent", resourceField = "mesh",
    renderMeta = M.ProbeRenderMetadata({}) }
  local g = mock("female", { shell })
  g.probes = 0
  g.body.api.meshPath = function(c) g.probes = g.probes + 1; return "base\\evb\\" .. c.name .. ".mesh" end
  g.body:SetWanted(true, "path")
  g.run(2.0)
  check("C3 unknown part: lazy mesh path on the one-time line, probed once over many scans",
    g.count("unknown part left untouched: evb_shell_torso hash=424242 enabled=true componentType=entSkinnedMeshComponent "
      .. "resourceField=mesh path=base\\evb\\evb_shell_torso.mesh") == 1 and g.probes == 1, g.probes .. " " .. table.concat(g.logs, " | "))
  local savedBody = M.body
  M.body = g.body
  M.Dump()
  M.body = savedBody
  check("C3 Dump: the mesh path probe on the dump line", g.count("evb_shell_torso enabled=true hash=424242 componentType="
    .. "entSkinnedMeshComponent resourceField=mesh path=base\\evb\\evb_shell_torso.mesh") == 1 and g.probes == 2)
end

-- ---------------------------------------------------------------- male
do
  local g = mock("male", maleSet())
  local before = snapshot(g.comps)
  g.body:SetWanted(true, "test")
  g.run(0.2)
  check("male: neck hidden, TPP personal link enabled, torso/arms untouched",
    byName(g.comps, "n0_000_pma_fpp__neck2814").enabled == false and byName(g.comps, "a0_001_ma__personal_link_default6377").enabled == true
    and byName(g.comps, "t0_000_pma_base__full").enabled == true and byName(g.comps, "a0_000_ma_base__full_ag_hq1491").enabled == true
    and byName(g.comps, "legs").enabled == false and g.toggles == 2, g.toggles)
  check("male: unknown shirt logged once", g.count("t1_unknown_shirt") == 1)
  g.body:SetWanted(false, "test")
  g.run(0.1)
  check("male: restored", snapshot(g.comps) == before and g.deact == 1)
end

-- ---------------------------------------------------------------- game-owned representation
do
  local g = mock("female", femaleSet(), { rep = "tpp" }) -- e.g. vehicle TPP camera already active
  g.reason = "vehicleTPP"
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  check("game-owned: no activation, owner = game, parts still verified", g.act == 0 and g.body.owner == "game"
    and byName(g.comps, "left_arm").enabled == false)
  g.body:SetWanted(false, "camera")
  g.run(0.2)
  check("game-owned: never deactivated by UTPP, own part changes undone", g.deact == 0 and g.rep == "tpp"
    and byName(g.comps, "left_arm").enabled == true)
  g.reason = nil
  g.run(0.2)
  check("game-owned without reason: still not deactivated", g.deact == 0 and g.rep == "tpp")
  g.rep = "fpp"
  g.run(0.1)
  check("game released it: owner cleared", g.body.owner == nil)
end
do
  local g = mock("female", femaleSet(), { rep = "tpp" })
  g.body:SetWanted(true, "camera") -- quest / mirror enabled it without a detectable reason
  g.run(0.1)
  g.body:SetWanted(false, "camera")
  g.run(0.1)
  check("representation active before the request (quest): never deactivated", g.deact == 0 and g.act == 0)
end
do
  local g = mock("female", femaleSet())
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  g.reason = "takeover" -- the game starts using the representation while UTPP owns it
  g.body:SetWanted(false, "camera cut")
  g.run(0.2)
  check("UTPP-owned + game reason at release: handed over, not deactivated", g.deact == 0 and g.body.owner == "game"
    and g.body.stats.handovers == 1)
  g.reason = nil
  g.rep = "fpp" -- the game ends the takeover
  g.run(0.1)
  check("after the game released it: no late deactivation", g.deact == 0 and g.body.owner == nil)
end
do
  local g = mock("female", femaleSet())
  g.body:SetExternal("mirror", true)
  g.body:SetWanted(true, "camera")
  g.run(0.3)
  check("external game-owned reason (mirror): UTPP does not activate", g.act == 0)
  g.body:SetExternal("mirror", false)
  g.run(0.1)
  check("reason cleared: activation", g.act == 1)
end

-- ---------------------------------------------------------------- game deactivates / does not activate
do
  local g = mock("female", femaleSet())
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  g.rep = "fpp" -- the game switched it off (e.g. scene end) while UTPP still wants TPP
  g.run(0.2)
  check("game deactivated while wanted: activated again", g.act == 2 and g.rep == "tpp" and g.count("deactivated by the game") == 1)
end
do
  local g = mock("female", femaleSet())
  g.ignoreActivate = true
  g.body:SetWanted(true, "camera")
  g.run(6.0)
  check("activation never confirmed: limited retries, logged once, no part changes",
    g.act == 1 + M.DEFAULTS.maxRetries and g.body.failed and g.count("did not activate") == 1 and g.toggles == 0, g.act)
  g.body:SetWanted(false, "x"); g.run(0.1)
  check("failed activation: released without deactivating a representation that is off", g.deact == 0)
end
do -- TppHead slot unreadable: trust the request after the timeout
  local g = mock("female", femaleSet(), { rep = "unknown" })
  g.process = function() g.q = {} end
  g.body:SetWanted(true, "camera")
  g.run(0.5)
  check("rep unreadable: no part changes before the activation timeout", g.toggles == 0 and g.act == 1)
  g.run(1.0)
  check("rep unreadable: parts applied after the timeout, no retry storm", g.toggles == 6 and g.act == 1, g.toggles)
  g.body:SetWanted(false, "x"); g.run(0.1)
  check("rep unreadable: deactivated by its owner on release", g.deact == 1)
end

-- ---------------------------------------------------------------- 0.1.7 body.retry (male V save load, log 23:37:43-48)
-- The game ignores every activation (a load in progress): 0.1.6 gave up after 3 retries for the rest of the request and a
-- repeated SetWanted(true) changed nothing. With cfg.retry the request is re-armed by a repeated request / a nudge, new
-- rounds come every retryEvery s with cycling strategies (event, event, toggle, item, event ...), and the first request
-- after a player change waits settle s.
do
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  g.items = 0
  g.body.api.equipTppHead = function() g.items = g.items + 1; return true, "mock item" end
  g.ignoreActivate = true
  g.body:SetWanted(true, "camera")
  g.run(0.3)
  check("retry: first request after attach is immediate (no settle without a player change)", g.act == 1, g.act)
  g.run(4.0)
  check("retry: 3 retries, then 'next round in 5 s' (not 'giving up')", g.act == 4 and g.body.failed == true
    and g.count("next round in 5 s") == 1 and g.count("giving up") == 0, g.act)
  check("retry: activate lines carry the strategy and the slot state", g.count("via event slot=fpp attempt=1") == 1
    and g.count("(retry 1) via event slot=fpp attempt=2") == 1 and g.count("(retry 2) via toggle slot=fpp attempt=3") == 1
    and g.count("(retry 3) via item slot=fpp attempt=4 (head item equipped: mock item)") == 1, table.concat(g.logs, " | "))
  check("retry: toggle strategy queued a deactivate first, item strategy equipped the head", g.deact == 1 and g.items == 1
    and g.body.stats.toggles == 1 and g.body.stats.items == 1)
  local actBefore = g.act
  g.run(4.0) -- 4.3 s after the give-up at ~4.0 s: not yet
  check("retry: no round before retryEvery", g.act == actBefore, g.act)
  g.run(1.5)
  check("retry: round 1 after 5 s, logged with the slot, attempt 5 = event", g.act == actBefore + 1 and g.body.rounds == 1
    and g.count("representation retry round 1 (every 5 s): slot=fpp") == 1 and g.count("via event slot=fpp attempt=5") == 1, g.act)
  g.run(10.0)
  check("retry: rounds keep coming (never final)", g.body.rounds >= 2 and g.body.stats.rounds == g.body.rounds, g.body.rounds)
  -- a repeated request re-arms the budget at once
  g.run(1.0)
  g.body.failed = true; g.body.retryAt = g.t + 100
  g.body:SetWanted(true, "camera again")
  check("retry: repeated SetWanted(true) while given up re-arms (logged)", g.body.failed == false and g.body.retries == 0
    and g.count("TPP body requested again (camera again): rep=fpp owner=utpp, was given up - retries re-armed") == 1)
  -- a nudge re-arms too and opens a fast verification window
  g.run(0.2)
  g.body.failed = true; g.body.retryAt = g.t + 100
  local ok = g.body:Nudge(g.t, "aim enter")
  check("retry: Nudge re-arms a given-up request and is logged", ok == true and g.body.failed == false
    and g.count("nudge (aim enter): rep=fpp owner=utpp retries=0 - parts verified now and for 1.0 s, given-up request re-armed") == 1)
  check("retry: status line carries the counters", string.find(g.body:StatusLine(), "retry=true attempt=", 1, true) ~= nil
    and string.find(g.body:StatusLine(), "rearms=2 nudges=1 toggles=", 1, true) ~= nil, g.body:StatusLine())
  -- the game finally accepts: parts applied, no more rounds
  g.ignoreActivate = false
  g.run(2.0)
  local rounds = g.body.rounds
  g.run(12.0)
  check("retry: once active no further rounds, parts hidden", g.rep == "tpp" and g.body.rounds == rounds
    and byName(g.comps, "left_arm").enabled == false and g.body.failed == false)
  g.body:SetWanted(false, "x"); g.run(0.1)
  check("retry: released normally", g.body.owner == nil and byName(g.comps, "left_arm").enabled == true)
end
do -- nudge = fast verification window (parts re-checked every 0.1 s for 1 s), silent when retry is off
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  byName(g.comps, "left_arm").enabled = true -- the game re-resolved the appearance at the aim enter
  g.body:Nudge(g.t, "aim enter")
  g.run(0.05)
  check("nudge: part re-hidden within 0.05 s (not the 0.5 s watchdog)", byName(g.comps, "left_arm").enabled == false)
  byName(g.comps, "left_arm").enabled = true
  g.run(0.15)
  check("nudge: fast window still open after 0.15 s", byName(g.comps, "left_arm").enabled == false)
  g.run(1.0)
  byName(g.comps, "left_arm").enabled = true
  g.run(0.15)
  check("nudge: window closed after 1 s (slow watchdog again)", byName(g.comps, "left_arm").enabled == true)
  g.run(0.5)
  check("nudge: slow watchdog still re-hides", byName(g.comps, "left_arm").enabled == false)
  local g2 = mock("female", femaleSet())
  g2.body:SetWanted(true, "camera")
  g2.run(0.2)
  check("retry off: Nudge is a no-op", g2.body:Nudge(g2.t, "aim") == false and g2.count("nudge") == 0)
  g2.ignoreActivate = true; g2.rep = "fpp"
  g2.run(6.0)
  g2.body:SetWanted(true, "again")
  g2.run(6.0)
  check("retry off: 0.1.6 behaviour - gave up, repeated request a no-op, no rounds", g2.body.failed == true
    and g2.count("giving up until the next request") == 1 and g2.count("requested again") == 0 and g2.body.rounds == 0)
end
do -- settle after a player change (save load): the first activation waits 0.5 s with retry on, immediate with retry off
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  g.key = "p2"; g.rep = "fpp"; g.comps = femaleSet()
  g.run(0.3)
  check("settle: no activation 0.3 s after the player change", g.act == 1 and g.body.settleUntil ~= nil, g.act)
  g.run(0.4)
  check("settle: activated after 0.5 s, parts applied", g.act == 2 and byName(g.comps, "left_arm").enabled == false, g.act)
  -- unfreeze from noplayer: the same wait
  g.frozenR = "noplayer"; g.run(0.2)
  g.key = "p3"; g.rep = "fpp"; g.comps = femaleSet()
  g.frozenR = nil
  g.run(0.3)
  check("settle: unfreeze (noplayer) + new entity waits too", g.act == 2, g.act)
  g.run(0.4)
  check("settle: then activates", g.act == 3, g.act)
  local g2 = mock("female", femaleSet())
  g2.body:SetWanted(true, "camera")
  g2.run(0.2)
  g2.key = "p2"; g2.rep = "fpp"; g2.comps = femaleSet()
  g2.run(0.1)
  check("retry off: immediate activation after the player change (0.1.6)", g2.act == 2, g2.act)
end

-- ---------------------------------------------------------------- restore does not fight the game
do
  local g = mock("female", femaleSet())
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  byName(g.comps, "t0_000_pwa_base__full").enabled = false -- the game changed a part we enabled
  local idx
  for i, c in ipairs(g.comps) do if c.name == "right_arm" then idx = i end end
  table.remove(g.comps, idx) -- an FPP part vanished (appearance change)
  g.body:SetWanted(false, "camera")
  g.run(0.1)
  check("restore skips parts changed or removed by the game", byName(g.comps, "t0_000_pwa_base__full").enabled == false
    and byName(g.comps, "left_arm").enabled == true and g.body.stats.restoreSkipped == 2, g.body.stats.restoreSkipped)
end

-- ---------------------------------------------------------------- freeze, disable, player change, gender
do
  local g = mock("female", femaleSet())
  g.frozenR = "photomode"
  g.body:SetWanted(true, "camera")
  g.run(1.0)
  check("photo mode: frozen, no writes", g.act == 0 and g.toggles == 0)
  g.frozenR = nil
  g.run(0.2)
  check("after photo mode: full verification", g.act == 1 and byName(g.comps, "left_arm").enabled == false)
end
do
  local g = mock("female", femaleSet())
  local before = snapshot(g.comps)
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  g.cfg.enabled = false -- setting switched off
  g.run(0.2)
  check("disabled while active: parts restored and own representation released", snapshot(g.comps) == before and g.deact == 1)
  g.run(1.0)
  check("disabled: stays inactive", g.act == 1 and g.deact == 1)
end
do
  local g = mock("female", femaleSet())
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  g.key = "p2"; g.rep = "fpp"; g.comps = femaleSet() -- save loaded: new entity
  g.run(0.2)
  check("player entity changed: records dropped, representation requested again",
    g.body.stats.resets == 1 and g.act == 2 and byName(g.comps, "left_arm").enabled == false)
end
do
  local g = mock(nil, femaleSet())
  g.body:SetWanted(true, "camera")
  g.run(1.0)
  check("unknown gender: L1 only, L2 skipped with one log line", g.act == 1 and g.toggles == 0 and g.count("gender unknown") == 1)
end
do
  local g = mock("female", femaleSet())
  g.cfg.hideFpp, g.cfg.showTpp = false, false
  g.body:SetWanted(true, "camera")
  g.run(0.3)
  check("hideFpp/showTpp off: representation only", g.act == 1 and g.toggles == 0)
end

-- ---------------------------------------------------------------- L3 garments
do
  local garments = {
    { slot = "AttachmentSlots.Legs", key = "Legs|Items.Pants", appearance = "l1_001_pants__red&FPP" },
    { slot = "AttachmentSlots.Feet", key = "Feet|Items.Shoes", appearance = "s1_002_shoes!camera=fpp+gender=w%12" },
    { slot = "AttachmentSlots.Feet", key = "Feet|Items.Socks", appearance = "s1_003_socks_plain" },
  }
  local g = mock("female", femaleSet(), { garments = garments })
  g.cfg.garmentOffsets = true
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  check("L3: FPP garment variants switched to TPP, plain names untouched",
    garments[1].appearance == "l1_001_pants__red&TPP" and garments[2].appearance == "s1_002_shoes!camera=tpp+gender=w%12"
    and garments[3].appearance == "s1_003_socks_plain" and g.appCalls == 2)
  check("L3: garment offsets disabled once", g.offsets == false and g.offsetCalls == 1)
  g.run(2.0)
  check("L3: idempotent", g.appCalls == 2 and g.offsetCalls == 1)
  g.body:SetWanted(false, "camera")
  g.run(0.1)
  check("L3: restored to the FPP variants and offsets on",
    garments[1].appearance == "l1_001_pants__red&FPP" and garments[2].appearance == "s1_002_shoes!camera=fpp+gender=w%12"
    and g.offsets == true and g.offsetCalls == 2)
  -- item swapped while in TPP: the old record is dropped, the new item is not "restored" to a stale name
  g.body:SetWanted(true, "camera")
  g.run(0.2)
  garments[1].key, garments[1].appearance = "Legs|Items.Skirt", "l1_009_skirt&FPP"
  g.run(0.6)
  g.body:SetWanted(false, "camera")
  g.run(0.1)
  check("L3: swapped item handled (new item switched and restored)", garments[1].appearance == "l1_009_skirt&FPP")
end
do
  local garments = { { slot = "AttachmentSlots.Legs", key = "Legs|Items.Pants", appearance = "l1_001_pants&FPP" } }
  local g = mock("female", femaleSet(), { garments = garments })
  g.cfg.garments = false
  g.body:SetWanted(true, "camera")
  g.run(0.3)
  check("L3 off: garments and offsets untouched", garments[1].appearance == "l1_001_pants&FPP" and g.offsetCalls == 0)
end

-- ---------------------------------------------------------------- module API + utpp_core settings
do
  local g = mock("female", femaleSet())
  M.body = g.body
  g.body.cfgFn = M.Config
  M.core = nil
  M.settings.enabled = true
  check("API without utpp_core: enabled by default", M.Enabled() == true)
  M.settings.enabled = false
  local ok, why = M.SetTppBody(true, "camera")
  check("API: setting off -> SetTppBody refused", ok == false and why == "disabled" and M.IsTppBody() == false)
  M.settings.enabled = true
  ok = M.SetTppBody(true, "camera")
  check("API: setting on -> SetTppBody accepted", ok == true and M.IsTppBody() == true)
  M.SetGameOwned("mirror", true)
  check("API: SetGameOwned / GetState / Status", M.GetState().wanted == true and type(M.Status()) == "string"
    and string.find(M.Status(), "wanted=true", 1, true) ~= nil and g.body.external.mirror == true)
  M.SetGameOwned("mirror", false)
  M.SetTppBody(false)
  M.settings.enabled = true
end
if UTPP_CORE_PATH then
  local savedR = registerForEvent
  registerForEvent = nil
  local core = dofile(UTPP_CORE_PATH)
  registerForEvent = savedR
  core.Save = function() return true end -- no file writes in the test
  check("utpp_core: settings registered", M.RegisterSettings(core) == true and core.settings.defs["body.impl"] ~= nil
    and core.settings.defs["body.impl"].type == "bool" and core.settings.defs["body.impl"].default == true
    and core.settings.defs["body.impl"].label == "Use UTPP body visibility" and core.settings.sections.body ~= nil)
  check("utpp_core: default on", M.Config().enabled == true)
  core.Set("body.impl", false)
  check("utpp_core: switched off", M.Config().enabled == false)
  core.Set("body.impl", true)
  check("utpp_core: switched on again", M.Config().enabled == true)
  core.Set("body.garmentOffsets", true)
  check("utpp_core: option values reach the config", M.Config().garmentOffsets == true and M.Config().hideFpp == true)
  check("utpp_core: body.retry registered, default on, reaches the config", core.settings.defs["body.retry"] ~= nil
    and core.settings.defs["body.retry"].default == true and M.Config().retry == true)
  core.Set("body.retry", false)
  check("utpp_core: body.retry off reaches the config", M.Config().retry == false)
  core.Set("body.retry", true)
  check("module: Nudge API without a body attached refuses", (function() local b = M.body; M.body = nil; local ok = M.Nudge("x"); M.body = b; return ok == false end)())
  check("utpp_core: head guard setting registered, default on", core.settings.defs["body.headGuard"] ~= nil
    and M.HeadGuardEnabled() == true)
  core.Set("body.headGuard", false)
  check("utpp_core: head guard setting off reaches the guard", M.HeadGuardEnabled() == false)
  M.SetHeadGuard(true)
  check("head guard A/B override (not saved) wins over the setting", M.HeadGuardEnabled() == true
    and core.Get("body.headGuard") == false)
  M.SetHeadGuard(nil)
  core.Set("body.headGuard", true)
  M.core = nil
end

-- ---------------------------------------------------------------- head guard (vehicle mount / unmount, vehicle windows)
-- 0.1.2 world model (log utpp-0.5.5-user-play 02:55:18.44-19.83): the TppHead slot of AttachmentSlots.TppHead; the game
-- processes a queued representation event after opts.eventLag s (default 0.1) unless ignored; a TPP head item is
-- "spawning" for opts.spawn s (default 0) before it shows in the slot; opts.eventThrows = the event cannot be created
-- (0.1.1 in game: "cannot create ActivateTPPRepresentationEvent"); opts.resetEvery = the game puts the FPP head back
-- that long after every repair (a game that keeps winning: the guard must stop, not fight).
local function guardWorld(opts)
  opts = opts or {}
  local w = { slot = opts.slot or "tpp", events = 0, items = 0, feat = 0, released = 0, logs = {}, pendingAt = nil,
              spawnAt = nil, resetAt = nil, frozenR = nil, now = 0 }
  -- 0.1.3 options: hiddenSpawn = the slot keeps reading its old state while our item spawns (game log 03:31:24: polls
  -- read "fpp" after "add=true, spawning=true"); headItemState reports the spawn; equipping again restarts the spawn
  -- (remove + add); opts.activate(w) replaces the representation call (bridge tests); opts.guard = guard options.
  local api = {
    headSlot = function() if w.spawnAt and not opts.hiddenSpawn then return "spawning" end return w.slot end,
    headItemState = function()
      if opts.noItemState then error("no item state") end
      if w.spawnAt then return "spawning" end
      return w.slot == "tpp" and "inslot" or "none"
    end,
    activateRep = function()
      w.events = w.events + 1
      if opts.activate then return opts.activate(w) end
      if opts.eventThrows then error("cannot create gameActivateTPPRepresentationEvent") end
      if not opts.ignoreEvent then w.pendingAt = w.now + (opts.eventLag or 0.1) end
      return "gameActivateTPPRepresentationEvent.new()"
    end,
    feature = function(on) w.feat = w.feat + 1 end,
    equipTppHead = function()
      w.items = w.items + 1
      w.equipTimes = w.equipTimes or {}
      w.equipTimes[#w.equipTimes + 1] = w.now
      if opts.itemFails then return false, "AddItemToSlot false" end
      if (opts.spawn or 0) > 0 then w.spawnAt = w.now + opts.spawn else w.slot = "tpp" end
      return true, "Items.PlayerWaTppHead"
    end,
    releaseRep = function() w.released = w.released + 1; return true end,
    frozen = function() return w.frozenR end,
    log = function(m) w.logs[#w.logs + 1] = m end,
  }
  w.on = true
  w.g = M.NewHeadGuard(api, function() return w.on end, opts.guard)
  w.step = function(t, reason, hint, keepWindow)
    w.now = t
    if w.pendingAt and t >= w.pendingAt then
      w.slot, w.pendingAt = "tpp", nil
      if opts.resetEvery then w.resetAt = t + opts.resetEvery end
    end
    if w.spawnAt and t >= w.spawnAt then
      w.slot, w.spawnAt = "tpp", nil
      if opts.resetEvery then w.resetAt = t + opts.resetEvery end
    end
    if w.resetAt and t >= w.resetAt then w.slot, w.resetAt = "fpp", nil end
    if not keepWindow then w.g:SetWindow(reason, hint) end
    w.g:Step(t)
  end
  return w
end
local function countLog(logs, needle)
  local n = 0
  for _, l in ipairs(logs) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end
-- loses the head at a random sub-frame phase, returns the time without a TPP head (60 fps, 2 s)
local function lossTime(opts, phase)
  local w = guardWorld(opts)
  local t = (phase or 0) / 60
  for _ = 1, 10 do t = t + 1 / 60; w.step(t, "unmount") end
  w.slot = "fpp"
  local lostAt, backAt = t, nil
  for _ = 1, 120 do t = t + 1 / 60; w.step(t, "unmount"); if not backAt and w.slot == "tpp" and not w.spawnAt then backAt = t end end
  return backAt and (backAt - lostAt) or 99, w, t
end
do -- unmount: the game leaves the FPP head; the representation event repairs it within 0.3 s
  local worst, w, t = 0, nil, 0
  for phase = 0, 5 do
    local d
    d, w, t = lossTime({}, phase * 0.17)
    worst = math.max(worst, d)
  end
  check("head guard: missing head repaired by the representation event within 0.30 s (was ~2 s)", worst <= 0.30
    and w.events == 1 and w.items == 0, string.format("worst %.2f s, events %d items %d", worst, w.events, w.items))
  check("head guard: slot changes and the repair logged (with the creation method)", countLog(w.logs, "head slot tpp -> fpp") == 1
    and countLog(w.logs, "-> TPP representation requested (gameActivateTPPRepresentationEvent.new())") == 1
    and countLog(w.logs, "head back after") == 1)
  w.step(t + 0.1, nil)
  check("head guard: window end logged once", countLog(w.logs, "head guard closed (unmount, window ended)") == 1
    and not w.g.active and w.released == 0)
end
do -- the event cannot be created (0.1.1 in game): the head item follows in the same step
  local d, w = lossTime({ eventThrows = true })
  check("head guard: event creation fails -> TPP head item at once, head back within 0.30 s", d <= 0.30 and w.items == 1
    and w.g.stats.eventFails == 1 and countLog(w.logs, "requested FAILED: ") == 1, string.format("%.2f s", d))
end
do -- the event is ignored by the game: the head item as the second repair, still within 0.30 s
  local worst, w = 0, nil
  for phase = 0, 5 do
    local d
    d, w = lossTime({ ignoreEvent = true }, phase * 0.17)
    worst = math.max(worst, d)
  end
  check("head guard: event ignored -> TPP head item (repair 2), head back within 0.30 s", worst <= 0.30
    and w.events == 1 and w.items == 1, string.format("worst %.2f s", worst))
end
do -- a spawning TPP head is waited for, never stacked
  local d, w = lossTime({ eventThrows = true, spawn = 0.4 })
  check("head guard: spawning TPP head waited for (one item, no second repair)", w.items == 1 and w.events == 1
    and (w.g.stats.spawnWaits or 0) > 0 and d < 0.7, string.format("%.2f s, items %d", d, w.items))
end
do -- nothing works: bounded, then gives up until the next window
  local w = guardWorld({ ignoreEvent = true, itemFails = true })
  local t = 0
  w.step(t, "exit"); w.slot = "empty"
  for _ = 1, 300 do t = t + 1 / 60; w.step(t, "exit") end
  check("head guard: max 4 repairs per loss, then gives up (logged once)", w.g.stats.repairs == 4 and w.g.stats.gaveUp == 1
    and countLog(w.logs, "giving up until the next") == 1 and w.events == 2 and w.items == 2)
  t = t + 1
  w.step(t, nil)
  for _ = 1, 60 do t = t + 1 / 60; w.step(t, "unmount") end
  check("head guard: next window tries again", w.g.stats.repairs > 4)
end
do -- the game takes the head away again after every repair: max 8 repairs per window, no endless fight
  local w = guardWorld({ resetEvery = 0.2 })
  local t = 0
  w.step(t, "unmount"); w.slot = "fpp"
  for _ = 1, 600 do t = t + 1 / 60; w.step(t, "unmount") end
  check("head guard: game keeps resetting the slot -> 8 repairs in the window, then stops", w.g.stats.repairs == 8
    and w.g.stats.gaveUp == 1 and countLog(w.logs, "giving up until the next window") == 1,
    string.format("repairs %d", w.g.stats.repairs))
end
do -- the game restores the head itself: no repair, no fight
  -- 0.1.8: window 'unmount' (the 0.05 s delay); the 'exit' window repairs on the first poll (delay 0, MOTO-1 tests below)
  local w = guardWorld()
  local t = 0
  w.step(t, "unmount"); w.slot = "fpp"
  t = t + 0.03; w.step(t, "unmount"); w.slot = "tpp"
  for _ = 1, 60 do t = t + 1 / 60; w.step(t, "unmount") end
  check("head guard: swap shorter than the delay left to the game (the game equips its head)", w.events == 0 and w.items == 0)
  w = guardWorld({ slot = "tpp" })
  for i = 1, 300 do w.step(i / 60, "unmount") end
  check("head guard: TPP head present -> only reads (0.05 s polls), never writes", w.events == 0 and w.items == 0
    and w.g.stats.polls >= 90 and w.g.stats.polls <= 110, tostring(w.g.stats.polls))
end
do -- FPP is never broken: a representation the guard activated is released when UTPP no longer wants the TPP body, only then
  local _, w, t = lossTime({})
  w.step(t + 0.05, nil, "fpp")
  check("head guard: window ends with the TPP body no longer wanted -> the guard's representation released once", w.released == 1
    and countLog(w.logs, "representation activated by the guard released") == 1)
  _, w, t = lossTime({})
  w.step(t + 0.05, nil, nil)
  check("head guard: window ends otherwise (ride / time over) -> nothing released", w.released == 0)
  w = guardWorld({ slot = "tpp" })
  for i = 1, 30 do w.step(i / 60, "unmount") end
  w.step(1, nil, "fpp")
  check("head guard: TPP body no longer wanted without a guard repair -> nothing released (the game owns it)", w.released == 0)
  _, w, t = lossTime({ eventThrows = true })
  w.step(t + 0.05, nil, "fpp")
  check("head guard: only the head item used -> nothing released", w.released == 0)
end
do -- no window / other head item / unreadable / switched off / frozen -> no writes
  local w = guardWorld({ slot = "fpp" })
  for i = 1, 60 do w.step(i / 60, nil) end
  check("head guard: no window -> no reads, no writes", w.g.stats.polls == 0 and w.events == 0)
  w = guardWorld({ slot = "other" })
  for i = 1, 60 do w.step(i / 60, "unmount") end
  check("head guard: other head item never touched", w.events == 0 and w.items == 0)
  w = guardWorld({ slot = "unknown" })
  for i = 1, 60 do w.step(i / 60, "unmount") end
  check("head guard: unreadable slot never touched", w.events == 0 and w.items == 0)
  w = guardWorld({ slot = "fpp" }); w.on = false
  for i = 1, 60 do w.step(i / 60, "unmount") end
  check("head guard: setting off -> no repair, logged once", w.events == 0 and countLog(w.logs, "switched off (setting)") == 1)
  w = guardWorld({ slot = "fpp" }); w.frozenR = "photomode"
  for i = 1, 60 do w.step(i / 60, "unmount") end
  check("head guard: photo mode -> no repair", w.events == 0)
end

-- ---------------------------------------------------------------- 0.1.3 redscript bridge + head item spawn wait
-- Mock CET environments for M.QueueRep / M.RedsSlot. bridge: "global" (Game.UTPPQueueTppRepresentation answers),
-- "system" (only the UTPPBodyBridge scriptable system answers; the global returns nil like an unresolved CET call),
-- "none" (utpp_body.reds not installed). cet: "fails" (game 03:31:24.256: no name can be created) | "works".
-- answer = what the bridge returns (default true; false = no player). calls = log of every call.
local function bridgeEnv(bridge, cet, answer, onQueue)
  local env = { calls = {} }
  if answer == nil then answer = true end
  local function rec(s) env.calls[#env.calls + 1] = s end
  local player = { QueueEvent = function(_, ev) rec("QueueEvent " .. tostring(ev and ev.type)); if onQueue then onQueue(ev and ev.type) end end }
  local system = { QueueTppRepresentation = function(_, on) rec("system " .. tostring(on)); if onQueue then onQueue(on) end; return answer end,
                   TppHeadSlotState = function() rec("system slot"); return "fpp" end }
  local G = { GetPlayer = function() return player end }
  if bridge == "global" then
    G.UTPPQueueTppRepresentation = function(on) rec("global " .. tostring(on)); if onQueue then onQueue(on) end; return answer end
    G.UTPPTppHeadSlotState = function() rec("global slot"); return "fpp" end
  elseif bridge == "system" then
    G.UTPPQueueTppRepresentation = function() return nil end
    G.UTPPTppHeadSlotState = function() return nil end
  end
  G.GetScriptableSystemsContainer = function()
    return { Get = function(_, name) if bridge == "system" and name == "UTPPBodyBridge" then return system end return nil end }
  end
  env.Game = G
  local known = { gameActivateTPPRepresentationEvent = true, gameDeactivateTPPRepresentationEvent = true }
  env.NewObject = function(n)
    local base = string.gsub(n, "^handle:", "")
    if cet ~= "works" or not known[base] then error("Unknown type " .. n) end
    return { type = base }
  end
  return env
end

if type(M.QueueRep) == "function" then
  local env = bridgeEnv("global", "fails")
  local ok, how = pcall(M.QueueRep, true, env)
  check("bridge: Game.UTPPQueueTppRepresentation available -> used first, called with true, nothing created in CET",
    ok and how == "Game.UTPPQueueTppRepresentation(true)" and #env.calls == 1 and env.calls[1] == "global true", tostring(how))
  env = bridgeEnv("global", "fails")
  ok, how = pcall(M.QueueRep, false, env)
  check("bridge: deactivation passes false", ok and how == "Game.UTPPQueueTppRepresentation(false)" and env.calls[1] == "global false",
    tostring(how))
  env = bridgeEnv("system", "fails")
  ok, how = pcall(M.QueueRep, true, env)
  check("bridge: global unresolved in CET -> UTPPBodyBridge scriptable system", ok
    and how == "UTPPBodyBridge:QueueTppRepresentation(true)" and env.calls[1] == "system true", tostring(how))
  env = bridgeEnv("none", "works")
  ok, how = pcall(M.QueueRep, true, env)
  check("bridge: .reds not installed, CET creation works -> old route (RTTI name) + QueueEvent on the player", ok
    and how == "NewObject(gameActivateTPPRepresentationEvent)" and env.calls[1] == "QueueEvent gameActivateTPPRepresentationEvent",
    tostring(how))
  env = bridgeEnv("none", "fails")
  ok, how = pcall(M.QueueRep, true, env)
  check("bridge: nothing works (game without the .reds) -> error lists every route", not ok
    and string.find(tostring(how), "Game.UTPPQueueTppRepresentation error", 1, true) ~= nil
    and string.find(tostring(how), "UTPPBodyBridge error", 1, true) ~= nil
    and string.find(tostring(how), "cannot create gameActivateTPPRepresentationEvent", 1, true) ~= nil, tostring(how))
  env = bridgeEnv("global", "fails", false)
  ok, how = pcall(M.QueueRep, true, env)
  check("bridge: answer false (no player) is not success", not ok
    and string.find(tostring(how), "Game.UTPPQueueTppRepresentation=false", 1, true) ~= nil, tostring(how))
  local s, rhow = M.RedsSlot(bridgeEnv("global", "fails"))
  local s2, rhow2 = M.RedsSlot(bridgeEnv("system", "fails"))
  local okN, errN = pcall(M.RedsSlot, bridgeEnv("none", "fails"))
  check("bridge: slot state via the global / the system; missing .reds reported", s == "fpp" and rhow == "Game.UTPPTppHeadSlotState"
    and s2 == "fpp" and rhow2 == "UTPPBodyBridge" and not okN and string.find(tostring(errN), "utpp_body.reds installed?", 1, true) ~= nil,
    tostring(errN))
else
  check("bridge: M.QueueRep present (0.1.3)", false)
end

-- 0.1.4: default env in the CET mod sandbox (log utpp-0.5.7-user-play t=186.50: "attempt to index local 'env' (a nil value)").
-- The chunk runs with setfenv in a table that has Game / NewObject but NO _G (as in CET); QueueRep is called WITHOUT env.
do
  local realG = _G
  local function sandboxLoad(withGetfenv)
    local calls = {}
    local sbGame = { UTPPQueueTppRepresentation = function(on) calls[#calls + 1] = on; return true end }
    local sb = setmetatable({ Game = sbGame, NewObject = function() error("no CET creation") end },
      { __index = function(_, k) if k == "_G" then return nil end; if k == "getfenv" and not withGetfenv then return nil end
        return realG[k] end })
    local chunk = assert(loadfile(UTPP_BODY_PATH))
    setfenv(chunk, sb)
    local mod = chunk()
    return mod, sb, sbGame, calls
  end
  local mod, sb, sbGame, calls = sandboxLoad(true)
  -- negative control: the old default (_G) is nil in this sandbox and indexing it fails exactly like in the log
  local oldF = setfenv(function() return _G.Game end, sb)
  local okOld, errOld = pcall(oldF)
  check("sandbox: _G is nil in the CET-like sandbox, so the old default env.Game failed (negative control)",
    not okOld and string.find(tostring(errOld), "attempt to index", 1, true) ~= nil, tostring(errOld))
  check("sandbox: M.DefaultEnv().Game is the sandbox Game", mod.DefaultEnv().Game == sbGame)
  local ok, how = pcall(mod.QueueRep, true)
  check("sandbox: QueueRep(true) without env reaches the bridge", ok and how == "Game.UTPPQueueTppRepresentation(true)"
    and #calls == 1 and calls[1] == true, tostring(how))
  ok, how = pcall(mod.QueueRep, false)
  check("sandbox: QueueRep(false) without env passes false", ok and how == "Game.UTPPQueueTppRepresentation(false)"
    and calls[2] == false, tostring(how))
  local okS, errS = pcall(mod.RedsSlot)
  check("sandbox: RedsSlot() without env does not fail on a nil env (bridge answer decides)",
    okS or string.find(tostring(errS), "bridge not reachable", 1, true) ~= nil, tostring(errS))
  local okC, errC = pcall(mod.CreateNative, mod.EVENT_NAMES.activate)
  check("sandbox: CreateNative without env reports the tried methods, not a nil env",
    not okC and string.find(tostring(errC), "cannot create", 1, true) ~= nil, tostring(errC))
  -- Game is looked up live: replaced after load (CET re-creates globals) -> the new one is used
  local calls2 = {}
  sb.Game = { UTPPQueueTppRepresentation = function(on) calls2[#calls2 + 1] = on; return true end }
  ok, how = pcall(mod.QueueRep, true)
  check("sandbox: Game replaced after load -> live lookup", ok and #calls2 == 1 and #calls == 2, tostring(how))
  -- no getfenv and no _G: values captured at load (fallback)
  local calls3 = {}
  local fb = M.MakeEnv(nil, { Game = { UTPPQueueTppRepresentation = function(on) calls3[#calls3 + 1] = on; return true end } })
  ok, how = pcall(M.QueueRep, true, fb)
  check("sandbox: MakeEnv(nil, captured) -> captured Game used", ok and how == "Game.UTPPQueueTppRepresentation(true)"
    and #calls3 == 1, tostring(how))
  check("module: VERSION 0.1.11", M.VERSION == "0.1.11")
end

-- Exit as in log utpp-0.5.6-user-play 03:31:22.62-24.89 (first exit): window "exit" with the TPP head, unmount at
-- 1.58 s -> slot "empty", 0.05 s later the game puts the FPP head in. bridge = "global" | "none" (game without the
-- .reds: CET creation fails as in the log); spawn = how long our head item spawns; the slot keeps reading "fpp"
-- meanwhile (hiddenSpawn). -> time without a TPP head after the unmount, world
local function exitFromLog(bridge, spawn, guardOpts)
  local w
  local q = M.QueueRep or function() error("cannot create gameActivateTPPRepresentationEvent (module without bridge)") end
  w = guardWorld({ spawn = spawn, hiddenSpawn = true, guard = guardOpts, activate = function(world)
    local how = q(true, bridgeEnv(bridge, "fails", true, function() world.pendingAt = world.now + 0.1 end))
    return how
  end })
  local t = 0
  for _ = 1, 95 do t = t + 1 / 60; w.step(t, "exit") end -- 1.58 s with the TPP head
  local lostAt, fppAt, backAt = t, t + 0.05, nil
  w.slot = "empty"
  for _ = 1, 180 do
    t = t + 1 / 60
    if fppAt and t >= fppAt then
      if w.slot == "empty" then w.slot = "fpp" end
      fppAt = nil
    end
    w.step(t, "unmount")
    if not backAt and w.slot == "tpp" then backAt = t end
  end
  return backAt and (backAt - lostAt) or 99, w
end
do
  local d, w = exitFromLog("global", 0.4)
  check("log scenario, bridge available: head back <= 0.30 s through Game.UTPPQueueTppRepresentation, no head item",
    d <= 0.30 and w.events == 1 and w.items == 0
    and countLog(w.logs, "slot tpp -> empty") == 1 and countLog(w.logs, "(Game.UTPPQueueTppRepresentation(true))") == 1,
    string.format("%.2f s, events %d items %d", d, w.events, w.items))
  d, w = exitFromLog("none", 0.4)
  local addsInSpawn = 0
  for _, et in ipairs(w.equipTimes or {}) do if et > (w.equipTimes[1] or 0) and et < w.equipTimes[1] + 0.45 then addsInSpawn = addsInSpawn + 1 end end
  check("log scenario, no bridge, spawn 0.4 s: one head item, not removed / added again, head back <= 0.60 s",
    d <= 0.60 and w.items == 1 and addsInSpawn == 0 and w.g.stats.gaveUp == 0 and w.g.stats.itemWaits > 0
    and countLog(w.logs, "requested FAILED: no route for the TPP representation") == 1,
    string.format("%.2f s, items %d, adds during the spawn %d, gaveUp %d", d, w.items, addsInSpawn, w.g.stats.gaveUp))
  -- negative control inside the model: 0.1.2 cadence (no item wait) - every re-add restarts the spawn: 4 items as in
  -- the log, gives up, the head only arrives with the last item (>= 0.8 s). NOTE: in game even the last item never
  -- arrived (head back only 5 s later with the game's own representation) - the model does not reproduce that part.
  d, w = exitFromLog("none", 0.4, { itemWait = 0, itemWaitMax = 0 })
  check("log scenario negative control: 0.1.2 cadence (re-add every 0.15 s) restarts the spawn 4x, gives up, head late",
    d >= 0.8 and w.items == 4 and w.g.stats.gaveUp == 1, string.format("%.2f s, items %d", d, w.items))
end
do -- the spawn is cancelled at once (item state "none"): next repair only after itemWait, bounded to 4 per loss
  local w = guardWorld({ hiddenSpawn = true, eventThrows = true, spawn = 99 })
  w.g.api.headItemState = function() return "none" end
  local t = 0
  w.step(t, "unmount"); w.slot = "fpp"
  for _ = 1, 240 do t = t + 1 / 60; w.step(t, "unmount") end
  local minGap = 99
  for i = 2, #(w.equipTimes or {}) do minGap = math.min(minGap, w.equipTimes[i] - w.equipTimes[i - 1]) end
  check("head item cancelled at once: repairs >= 0.5 s apart, 4 per loss, then gives up", w.items == 4 and minGap >= 0.5
    and w.g.stats.gaveUp == 1 and countLog(w.logs, "head item after") == 4, string.format("items %d, min gap %.2f s", w.items, minGap))
end
do -- spawn reported forever: waits up to itemWaitMax (1.5 s), then the next repair
  local w = guardWorld({ hiddenSpawn = true, eventThrows = true, spawn = 99 })
  local t = 0
  w.step(t, "unmount"); w.slot = "fpp"
  for _ = 1, 150 do t = t + 1 / 60; w.step(t, "unmount") end
  local gap = (w.equipTimes and w.equipTimes[2] or 99) - (w.equipTimes and w.equipTimes[1] or 0)
  check("head item spawning longer than 1.5 s: next repair after itemWaitMax", #(w.equipTimes or {}) == 2 and gap >= 1.5 and gap < 1.6,
    string.format("gap %.2f s", gap))
end
do -- item state unreadable: plain minimum wait
  local w = guardWorld({ hiddenSpawn = true, eventThrows = true, spawn = 99, noItemState = true })
  local t = 0
  w.step(t, "unmount"); w.slot = "fpp"
  for _ = 1, 50 do t = t + 1 / 60; w.step(t, "unmount") end
  local gap = (w.equipTimes and w.equipTimes[2] or 99) - (w.equipTimes and w.equipTimes[1] or 0)
  check("head item state unreadable: next repair after the 0.5 s minimum", gap >= 0.5 and gap < 0.6, string.format("gap %.2f s", gap))
end

-- ---------------------------------------------------------------- 0.1.2 native object creation (CET naming)
-- Mock of CET's class lookup as established for these classes: RTTI names are known (RED4ext.SDK
-- Generated\game\ActivateTPPRepresentationEvent.hpp NAME = "gameActivateTPPRepresentationEvent"), the redscript alias
-- ("ActivateTPPRepresentationEvent") is not a CET global and NewObject rejects it - which reproduces the in-game error.
local function cetEnv(withNew)
  local known = { gameActivateTPPRepresentationEvent = true, gameDeactivateTPPRepresentationEvent = true,
                  gameAnimFeature_TPPRepresentation = true }
  local env = {}
  env.NewObject = function(n)
    local base = string.gsub(n, "^handle:", "")
    if not known[base] then error("Unknown type " .. n) end
    return { type = base }
  end
  if withNew then
    for n in pairs(known) do env[n] = { new = function() return { type = n } end } end
  end
  return env
end
if type(M.CreateNative) == "function" then
  local o, how = M.CreateNative(M.EVENT_NAMES.activate, cetEnv(true))
  check("CET: activation event created by its RTTI name (.new())", o and o.type == "gameActivateTPPRepresentationEvent"
    and how == "gameActivateTPPRepresentationEvent.new()", how)
  o, how = M.CreateNative(M.EVENT_NAMES.activate, cetEnv(false))
  check("CET: without the .new() global -> NewObject(RTTI name)", o and o.type == "gameActivateTPPRepresentationEvent"
    and how == "NewObject(gameActivateTPPRepresentationEvent)", how)
  o = M.CreateNative(M.EVENT_NAMES.deactivate, cetEnv(true))
  local f = M.CreateNative(M.EVENT_NAMES.feature, cetEnv(false))
  check("CET: deactivation event and anim feature by RTTI name", o.type == "gameDeactivateTPPRepresentationEvent"
    and f.type == "gameAnimFeature_TPPRepresentation")
  local ok, err = pcall(M.CreateNative, { "ActivateTPPRepresentationEvent", "handle:ActivateTPPRepresentationEvent" }, cetEnv(true))
  check("CET negative control: the 0.1.1 names (alias only) fail like in game", not ok
    and string.find(tostring(err), "cannot create ActivateTPPRepresentationEvent", 1, true) ~= nil, tostring(err))
else
  check("CET: M.CreateNative present (0.1.2)", false)
end

-- ---------------------------------------------------------------- 0.1.6 FPP entity meshes in TPP (earlier standalone test 2)
-- earlier test log, item 8 (two shadows: the FPP shadow-only proxy of the player entity
-- t0_000_p?a_base__full_shadow, castShadows Always, stays on next to the TPP body) and item 5 (metal of the hand
-- cyberware through the body: entity meshes a0_004__weapon_grip_* / a0_000_nomad_base_fists / fpp_lights with
-- renderingPlaneAnimationParam = renderPlane, drawn on the weapon plane; Extracted\game\json\vanilla\player_wa_fpp.ent.json)
check("0.1.6 table: shadow proxies -> shadowProxy (female body + head, male body, male head without a resource)",
  C("female", "t0_000_pwa_base__full_shadow", H("female", "t0_000_pwa_base__full_shadow")) == "shadowProxy"
  and C("female", "head_shadowmesh", H("female", "head_shadowmesh")) == "shadowProxy"
  and C("male", "t0_000_pma_base__full_shadow", H("male", "t0_000_pma_base__full_shadow")) == "shadowProxy"
  and C("male", "head_shadowmesh", nil) == "shadowProxy")
check("0.1.6 table: weapon-plane entity meshes -> renderPlaneFpp (grip device / decal 02 / fists / lights, male grip)",
  C("female", "a0_004__weapon_grip_device", H("female", "a0_004__weapon_grip_device", "renderPlaneFpp")) == "renderPlaneFpp"
  and C("female", "a0_004__weapon_grip_decal_02", H("female", "a0_004__weapon_grip_decal_02")) == "renderPlaneFpp"
  and C("female", "a0_000_nomad_base_fists", H("female", "a0_000_nomad_base_fists")) == "renderPlaneFpp"
  and C("female", "fpp_lights", H("female", "fpp_lights")) == "renderPlaneFpp"
  and C("male", "a0_004__weapon_grip_device", H("male", "a0_004__weapon_grip_device", "renderPlaneFpp")) == "renderPlaneFpp")
check("0.1.6 table: same name as the TPP photo-mode decal -> resolved by hash, name alone = ambiguous",
  C("female", "a0_004__weapon_grip_decal_01", H("female", "a0_004__weapon_grip_decal_01", "renderPlaneFpp")) == "renderPlaneFpp"
  and C("female", "a0_004__weapon_grip_decal_01", H("female", "a0_004__weapon_grip_decal_01", "tppOnly")) == "tppOnly"
  and C("female", "a0_004__weapon_grip_decal_01", nil) == "ambiguous")
check("0.1.6 table: other entity / appearance parts unchanged (body entity, appearance grip fppOnly, injection entity)",
  C("female", "body", H("female", "t0_000_pwa_fpp__torso")) == "entity"
  and C("female", "a0_004__weapon_grip_device0226", H("female", "a0_004__weapon_grip_device0226", "fppOnly")) == "fppOnly"
  and C("female", "a0_008_wa__fpp_right_q001_injection_mark", nil) == "entity")
check("0.1.6 table: counts per gender (2 shadow proxies, 5 weapon-plane meshes each)",
  M.data.counts["female.shadowProxy"] == 2 and M.data.counts["female.renderPlaneFpp"] == 5
  and M.data.counts["male.shadowProxy"] == 2 and M.data.counts["male.renderPlaneFpp"] == 5)

local function entitySet()
  local s = femaleSet()
  s[#s + 1] = { name = "t0_000_pwa_base__full_shadow", hash = H("female", "t0_000_pwa_base__full_shadow"), enabled = true }
  s[#s + 1] = { name = "head_shadowmesh", hash = H("female", "head_shadowmesh"), enabled = false }
  s[#s + 1] = { name = "a0_004__weapon_grip_device", hash = H("female", "a0_004__weapon_grip_device", "renderPlaneFpp"), enabled = true }
  s[#s + 1] = { name = "a0_004__weapon_grip_decal_01", hash = H("female", "a0_004__weapon_grip_decal_01", "renderPlaneFpp"), enabled = true }
  s[#s + 1] = { name = "a0_004__weapon_grip_decal_01", hash = H("female", "a0_004__weapon_grip_decal_01", "tppOnly"), enabled = false }
  s[#s + 1] = { name = "a0_000_nomad_base_fists", hash = H("female", "a0_000_nomad_base_fists"), enabled = false }
  return s
end
local function decal(list, cls)
  local h = H("female", "a0_004__weapon_grip_decal_01", cls)
  for _, c in ipairs(list) do if c.name == "a0_004__weapon_grip_decal_01" and c.hash == h then return c end end
end
do
  local g = mock("female", entitySet())
  local before = snapshot(g.comps)
  g.body:SetWanted(true, "test")
  g.run(0.1)
  check("0.1.6 TPP: shadow proxy disabled, enabled weapon-plane meshes disabled",
    byName(g.comps, "t0_000_pwa_base__full_shadow").enabled == false and byName(g.comps, "a0_004__weapon_grip_device").enabled == false
    and decal(g.comps, "renderPlaneFpp").enabled == false)
  check("0.1.6 TPP: meshes the game left disabled untouched (head shadow, fists); TPP photo-mode decal shown (tppOnly)",
    byName(g.comps, "head_shadowmesh").enabled == false and byName(g.comps, "a0_000_nomad_base_fists").enabled == false
    and decal(g.comps, "tppOnly").enabled == true and g.body.stats.entityHidden == 3, g.body.stats.entityHidden)
  check("0.1.6 TPP: one log line per corrected component",
    g.count("TPP: shadow proxy t0_000_pwa_base__full_shadow disabled") == 1
    and g.count("TPP: weapon-plane FPP part a0_004__weapon_grip_device disabled") == 1
    and g.count("TPP: weapon-plane FPP part a0_004__weapon_grip_decal_01 disabled") == 1, table.concat(g.logs, " | "))
  byName(g.comps, "a0_004__weapon_grip_device").enabled = true -- the game enables the grip again (weapon draw)
  g.run(3.0)
  check("0.1.6 TPP: re-enabled grip hidden again by the watchdog, no second log line",
    byName(g.comps, "a0_004__weapon_grip_device").enabled == false and g.body.stats.reapplied == 1
    and g.count("a0_004__weapon_grip_device disabled") == 1)
  g.body:SetWanted(false, "test")
  g.run(0.1)
  check("0.1.6 FPP: exact initial state restored (proxy and grip on again, photo-mode decal off)", snapshot(g.comps) == before,
    snapshot(g.comps))
  check("0.1.6 FPP: one restore line per entity mesh", g.count("FPP: t0_000_pwa_base__full_shadow restored (enabled=true)") == 1
    and g.count("FPP: a0_004__weapon_grip_device restored") == 1 and g.count("FPP: a0_004__weapon_grip_decal_01 restored") == 1)
  -- setting off: the proxy stays (A/B), the weapon-plane meshes still follow hideFpp
  g.cfg.shadowProxy = false
  g.body:SetWanted(true, "test")
  g.run(0.1)
  check("0.1.6 body.shadowProxy off: proxy left on, grip still hidden",
    byName(g.comps, "t0_000_pwa_base__full_shadow").enabled == true and byName(g.comps, "a0_004__weapon_grip_device").enabled == false)
  g.body:SetWanted(false, "test")
  g.run(0.1)
  check("0.1.6 body.shadowProxy off: FPP restore exact", snapshot(g.comps) == before)
  -- game-owned representation (vehicle TPP camera): the proxy is hidden too (TPP body visible), restored on release
  local g2 = mock("female", entitySet(), { rep = "tpp" })
  g2.reason = "vehicleTPP"
  g2.body:SetWanted(true, "camera")
  g2.run(0.1)
  check("0.1.6 game-owned TPP: proxy hidden while the game owns the representation",
    g2.body.owner == "game" and byName(g2.comps, "t0_000_pwa_base__full_shadow").enabled == false and g2.deact == 0)
  -- negative control: the 0.1.5 behaviour (proxy = entity, never touched) keeps two shadows
  local saved = M.data
  local old = {}
  for _, p in ipairs(DATA.parts) do
    local q = { p[1], p[2], p[3], p[4], p[5], p[6] }
    if M.ENTITY_HIDE[q[2]] then q[2] = "entity" end
    old[#old + 1] = q
  end
  M.LoadData({ parts = old, generator = "gen_bodyparts.py 1.0 (test)" })
  local g3 = mock("female", entitySet())
  g3.body:SetWanted(true, "test")
  g3.run(0.1)
  check("0.1.6 negative control: 1.0 table (entity) leaves the proxy and the grip on in TPP (the reported bug)",
    byName(g3.comps, "t0_000_pwa_base__full_shadow").enabled == true and byName(g3.comps, "a0_004__weapon_grip_device").enabled == true)
  M.data = saved
  -- male V: proxy and grip device
  local gm = mock("male", { { name = "t0_000_pma_base__full_shadow", hash = H("male", "t0_000_pma_base__full_shadow"), enabled = true },
    { name = "a0_004__weapon_grip_device", hash = H("male", "a0_004__weapon_grip_device", "renderPlaneFpp"), enabled = true },
    { name = "t0_000_pma_base__full", hash = H("male", "t0_000_pma_base__full"), enabled = true } })
  gm.body:SetWanted(true, "test")
  gm.run(0.1)
  check("0.1.6 male V: proxy and grip device hidden, shared torso untouched",
    gm.comps[1].enabled == false and gm.comps[2].enabled == false and gm.comps[3].enabled == true)
end

-- ---------------------------------------------------------------- 0.1.6 vehicle windows
-- PSM Vehicle sequence of the motorcycle exit in earlier test log (14:14:11.42 mount start state 4 + mounted,
-- 14:14:13.32 seated state 1, 14:14:16.75 exit start state 4, unmount later). vw.state / vw.mounted are what the game reports.
local function vehWorld(opts)
  opts = opts or {}
  local vw = { state = 0, mounted = false, wanted = true, unmounts = {}, logs = {}, reads = 0 }
  local api = { vehicle = function() vw.reads = vw.reads + 1; return { state = vw.state, mounted = vw.mounted } end,
                log = function(m) vw.logs[#vw.logs + 1] = m end }
  vw.v = M.NewVehicleWindows(api, function() return vw.wanted end, function(now) vw.unmounts[#vw.unmounts + 1] = now end, opts)
  vw.t = 0
  vw.windows = {}
  vw.run = function(seconds, fn)
    local dt = 1 / 60
    for _ = 1, math.floor(seconds / dt + 0.5) do
      vw.t = vw.t + dt
      local r, h = vw.v:Step(vw.t)
      vw.last, vw.hint = r, h
      if (r or "-") ~= (vw.windows[#vw.windows] or "-") then vw.windows[#vw.windows + 1] = r or "-" end
      if fn then fn(vw.t, r, h) end
    end
  end
  vw.count = function(needle) return countLog(vw.logs, needle) end
  return vw
end
do
  local vw = vehWorld()
  vw.run(0.5)
  check("veh windows: on foot -> no window", vw.last == nil and vw.v.phase == "idle")
  vw.state, vw.mounted = 4, true
  vw.run(0.1)
  check("veh windows: mount start (PSM Transition + mounted) -> window mount", vw.last == "mount" and vw.v.phase == "mount")
  vw.run(1.8)
  vw.state = 1
  vw.run(0.1)
  check("veh windows: seated (Driving) -> window closed (the vehicle camera owns the ride)", vw.last == nil and vw.v.phase == "ride")
  vw.run(3.0)
  vw.state = 4
  vw.run(0.1)
  check("veh windows: exit start (Transition after Driving) -> window exit", vw.last == "exit" and vw.v.phase == "exit")
  vw.run(1.4)
  vw.state, vw.mounted = 0, false
  local tUnmount = vw.t
  vw.run(0.1)
  check("veh windows: unmount -> window unmount, body re-check called once", vw.last == "unmount" and #vw.unmounts == 1
    and math.abs(vw.unmounts[1] - tUnmount) < 0.05, #vw.unmounts)
  vw.run(2.8)
  check("veh windows: unmount window still open at 2.9 s", vw.last == "unmount")
  vw.run(0.3)
  check("veh windows: unmount window closed after 3 s, idle again", vw.last == nil and vw.v.phase == "idle")
  check("veh windows: window sequence mount, -, exit, unmount, -", table.concat(vw.windows, ",") == "mount,-,exit,unmount,-",
    table.concat(vw.windows, ","))
  check("veh windows: one log line per window change", vw.count("head guard window nil -> mount (vehicle") == 1
    and vw.count("head guard window mount -> nil (vehicle") == 1 and vw.count("head guard window nil -> exit (vehicle") == 1
    and vw.count("head guard window exit -> unmount (vehicle") == 1 and vw.count("head guard window unmount -> nil (vehicle") == 1,
    table.concat(vw.logs, " | "))
  check("veh windows: phase lines with the PSM state", vw.count("vehicle ride -> exit (exit start, PSM vehicle=4 mounted=true)") == 1
    and vw.count("vehicle exit -> after (unmount, PSM vehicle=0 mounted=false)") == 1)
  local reads = vw.reads
  vw.run(2.0)
  vw.wanted = false
  vw.run(2.0)
  check("veh windows: UTPP in FPP and idle -> no game reads", vw.reads == reads + 120, vw.reads - reads)
  vw.state, vw.mounted = 4, true
  vw.run(1.0)
  check("veh windows: UTPP in FPP -> no window even in a vehicle", vw.last == nil and vw.v.phase == "idle")
end
do
  local vw = vehWorld()
  vw.state, vw.mounted = 1, true -- TPP wanted only after the player was seated (load in a car)
  vw.run(0.2)
  check("veh windows: already seated -> ride, no mount window", vw.last == nil and vw.v.phase == "ride")
  vw.state = 4
  vw.run(0.5)
  vw.state = 1
  vw.run(0.1)
  check("veh windows: exit aborted (back to Driving) -> window closed, ride", vw.last == nil and vw.v.phase == "ride")
  vw.state = 4
  vw.run(0.2)
  vw.wanted = false
  vw.run(0.1)
  check("veh windows: UTPP leaves TPP during the exit -> window closed with hint fpp", vw.last == nil and vw.hint == "fpp")
  vw.wanted = true
  vw.state, vw.mounted = 0, false
  vw.run(0.1)
  vw.state, vw.mounted = 4, true
  vw.run(0.1)
  check("veh windows: mounting again inside the unmount window -> mount window", vw.last == "mount" and vw.v.phase == "mount")
end
-- head guard driven by the windows (exit as in the log: the game drops the TPP head 0.47 s after the exit start)
local function exitWindows(withWindows)
  local w = guardWorld({ eventLag = 0.1 })
  local vw = vehWorld()
  vw.state, vw.mounted = 1, true
  local lostAt, backAt = nil, nil
  vw.run(0.5, function(t, r, h) if withWindows then w.g:SetWindow(r, h) end; w.step(t, nil, nil, true) end)
  local exitAt = vw.t
  vw.state = 4
  vw.run(2.0, function(t, r, h)
    if lostAt == nil and t >= exitAt + 0.47 then w.slot, lostAt = "fpp", t end
    if t >= exitAt + 0.9 then vw.state, vw.mounted = 0, false end
    if withWindows then w.g:SetWindow(r, h) end
    w.step(t, nil, nil, true)
    if lostAt and backAt == nil and w.slot == "tpp" then backAt = t end
  end)
  return w, vw, lostAt, backAt
end
do
  local w, vw, lostAt, backAt = exitWindows(true)
  check("exit: guard window exit opened by utpp_body itself", countLog(w.logs, "head guard open (exit)") == 1,
    table.concat(w.logs, " | "))
  check("exit: head lost 0.47 s after the exit start repaired by the representation event",
    backAt ~= nil and backAt - lostAt < 0.3 and w.events >= 1 and countLog(w.logs, "head back after") == 1,
    tostring(backAt and (backAt - lostAt)))
  check("exit: window exit -> unmount at the unmount", countLog(w.logs, "head guard exit -> unmount") == 1)
  local w0, _, _, back0 = exitWindows(false)
  check("exit negative control (0.1.5: no internal windows): no guard, head stays FPP",
    back0 == nil and w0.events == 0 and countLog(w0.logs, "head guard open") == 0)
end
do
  -- body: the unmount re-arms a representation request that had given up during the ride
  local g = mock("female", femaleSet())
  g.ignoreActivate = true
  g.body:SetWanted(true, "camera")
  g.run(5.0)
  check("unmount re-check: precondition - gave up after the retries", g.body.failed == true and g.act == 4, g.act)
  g.ignoreActivate = false
  local act = g.act
  check("unmount re-check: Body:VehicleUnmount re-arms (logged)", g.body:VehicleUnmount(g.t) == true and g.body.failed == false
    and g.count("vehicle unmount: TPP representation re-checked now") == 1)
  g.run(0.1)
  check("unmount re-check: representation requested again and active", g.act == act + 1 and g.rep == "tpp" and g.body.owner == "utpp")
  g.body:SetWanted(false, "x")
  g.run(0.1)
  check("unmount re-check: not wanted -> no-op", g.body:VehicleUnmount(g.t) == false)
  -- fast verification inside a window: a part the game re-enables is hidden again within 0.1 s
  g.body:SetWanted(true, "camera")
  g.run(0.6)
  g.body.fastVerify = true
  byName(g.comps, "left_arm").enabled = true
  g.run(0.12)
  check("window fast verify: re-enabled FPP arm hidden within 0.12 s", byName(g.comps, "left_arm").enabled == false)
  g.body.fastVerify = false
  g.run(0.3)
  byName(g.comps, "left_arm").enabled = true
  g.run(0.12)
  check("outside a window the 0.5 s watchdog applies (arm still on after 0.12 s)", byName(g.comps, "left_arm").enabled == true)
end
do
  -- module wiring: M.StepVehicle feeds the guard
  local saved = { M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled }
  M.core, M.settings.enabled = nil, true
  local vw = { state = 4, mounted = true }
  local api = { vehicle = function() return { state = vw.state, mounted = vw.mounted } end, log = function() end,
                headSlot = function() return "tpp" end }
  M.Attach(api)
  M.body.wanted = true
  check("module: VehicleWanted true (TPP wanted, body enabled)", M.VehicleWanted() == true)
  check("module: StepVehicle -> guard internal window mount", M.StepVehicle(1.0) == "mount" and M.guard.want == "mount"
    and M.body.fastVerify == true)
  M.body.wanted = false
  check("module: TPP body not wanted -> no window, close hint fpp", M.VehicleWanted() == false
    and M.StepVehicle(1.1) == nil and M.guard.want == nil and M.guard.hint == "fpp")
  M.body.wanted = true
  M.body.frozen = "photomode"
  check("module: frozen -> vehicle step skipped", M.StepVehicle(1.2) == nil)
  M.body.frozen = nil
  check("module: Status carries the vehicle window state", string.find(M.Status(), "vehicle=", 1, true) ~= nil)
  M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled = saved[1], saved[2], saved[3], saved[4], saved[5], saved[6]
  check("module: body.shadowProxy default off (no utpp_core)", M.Config().shadowProxy == false)
end

-- ---------------------------------------------------------------- 0.1.8 vehicle batch (MOTO-1 C, CAR-7 (2), CAM-11)
-- Recording 2026-10-22 14:57: the strategy counter climbed over the session (attempt=9 toggle, 10 item, 11 event, 12
-- toggle, 13 item at five bike exits), so most exits were repaired by toggle / item; the head vanished or floated.
-- Mutants are made from the shipped source text (anchor must exist exactly once).
local function bodyMutant(from, to)
  local f = assert(io.open(UTPP_BODY_PATH, "rb"))
  local src = f:read("*a"); f:close()
  local a, b = src:find(from, 1, true)
  assert(a and not src:find(from, b + 1, true), "mutation anchor must exist exactly once: " .. from)
  local savedR = registerForEvent
  registerForEvent = nil
  local m = assert(loadstring(src:sub(1, a - 1) .. to .. src:sub(b + 1), "=utpp_body_mutant"))()
  registerForEvent = savedR
  return m
end
local function lastActivate(logs)
  for i = #logs, 1, -1 do if string.find(logs[i], "representation activate", 1, true) then return logs[i] end end
end
-- 9 activations while the game reads fpp, one Step reading tpp, then the game drops it again: the next activation
local function attemptScenario(Mod)
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  g.body = Mod.NewBody(g.body.api, function() return g.cfg end)
  g.body.api.equipTppHead = function() return true, "mock item" end
  g.body:SetWanted(true, "camera")
  g.ignoreActivate = true
  g.run(1 / 60) -- first Step: player key captured, activation 1 (the game reads fpp)
  for _ = 2, 9 do g.t = g.t + 1 / 60; g.body:Activate(g.t, false) end
  local attempt9 = g.body.attempt
  g.q, g.rep = {}, "tpp"
  g.t = g.t + 1 / 60; g.body:Step(g.t)
  g.rep = "fpp"
  g.ignoreActivate = true
  g.run(1 / 60)
  return g, attempt9
end
do -- MOTO-1 C (1): attempt resets after rep ok
  local g, a9 = attemptScenario(M)
  local line = lastActivate(g.logs)
  check("attempt resets after rep ok: 9 attempts, one 'tpp' read, the next loss activates via event attempt=1", a9 == 9
    and g.count("deactivated by the game - activating again") == 1 and line ~= nil
    and string.find(line, "via event slot=fpp attempt=1", 1, true) ~= nil, tostring(line))
  local Mm = bodyMutant("      self.attempt = 0 -- a success ends the attempt series", "      -- mutant: no reset")
  local gm = attemptScenario(Mm)
  local lineM = lastActivate(gm.logs)
  check("attempt resets after rep ok (mutation: no reset -> attempt=10 via item, the 0.1.7 behaviour; the test fails)",
    lineM ~= nil and string.find(lineM, "via item slot=fpp attempt=10", 1, true) ~= nil, tostring(lineM))
  -- on foot: the first activation of a request is still the plain event (unchanged), a series without success still cycles
  local g2 = mock("female", femaleSet())
  g2.cfg.retry = true
  g2.body:SetWanted(true, "camera")
  g2.run(0.05)
  check("on foot: first activation still 'via event attempt=1' (the only on-foot change is the reset after a success)",
    g2.count("via event slot=fpp attempt=1") == 1 and g2.rep == "tpp" and g2.body.attempt == 0, tostring(g2.body.attempt))
end
do -- MOTO-1 C (2): no toggle / item in a vehicle window
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  local cfg = g.cfg
  local out = {}
  for _, win in ipairs({ "exit", "mount", "unmount", false }) do
    for _, n in ipairs({ 3, 4 }) do
      g.body.vehWindow = win or nil
      g.body.attempt = n
      out[#out + 1] = tostring(win) .. n .. "=" .. g.body:Strategy(cfg)
    end
  end
  local s = table.concat(out, ",")
  check("no toggle/item in vehicle window: attempt 3 / 4 -> event in exit, mount and unmount; outside -> toggle / item",
    s == "exit3=event,exit4=event,mount3=event,mount4=event,unmount3=event,unmount4=event,false3=toggle,false4=item", s)
  -- module wiring: M.StepVehicle publishes the window to the body
  local saved = { M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp }
  M.core, M.settings.enabled = nil, true
  local vw = { state = 4, mounted = true }
  M.Attach({ vehicle = function() return { state = vw.state, mounted = vw.mounted } end, log = function() end,
             headSlot = function() return "tpp" end })
  M.body.wanted = true
  local r1 = M.StepVehicle(1.0)
  local w1 = M.body.vehWindow
  vw.state = 1
  M.StepVehicle(1.1)
  check("module: StepVehicle sets body.vehWindow (mount), cleared when seated", r1 == "mount" and w1 == "mount"
    and M.body.vehWindow == nil, tostring(w1) .. "/" .. tostring(M.body.vehWindow))
  M.body.owner, M.body.pendingSince = "utpp", 2.5
  check("module: guard.peerActivated true only for the body's activation of the same frame",
    M.guard.peerActivated(2.5) == true and M.guard.peerActivated(2.6) == false)
  M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp = saved[1], saved[2], saved[3], saved[4], saved[5], saved[6], saved[7]
end
do -- 0.1.8 fix: the toggle / item log path called a nil global 'short' when the deactivate / the item failed
  local g = mock("female", femaleSet())
  g.cfg.retry = true
  g.body:SetWanted(true, "camera")
  g.body.api.deactivateRep = function() error("deactivate boom") end
  g.body.api.equipTppHead = function() error("item boom") end
  g.body.attempt, g.body.rep = 2, "fpp"
  local okT, errT = pcall(g.body.Activate, g.body, 1.0, false)
  local okI, errI = pcall(g.body.Activate, g.body, 1.1, false)
  check("toggle / item failure is logged, not raised", okT and okI and g.count("via toggle slot=fpp attempt=3 (deactivate FAILED: ") == 1
    and g.count("via item slot=fpp attempt=4 (head item FAILED: ") == 1, tostring(errT) .. " / " .. tostring(errI))
end
do -- MOTO-1 C (3): exit window - delay 0, poll every 0.016 s for the first 1.0 s
  local w = guardWorld({})
  local t = 0
  for _ = 1, 28 do t = t + 1 / 60; w.step(t, "exit") end -- 0.47 s into the exit, head present
  local pollsBefore = w.g.stats.polls
  w.slot = "empty"
  t = t + 1 / 60; w.step(t, "exit")
  check("exit: polls every frame in the first second (28 frames -> 28 polls)", pollsBefore == 28, pollsBefore)
  check("exit: head lost 0.47 s into the exit -> event repair on the first poll (delay 0)", w.events == 1
    and countLog(w.logs, "head missing (slot empty, exit) -> TPP representation requested") == 1, w.events)
  for _ = 1, 33 do t = t + 1 / 60; w.step(t, "exit") end -- to ~1.03 s
  local p1 = w.g.stats.polls
  for _ = 1, 30 do t = t + 1 / 60; w.step(t, "exit") end -- 0.5 s after the fast part
  local slow = w.g.stats.polls - p1
  check("exit: after 1.0 s the 0.05 s poll again (0.5 s -> 7-11 polls, not 30)", slow >= 6 and slow <= 11, slow)
  -- control: the unmount window keeps the 0.05 s delay and poll
  local u = guardWorld({})
  t = 0
  for _ = 1, 28 do t = t + 1 / 60; u.step(t, "unmount") end
  check("unmount control: 0.05 s polls (28 frames -> <= 10 polls)", u.g.stats.polls <= 10, u.g.stats.polls)
  -- exit windows replay (log: the game drops the head 0.47 s after the exit start): head back within event lag + 1 frame
  local we, _, lostAt, backAt = exitWindows(true)
  check("exit replay: head back within 0.12 s (event lag 0.10 + one frame; 0.1.7: up to 0.05 delay + 0.05 poll more)",
    backAt ~= nil and backAt - lostAt <= 0.12 and we.events >= 1, tostring(backAt and (backAt - lostAt)))
  -- the body's activation of the same frame counts as the guard's event repair
  local d = guardWorld({})
  d.g.peerActivated = function() return true end
  t = 0
  for _ = 1, 5 do t = t + 1 / 60; d.step(t, "exit") end
  d.slot = "fpp"
  t = t + 1 / 60; d.step(t, "exit")
  check("exit: body activated in this frame -> no second event, logged", d.events == 0 and d.g.stats.shared == 1
    and countLog(d.logs, "already requested by the body this frame") == 1)
end
-- CAR-7 (2): a re-spawned TPP head stays hidden until 2 consecutive 'tpp' reads plus one frame
local function hideWorld()
  local w = guardWorld({ ignoreEvent = true, itemFails = true })
  w.hidden, w.hideCalls, w.showCalls = false, 0, 0
  w.g.api.setHeadHidden = function(on, who)
    if on then
      if w.slot ~= "tpp" and w.slot ~= "spawning" then return false, "no TPP head" end
      w.hideCalls, w.hidden = w.hideCalls + 1, true
      return true, "2 mesh component(s) hidden"
    end
    w.showCalls, w.hidden = w.showCalls + 1, false
    return true, "2 mesh component(s) shown"
  end
  return w
end
do
  local w = hideWorld()
  local t = 0
  local seq = { "tpp", "tpp", "tpp", "empty", "spawning", "spawning", "spawning", "tpp", "tpp", "tpp", "tpp" }
  local trace = {}
  for _, s in ipairs(seq) do
    w.slot = s
    t = t + 1 / 60; w.step(t, "exit")
    trace[#trace + 1] = s .. "=" .. (w.hidden and "H" or "v")
  end
  local tr = table.concat(trace, ",")
  check("spawning head hidden until 2 consecutive 'tpp' reads + 1 frame",
    tr == "tpp=v,tpp=v,tpp=v,empty=v,spawning=H,spawning=H,spawning=H,tpp=H,tpp=H,tpp=v,tpp=v", tr)
  check("spawning head: one hide, one show, both logged", w.hideCalls == 1 and w.showCalls == 1
    and countLog(w.logs, "re-spawned TPP head hidden until settled") == 1
    and countLog(w.logs, "TPP head shown again (settled, hidden 0.083 s, 2 tpp read(s))") == 1, table.concat(w.logs, " | "))
  -- a 'spawning' read between the 'tpp' reads restarts the count
  w = hideWorld()
  trace = {}
  for _, s in ipairs({ "tpp", "empty", "spawning", "tpp", "spawning", "tpp", "tpp", "tpp" }) do
    w.slot = s
    t = t + 1 / 60; w.step(t, "exit")
    trace[#trace + 1] = s .. "=" .. (w.hidden and "H" or "v")
  end
  tr = table.concat(trace, ",")
  check("spawning head: the count restarts on a non-'tpp' read", tr == "tpp=v,empty=v,spawning=H,tpp=H,spawning=H,tpp=H,tpp=H,tpp=v", tr)
  -- bounded: unreadable slot after the hide -> shown after hideMax (0.3 s)
  w = hideWorld()
  t = 0
  w.slot = "tpp"; t = t + 1 / 60; w.step(t, "exit")
  w.slot = "spawning"; t = t + 1 / 60; w.step(t, "exit")
  local hiddenAt = t
  w.slot = "unknown"
  local shownAt = nil
  for _ = 1, 40 do t = t + 1 / 60; w.step(t, "exit"); if shownAt == nil and not w.hidden then shownAt = t end end
  check("spawning head: hide bounded by hideMax (0.3 s)", shownAt ~= nil and shownAt - hiddenAt >= 0.29 and shownAt - hiddenAt <= 0.32
    and countLog(w.logs, "TPP head shown again (hideMax") == 1, tostring(shownAt and (shownAt - hiddenAt)))
  -- window closes while hidden -> shown
  w = hideWorld()
  t = 0
  w.slot = "tpp"; t = t + 1 / 60; w.step(t, "exit")
  w.slot = "spawning"; t = t + 1 / 60; w.step(t, "exit")
  t = t + 1 / 60; w.step(t, nil)
  check("spawning head: window closed -> shown at once", w.hidden == false and w.showCalls == 1
    and countLog(w.logs, "TPP head shown again (window closed") == 1)
  -- a head that never left the slot is never hidden
  w = hideWorld()
  t = 0
  for _ = 1, 120 do w.slot = "tpp"; t = t + 1 / 60; w.step(t, "unmount") end
  check("head present the whole window -> never hidden", w.hideCalls == 0 and w.showCalls == 0)
end
-- CAM-11 body side: optional utpp_vehiclecam hook onVehiclePerspective(prev, new)
local function vpWorld()
  local v = { slot = "fpp", equips = 0, hidden = false, hideCalls = 0, showCalls = 0, reads = 0, logs = {}, on = true,
              state = 6, mounted = true }
  local api = {
    headSlot = function() v.reads = v.reads + 1; return v.slot end,
    equipTppHead = function() v.equips = v.equips + 1; return true, "Items.PlayerWaTppHead" end,
    setHeadHidden = function(on, who)
      v.who = who
      if on then v.hideCalls, v.hidden = v.hideCalls + 1, true else v.showCalls, v.hidden = v.showCalls + 1, false end
      return true, on and "hidden" or "shown"
    end,
    vehicle = function() return { state = v.state, mounted = v.mounted } end,
    log = function(m) v.logs[#v.logs + 1] = m end,
  }
  v.p = M.NewVehiclePerspective(api, function() return v.on end)
  v.t = 0
  v.run = function(seconds) for _ = 1, math.floor(seconds * 60 + 0.5) do v.t = v.t + 1 / 60; v.p:Step(v.t) end end
  return v
end
do
  local v = vpWorld()
  v.run(1.0)
  check("hook: nothing pending -> no game reads", v.reads == 0, v.reads)
  check("hook: FPP -> TPP (0 -> 4) with the FPP head -> TPP head pre-equipped once, logged", v.p:OnChange(v.t, 0, 4) == true
    and v.equips == 1 and countLog(v.logs, "vehicle camera FPP -> TPP (0 -> 4): TPP head pre-equipped (Items.PlayerWaTppHead)") == 1)
  check("hook: the same event through a second route is ignored", v.p:OnChange(v.t, 0, 4) == false and v.equips == 1)
  v.slot = "tpp"
  check("hook: TPP -> TPP (4 -> 5) -> nothing", v.p:OnChange(v.t, 4, 5) == false and v.equips == 1)
  v.run(0.2)
  v.p:OnChange(v.t, 5, 0)
  v.run(0.40)
  check("hook: TPP -> FPP -> nothing hidden before the end of the 0.45 s blend", v.hideCalls == 0)
  v.run(0.07)
  check("hook: TPP -> FPP, blend over, TPP head still in the slot -> hidden (owner vehcam)", v.hideCalls == 1 and v.hidden
    and v.who == "vehcam")
  v.run(0.1)
  check("hook: still hidden while the slot reads tpp", v.hidden == true)
  v.slot = "fpp"
  v.run(1 / 60)
  check("hook: the game swapped the head -> shown again", v.hidden == false and v.showCalls == 1
    and countLog(v.logs, "TPP head shown again (head swapped, slot fpp") == 1)
  -- the head already swapped at the end of the blend -> nothing hidden
  v.p:OnChange(v.t, 0, 4)
  v.slot = "tpp"
  v.p:OnChange(v.t + 0.5, 4, 0)
  v.slot = "fpp"
  v.run(1.0)
  check("hook: head already FPP at the end of the blend -> nothing hidden", v.hideCalls == 1
    and countLog(v.logs, "FPP blend over: head slot fpp - nothing hidden") == 1)
  -- spawning / TPP head at FPP -> TPP: never stacked
  v.slot = "spawning"
  local eq = v.equips
  v.p:OnChange(v.t, 0, 3)
  check("hook: head spawning at FPP -> TPP -> nothing pre-equipped", v.equips == eq
    and countLog(v.logs, "head slot spawning - nothing pre-equipped") == 1)
  -- holdMax, FPP -> TPP while hidden, unmount while hidden
  local function hiddenWorld()
    local h = vpWorld()
    h.slot = "tpp"
    h.p:OnChange(h.t, 4, 0)
    h.run(0.47)
    return h
  end
  local h = hiddenWorld()
  h.run(0.45)
  check("hook: hide bounded by holdMax 0.5 s", h.hidden == true)
  h.run(0.07)
  check("hook: shown after holdMax", h.hidden == false and countLog(h.logs, "(holdMax, hidden") == 1)
  h = hiddenWorld()
  h.p:OnChange(h.t, 0, 4)
  check("hook: FPP -> TPP while hidden -> shown first, no pre-equip with the TPP head in the slot", h.hidden == false
    and h.equips == 0 and countLog(h.logs, "(FPP -> TPP, hidden") == 1)
  h = hiddenWorld()
  h.state, h.mounted = 0, false
  h.run(1 / 60)
  check("hook: unmount while hidden -> shown", h.hidden == false and countLog(h.logs, "(unmount, hidden") == 1)
  -- switched off: logged once, no writes
  local o = vpWorld()
  o.on = false
  o.p:OnChange(o.t, 0, 4); o.p:OnChange(o.t, 4, 0); o.run(1.0)
  check("hook: setting off -> no pre-equip, no hide, logged once", o.equips == 0 and o.hideCalls == 0 and o.reads == 0
    and countLog(o.logs, "perspective change ignored") == 1)
  -- review fix: the exit take-over (UTPP third-person view) inside the blend -> the TPP head is never hidden
  local x = vpWorld()
  x.view = false
  x.p.viewTpp = function() return x.view end
  x.slot = "tpp"
  x.p:OnChange(x.t, 2, 0)
  x.run(0.2)
  x.view = true -- keep 'game' -> exit: third person at once, TPP body requested, no further vehicle camera events
  x.run(1.0)
  check("hook: UTPP third-person view before the end of the blend -> TPP head not hidden, logged", x.hideCalls == 0
    and not x.hidden and countLog(x.logs, "FPP blend over: third-person view (UTPP camera) - nothing hidden") == 1,
    string.format("hides %d", x.hideCalls))
  -- hidden just before the exit: shown at once when the third-person view starts (not after holdMax)
  local y = vpWorld()
  y.view = false
  y.p.viewTpp = function() return y.view end
  y.slot = "tpp"
  y.p:OnChange(y.t, 2, 0)
  y.run(0.47)
  local hidBefore = y.hidden
  y.view = true
  y.run(1 / 60)
  check("hook: hidden head + UTPP third-person view -> shown on the next frame", hidBefore == true and y.hidden == false
    and countLog(y.logs, "(third-person view, hidden") == 1)
  -- the view check failing / absent -> the first person behaviour above is unchanged
  local z = vpWorld()
  z.p.viewTpp = function() error("read failed") end
  z.slot = "tpp"
  z.p:OnChange(z.t, 2, 0)
  z.run(0.47)
  check("hook: view check failing -> treated as first person (hidden as before)", z.hidden == true and z.hideCalls == 1)
end
do -- review fix: M.Attach wires the view check to the requested TPP body
  local saved = { M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp }
  M.core, M.settings.enabled = nil, true
  M.Attach({ log = function() end, headSlot = function() return "tpp" end })
  local before = M.vpersp:ViewTpp()
  M.body.wanted = true
  local after = M.vpersp:ViewTpp()
  check("module: VPersp view check follows IsTppBody (false -> true)", before == false and after == true)
  M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp = saved[1], saved[2], saved[3], saved[4], saved[5], saved[6], saved[7]
end
do -- module: run-time detection of utpp_vehiclecam.onVehiclePerspective (absent hook = no change)
  local saved = { M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp, M.vcamHook, M.clock, M.LogLine }
  local lines = {}
  M.LogLine = function(l) lines[#lines + 1] = l end
  M.core, M.settings.enabled = nil, true
  local equips = 0
  M.Attach({ log = function() end, headSlot = function() return "fpp" end,
             equipTppHead = function() equips = equips + 1; return true, "x" end })
  M.vcamHook = { state = "pending", tries = 0, nextTry = 0 }
  local vc = {}
  check("module: hook found -> field set to M.OnVehiclePerspective", M.TryHookVehicleCam(1.0, function(n)
    return n == "utpp_vehiclecam" and vc or nil end) == "attached" and vc.onVehiclePerspective == M.OnVehiclePerspective
    and countLog(lines, "vehicle camera hook: utpp_vehiclecam.onVehiclePerspective set") == 1, table.concat(lines, " | "))
  M.clock = 5.0
  check("module: the vehicle camera calls the hook -> body handles it", vc.onVehiclePerspective(0, 4) == true and equips == 1)
  -- an existing function in the field is chained (called after ours)
  M.vcamHook = { state = "pending", tries = 0, nextTry = 0 }
  local other = 0
  local vc2 = { onVehiclePerspective = function() other = other + 1 end }
  M.TryHookVehicleCam(2.0, function() return vc2 end)
  M.clock = 6.0
  vc2.onVehiclePerspective(0, 5)
  check("module: existing hook function chained", other == 1 and equips == 2 and M.vcamHook.state == "attached")
  -- not loaded: tried once per second for 30 s, then absent (logged once), never again
  M.vcamHook = { state = "pending", tries = 0, nextTry = 0 }
  local asks = 0
  for i = 1, 40 * 60 do M.TryHookVehicleCam(i / 60, function() asks = asks + 1; return nil end) end
  check("module: utpp_vehiclecam absent -> 30 tries, then 'absent' (logged once)", M.vcamHook.state == "absent" and asks == 30
    and countLog(lines, "utpp_vehiclecam not found") == 1, asks)
  check("module: Status carries the hook state", string.find(M.Status(), "hook=absent", 1, true) ~= nil, M.Status())
  M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp = saved[1], saved[2], saved[3], saved[4], saved[5], saved[6], saved[7]
  M.vcamHook, M.clock, M.LogLine = saved[8], saved[9], saved[10]
end

-- ---------------------------------------------------------------- 0.1.9 exit keep of the TPP representation
-- Recording 2026-10-28 19:55 (diag-20261028-c9 E2): the game drops the representation 0.47 s into every bike exit. RepKeep
-- applies the anim feature every frame of the 'exit' window and holds the redscript latch (refreshed, bounded).
local function rkWorld(opts)
  opts = opts or {}
  local w = { t = 10.0, logs = {}, features = 0, latchOn = 0, latchOff = 0, lastSeconds = nil, diag = opts.diag,
              latchAnswer = opts.latchAnswer, diagReads = 0 }
  w.api = {
    log = function(m) w.logs[#w.logs + 1] = m end,
    feature = function(on) assert(on == true, "feature(false) from the keep"); w.features = w.features + 1 end,
    setRepKeep = function(on, s)
      if on then w.latchOn = w.latchOn + 1; w.lastSeconds = s else w.latchOff = w.latchOff + 1 end
      if w.latchAnswer ~= nil then return w.latchAnswer end
      return true
    end,
    repKeepDiag = function() w.diagReads = w.diagReads + 1; return w.diag end,
  }
  w.k = M.NewRepKeep(w.api, opts.k)
  -- frames at 60 fps with want = fn(t) or a constant
  w.run = function(secs, want)
    local n = math.floor(secs * 60 + 0.5)
    for _ = 1, n do
      w.t = w.t + 1 / 60
      local wv = want
      if type(want) == "function" then wv = want(w.t) end
      w.k:Step(w.t, wv)
    end
  end
  return w
end
do
  -- idle: nothing written, nothing read
  local w = rkWorld({ diag = "game=0 kept=0 own=0 finalDeact=0 finalAct=0 last=-1.000 keep=0.00" })
  w.run(1.0, false)
  check("repKeep: no exit window -> no feature, no latch, no diag reads", w.features == 0 and w.latchOn == 0 and w.latchOff == 0
    and w.diagReads == 0 and #w.logs == 0)
  -- bike exit as recorded: window 1.97 s, then the unmount
  w.run(1.97, true)
  local frames = math.floor(1.97 * 60 + 0.5)
  check("repKeep: exit window -> anim feature on every frame from the first", w.features == frames, w.features .. "/" .. frames)
  check("repKeep: latch set at the open and refreshed every 0.25 s with 0.5 s", w.latchOn >= 8 and w.latchOn <= 9
    and w.lastSeconds == 0.5, w.latchOn)
  check("repKeep: open logged once", countLog(w.logs, "representation keep on (exit window)") == 1, table.concat(w.logs, " | "))
  local f0 = w.features
  w.run(1 / 60, false)
  check("repKeep: window end -> latch cleared, closed once, no more features", w.latchOff == 1 and w.features == f0
    and countLog(w.logs, "representation keep off (window ended)") == 1 and not w.k.active)
  w.run(2.0, false)
  check("repKeep: after the window -> no writes", w.latchOff == 1 and w.features == f0)
end
do
  -- the window outlives maxWindow -> off, blocked until the window ends, a new window opens again
  local w = rkWorld()
  w.run(4.0, true)
  local f1, on1 = w.features, w.latchOn
  check("repKeep: maxWindow 3 s -> closed once, features only for 3 s", w.k.stats.timeouts == 1 and not w.k.active
    and w.latchOff == 1 and f1 >= 179 and f1 <= 181 and countLog(w.logs, "representation keep off (maxWindow 3.0 s)") == 1, f1)
  w.run(1.0, true)
  check("repKeep: still the same window after the timeout -> stays off", w.features == f1 and w.latchOn == on1)
  w.run(1 / 60, false)
  w.run(0.5, true)
  check("repKeep: next window -> on again", w.k.active and w.features > f1 and w.k.stats.windows == 2)
end
do
  -- latch not reachable (no .reds / no player): logged once, the anim feature still every frame
  local w = rkWorld({ latchAnswer = false })
  w.run(1.0, true)
  check("repKeep: latch refused -> logged once, feature every frame", countLog(w.logs, "redscript latch not set") == 1
    and w.features == 60 and w.k.stats.latchFails >= 4)
  -- api without feature / latch (bridge-less test api): no error
  local k = M.NewRepKeep({ log = function() end })
  local ok = pcall(function() for i = 1, 30 do k:Step(i / 60, true) end; k:Step(1.0, false) end)
  check("repKeep: api without feature / latch / diag -> no error", ok)
end
do
  -- native vehicle exit guard: opened once with the window, closed with it (window end and maxWindow), never refreshed
  local w = rkWorld()
  local calls = {}
  w.api.nativeGuard = function(on) calls[#calls + 1] = on; return true end
  w.run(1.0, true)
  check("repKeep: native guard opened once at the window start", #calls == 1 and calls[1] == true and w.k.nativeLive == true)
  w.run(1 / 60, false)
  check("repKeep: native guard closed at the window end", #calls == 2 and calls[2] == false)
  w.run(4.0, true)
  check("repKeep: maxWindow -> native guard closed with the keep, not reopened in the same window",
    #calls == 4 and calls[3] == true and calls[4] == false, #calls)
  check("repKeep: status carries the native counts", string.find(w.k:StatusLine(), "native=2/2(absent 0)", 1, true) ~= nil,
    w.k:StatusLine())
  -- absent / not installed native: logged once, the script keep still runs
  local w2 = rkWorld()
  w2.api.nativeGuard = function() return false end
  w2.run(0.5, true); w2.run(1 / 60, false); w2.run(0.5, true)
  check("repKeep: native guard not live -> logged once, feature still every frame",
    countLog(w2.logs, "native vehicle exit guard not live") == 1 and w2.k.stats.nativeAbsent == 2 and w2.features == 60)
  local w3 = rkWorld()
  w3.api.nativeGuard = function() error("no such function") end
  local ok = pcall(function() w3.run(0.5, true); w3.run(1 / 60, false) end)
  check("repKeep: native guard raising -> no error, logged once", ok and countLog(w3.logs, "native vehicle exit guard not live") == 1)
end
do
  -- diagnostics: a counter change is logged once (the clocks 'last' / 'keep' alone are not); polled for tail s after close
  local w = rkWorld({ diag = "game=0 kept=0 own=0 finalDeact=0 finalAct=0 last=-1.000 keep=0.50" })
  w.run(0.40, true)
  w.diag = "game=0 kept=0 own=0 finalDeact=0 finalAct=0 last=-1.000 keep=0.33"
  w.run(0.07, true)
  w.diag = "game=1 kept=1 own=0 finalDeact=0 finalAct=0 last=0.010 keep=0.40"
  w.run(0.20, true)
  w.diag = "game=1 kept=1 own=0 finalDeact=0 finalAct=0 last=0.200 keep=0.31"
  w.run(1.30, true)
  w.run(1 / 60, false)
  w.diag = "game=1 kept=1 own=0 finalDeact=1 finalAct=1 last=1.600 keep=0.00"
  w.run(0.3, false)
  check("repKeep: diag - two counter changes logged (in and after the window), clocks ignored",
    countLog(w.logs, "representation keep diag: game=1 kept=1") == 2
    and countLog(w.logs, "(window open)") == 1 and countLog(w.logs, "(after the window)") == 1, table.concat(w.logs, " | "))
  w.run(0.35, false) -- 0.65 s after the close: past tail (0.6 s)
  local reads = w.diagReads
  w.run(2.0, false)
  check("repKeep: diag polling stops tail s after the window", reads > 0 and w.diagReads == reads, reads .. " -> " .. w.diagReads)
  check("repKeep: off line carries the last diag", countLog(w.logs, "diag game=1 kept=1 own=0 finalDeact=0") == 1)
end
do
  -- module: the keep follows the 'exit' window, the wanted TPP body, the switches and the game reason
  local saved = { M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp, M.repkeep, M.settings.repKeep,
                  M.headGuardOverride }
  M.core, M.settings.enabled, M.settings.repKeep, M.headGuardOverride = nil, true, true, nil
  local veh = { state = 1, mounted = true }
  local feats, latch = 0, {}
  M.Attach({ log = function() end, headSlot = function() return "tpp" end, repState = function() return "tpp" end,
             vehicle = function() return { state = veh.state, mounted = veh.mounted } end,
             feature = function(on) if on then feats = feats + 1 end end,
             setRepKeep = function(on) latch[#latch + 1] = on; return true end })
  M.body.wanted = true
  M.body.gameReason = nil
  check("module: RepKeepWanted - exit + wanted -> true; mount / ride / nil -> false", M.RepKeepWanted("exit") == true
    and M.RepKeepWanted("mount") == false and M.RepKeepWanted("unmount") == false and M.RepKeepWanted(nil) == false)
  M.body.gameReason = "vehicleTPP"
  local vt = M.RepKeepWanted("exit")
  M.body.gameReason = "scene"
  local sc = M.RepKeepWanted("exit")
  M.body.gameReason = "takeover"
  local tk = M.RepKeepWanted("exit")
  M.body.gameReason = nil
  check("module: RepKeepWanted - vehicle TPP camera reason kept, scene / takeover not", vt == true and sc == false and tk == false)
  M.body.wanted = false
  local fpp = M.RepKeepWanted("exit")
  M.body.wanted = true
  M.SetRepKeep(false)
  local off = M.RepKeepWanted("exit")
  M.SetRepKeep(true)
  M.SetHeadGuard(false)
  local hg = M.RepKeepWanted("exit")
  M.SetHeadGuard(nil)
  check("module: RepKeepWanted - FPP / SetRepKeep(false) / head guard off -> false", fpp == false and off == false and hg == false)
  -- ride -> exit -> unmount through M.StepVehicle (60 fps)
  local t = 50.0
  local function frames(n) for _ = 1, n do t = t + 1 / 60; M.StepVehicle(t) end end
  frames(30) -- ride
  local rideFeats = feats
  veh.state = 4
  frames(118) -- exit (1.97 s)
  local exitFeats = feats - rideFeats
  veh.state, veh.mounted = 0, false
  frames(30) -- unmount window
  check("module: StepVehicle - feature only in the exit window, latch on then cleared at the unmount", rideFeats == 0
    and exitFeats == 118 and feats - rideFeats == 118 and latch[1] == true and latch[#latch] == false and not M.repkeep.active,
    string.format("ride %d exit %d total %d", rideFeats, exitFeats, feats))
  -- the user switches to FPP mid-exit -> off at once
  veh.state, veh.mounted = 1, true
  frames(5)
  veh.state = 4
  frames(20)
  local wasOn = M.repkeep.active
  M.body.wanted = false
  frames(1)
  check("module: FPP mid-exit -> keep off on the next frame", wasOn == true and M.repkeep.active == false
    and latch[#latch] == false)
  check("module: Status carries the keep", string.find(M.Status(), "repKeep=true active=false", 1, true) ~= nil, M.Status())
  M.body, M.guard, M.veh, M.api, M.core, M.settings.enabled, M.vpersp, M.repkeep = saved[1], saved[2], saved[3], saved[4],
    saved[5], saved[6], saved[7], saved[8]
  M.settings.repKeep, M.headGuardOverride = saved[9], saved[10]
end

-- ---------------------------------------------------------------- 0.1.11 wrapper variable utpp_foot_tpp
-- 1 while the UTPP third-person body is wanted, 0 otherwise and for a vanilla take-over; always in place before the
-- representation is activated (the graph edge ANY -> tpp_securitycam_hacking is a rising edge)
do
  local function last(g) return g.foot[#g.foot] and g.foot[#g.foot].v end
  local g = mock("female", femaleSet())
  g.run(0.5)
  check("foot: idle sends 0 once, then only the periodic re-assert", g.foot[1] and g.foot[1].v == 0 and #g.foot == 1
    and g.body.stats.footSends == 1, #g.foot)
  g.run(4.0)
  check("foot: idle re-assert every footReassert s (2 s) carries 0", #g.foot == 3 and last(g) == 0, #g.foot)
  g.body:SetWanted(true, "test")
  g.run(0.1)
  local firstOne
  for i, f in ipairs(g.foot) do if f.v == 1 then firstOne = f; break end end
  check("foot: 1 is sent before the first activation and before the first feature write",
    firstOne ~= nil and firstOne.act == 0 and firstOne.feat == 0 and g.act == 1, firstOne and (firstOne.act .. "/" .. firstOne.feat))
  local n = #g.foot
  g.run(1.0)
  check("foot: no resend inside the re-assert interval while the value holds", #g.foot == n, #g.foot - n)
  g.run(2.2)
  check("foot: re-asserted after footReassert s, still 1 (TPP body stays wanted)", #g.foot == n + 1 and last(g) == 1, #g.foot - n)
  -- a vanilla take-over of control (security camera): back to 0 at once, 1 again when it ends
  g.reason = "takeover"
  g.run(0.1)
  check("foot: take-over of control -> 0", last(g) == 0)
  g.reason = nil
  g.run(0.1)
  check("foot: take-over over -> 1", last(g) == 1)
  -- other game-owned reasons keep the value (same locomotion set, only its rank changes)
  g.reason = "vehicleTPP"
  g.run(0.1)
  check("foot: vehicle TPP camera keeps 1", last(g) == 1)
  g.reason = nil
  -- the setting switched off behaves like FPP
  g.cfg.enabled = false
  g.run(0.1)
  check("foot: setting off -> 0", last(g) == 0)
  g.cfg.enabled = true
  g.run(0.1)
  check("foot: setting on again -> 1", last(g) == 1)
  -- player change (load / respawn): the new entity starts at 0, the value is sent again before any activation
  g.key = "p2"
  g.rep = "fpp"
  local m = #g.foot
  g.run(0.1)
  check("foot: player change -> value sent again at once", #g.foot > m and g.foot[m + 1].v == 1 and g.foot[m + 1].key == "p2",
    #g.foot - m)
  -- release
  g.body:SetWanted(false, "test")
  g.run(0.1)
  check("foot: FPP -> 0", last(g) == 0)
  -- unfreeze after noplayer: unknown again -> re-sent
  g.frozenR = "noplayer"
  g.run(0.1)
  local k = #g.foot
  g.frozenR = nil
  g.run(0.1)
  check("foot: unfreeze -> sent again", #g.foot > k)
  -- send failure (no player yet): retried quickly, logged once, no counted success
  local g2 = mock("female", femaleSet())
  g2.noFoot = true
  g2.body:SetWanted(true, "test")
  g2.run(1.0)
  local fails = g2.body.stats.footFails
  check("foot: failed send retried about every 0.25 s and logged once", fails >= 3 and fails <= 6 and g2.count("utpp_foot_tpp not sent yet") == 1
    and g2.body.stats.footSends == 0, fails)
  g2.noFoot = false
  g2.run(0.3)
  check("foot: sent as soon as the player exists", last(g2) == 1 and g2.body.stats.footSends >= 1)
  -- an api without footTpp (old scripts) changes nothing
  local g3 = mock("female", femaleSet())
  g3.body.api.footTpp = nil
  g3.body:SetWanted(true, "test")
  g3.run(0.3)
  check("foot: api without footTpp -> no sends, body still activates", #g3.foot == 0 and g3.act == 1)
end

-- wrapper variable utpp_melee_tpp (gate of the UTPP melee clips): 1 together with utpp_foot_tpp while the mod's animations are the
-- pack in force, 0 with the Game Animations pack (cfg.packGame); the foot variable itself is not touched by the pack
do
  local function setup(Mod)
    local g = mock("female", femaleSet())
    g.melee = {}
    g.body.api.meleeTpp = function(on) g.melee[#g.melee + 1] = on and 1 or 0; return true end
    if Mod then g.body = Mod.NewBody(g.body.api, function() return g.cfg end) end
    return g
  end
  local function lastOf(t) return t[#t] end
  local g = setup()
  g.body:SetWanted(true, "test")
  g.run(0.3)
  check("melee variable: 1 with the foot variable while the mod's animations are in force", lastOf(g.melee) == 1 and g.foot[#g.foot].v == 1)
  local n = #g.melee
  g.run(1.0)
  check("melee variable: no resend inside the re-assert interval", #g.melee == n)
  g.cfg.packGame = true
  g.run(0.1)
  check("melee variable: 0 as soon as the Game Animations pack is in force, the foot variable stays 1",
    lastOf(g.melee) == 0 and g.foot[#g.foot].v == 1, tostring(lastOf(g.melee)))
  g.cfg.packGame = false
  g.run(0.1)
  check("melee variable: back to 1 with the mod's animations", lastOf(g.melee) == 1)
  g.body:SetWanted(false, "test")
  g.run(0.3)
  check("melee variable: 0 when the UTPP body is not wanted (vanilla / first person)", lastOf(g.melee) == 0 and g.foot[#g.foot].v == 0)
  local g2 = mock("female", femaleSet())
  g2.body:SetWanted(true, "test")
  local okRun = pcall(function() g2.run(0.3) end)
  check("melee variable: an api without meleeTpp (older scripts) changes nothing", okRun and g2.act == 1 and #g2.foot > 0)
  local g3 = setup()
  g3.body.api.meleeTpp = function() return false end
  g3.body:SetWanted(true, "test")
  g3.run(1.0)
  check("melee variable: a refused send (no player) is retried and logged once", g3.body.stats.footSends >= 1 and g3.count("utpp_melee_tpp not sent yet") == 1)
  -- the gate must follow the pack: a mutant that ignores cfg.packGame is caught
  local f = assert(io.open(UTPP_BODY_PATH, "rb"))
  local src = f:read("*a"); f:close()
  local from = "local v = (want and not self:Cfg().packGame) and 1 or 0"
  local a, b = src:find(from, 1, true)
  assert(a and not src:find(from, b + 1, true), "mutation anchor must exist exactly once")
  registerForEvent = nil
  local Mut = assert(loadstring(src:sub(1, a - 1) .. "local v = want and 1 or 0" .. src:sub(b + 1), "=utpp_body_mutant"))()
  registerForEvent = savedRegister
  local gm = setup(Mut)
  gm.body:SetWanted(true, "test")
  gm.run(0.3)
  gm.cfg.packGame = true
  gm.run(0.1)
  check("melee variable: mutation [pack ignored] is caught", lastOf(gm.melee) ~= 0)
end

print(string.format("utpp_body offline test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_body offline test failed")

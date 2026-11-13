-- UTPP utpp_camera: stance filter + pivot timing (fix batch 2, 2026-10-21 12:15 recording; P1 A/C and P7 F1).
--  * CM.pivotStep: a clean in-place crouch / stand-up takes the body's clip timing (crouchDelay + crouchBlend, standDelay + standBlend), every
--    other change keeps stanceBlend 0.25 s. The camera used to be at the crouch height 0.44 s before the hips (50 % of the pivot move at +0.125 s,
--    hips 50 % at +0.565 s).
--  * CM.stanceFilterStep: port of adapter::FilteredStance (clean edge bypass, CrouchSprint 30 = stand, flicker / Dodge / slide rules).
--  * replay of the PSM LocomotionDetailed + speed + "body upright" of the recorded session (fixtures/rec1215_psm_detail.tsv, 4000 rows) through
--    the filter and the pivot, the pre-fix camera next to it. The pre-fix camera held the crouch pivot (1.05 m) through whole CrouchSprint runs
--    with the body upright (8.74 s of 10.97 s wrong by > 0.2 m).
-- Globals from the runner: UTPP_CAMERA_MATH_PATH, UTPP_LUA_FIXTURE_DIR.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, (not ok and detail) and (" -- " .. tostring(detail)) or ""))
end
local CM = dofile(UTPP_CAMERA_MATH_PATH or "Source/plugin/UTPP/cet/utpp_camera/camera_math.lua")
local FPS = 60
local DT = 1 / FPS
-- the replay runs at the frame rate of the recording (the video is 58.6 fps VFR): a 0.135 s PSM run (the landing at feed t 338.04: Jump 18 ->
-- Crouch 3 -> CrouchSprint 30) is 8 frames = 0.138 s there, below the 0.15 s filter, like its 8 feeds in the DLL replay (at 1/60 s the
-- grid gives it 9 frames = 0.15 s)
local RFPS = 58
local RDT = 1 / RFPS

-- ================================================================ CM.pivotStep timing
-- hh of the pivot per frame after the stance flips at t = 0 (1 s of the old stance before it); aFast = the bypassed in-place edge
local function pivotRun(aCfg, aFrom, aTo, aFast, aSecs)
  local pv = CM.newPivot()
  local pos = { x = 0, y = 0, z = 0 }
  for _ = 1, FPS do CM.pivotStep(pv, aCfg, DT, pos, nil, aFrom, false, 0, false) end
  local hh = {}
  for i = 0, math.floor((aSecs or 1.2) * FPS) do
    CM.pivotStep(pv, aCfg, DT, pos, nil, aTo, false, 0, aFast)
    hh[i] = pv.hh
  end
  return hh
end
local function timeTo(aHH, aFrom, aTo, aFrac)
  local target = aFrom + (aTo - aFrom) * aFrac
  for i = 0, #aHH do
    if (aTo < aFrom and aHH[i] <= target) or (aTo > aFrom and aHH[i] >= target) then return i * DT end
  end
  return math.huge
end
do
  local cfg = CM.config()
  local S, C = cfg.standHeight, cfg.crouchHeight
  local c = pivotRun(cfg, false, true, true)
  local t50, t90 = timeTo(c, S, C, 0.5), timeTo(c, S, C, 0.9)
  check("pivot: a clean in-place crouch: 50 % of the move at 0.36 .. 0.48 s (the hips of Stand_to_Crouch: 0.43 s)", t50 >= 0.36 and t50 <= 0.48, t50)
  check("pivot: a clean in-place crouch: 90 % of the move by 0.62 s (the hips: 0.57 s)", t90 <= 0.62, t90)
  check("pivot: a clean in-place crouch: the first 0.15 s do not move the pivot (the hips are 1.6 cm down at 0.1 s)", c[math.floor(0.15 * FPS)] == S)
  local u = pivotRun(cfg, true, false, true)
  local u50 = timeTo(u, C, S, 0.5)
  check("pivot: a clean in-place stand-up: 50 % of the move at 0.18 .. 0.30 s (Crouch_to_Stand: 0.24 s)", u50 >= 0.18 and u50 <= 0.30, u50)
  -- every other change keeps the 0.25 s blend (walking, filtered, slide exit, CrouchSprint 30): finished at 0.25 s, 50 % at 0.125 s
  local w = pivotRun(cfg, false, true, false)
  local w50 = timeTo(w, S, C, 0.5)
  check("pivot: a change that is not a clean in-place edge keeps stanceBlend 0.25 s (50 % at 0.125 s, done at 0.25 s)",
        math.abs(w50 - 0.125) <= 2 * DT and w[math.floor(0.26 * FPS)] == C and w[math.floor(0.22 * FPS)] > C, w50)
  local wu = pivotRun(cfg, true, false, false)
  check("pivot: the stand-up of a walking change keeps stanceBlend 0.25 s", wu[math.floor(0.26 * FPS)] == S and wu[math.floor(0.22 * FPS)] < S)
  -- mutation (the pre-fix pivot: stanceBlend only, no delay) violates the in-place window
  local old = CM.config({ crouchDelay = 0, crouchBlend = cfg.stanceBlend, standDelay = 0, standBlend = cfg.stanceBlend })
  local o = pivotRun(old, false, true, true)
  local o50 = timeTo(o, S, C, 0.5)
  check("pivot (mutation): stanceBlend only puts the camera at 50 % at 0.125 s, 0.3 s before the hips (violates the window)", o50 < 0.30, o50)
  -- a target change in mid-blend: the new move starts from the pivot of the moment (continuous), a reversal during the delay does not jump
  local pv = CM.newPivot()
  local pos = { x = 0, y = 0, z = 0 }
  for _ = 1, FPS do CM.pivotStep(pv, cfg, DT, pos, nil, false, false, 0, false) end
  local prev, maxStep = pv.hh, 0
  local seq = { { true, 0.1 }, { false, 0.3 }, { true, 0.35 }, { false, 0.8 } }
  for _, s in ipairs(seq) do
    for _ = 1, math.floor(s[2] * FPS) do
      CM.pivotStep(pv, cfg, DT, pos, nil, s[1], false, 0, true)
      maxStep = math.max(maxStep, math.abs(pv.hh - prev))
      prev = pv.hh
    end
  end
  check("pivot: reversals inside the delay / blend are continuous (max step per frame < 3.5 cm: the 0.4 s smoothstep peaks at 3.1 cm)", maxStep < 0.035, maxStep)
end

-- ================================================================ CM.stanceFilterStep (port of adapter::FilteredStance)
local function filterRun(aCfg, aSeq)
  local st = CM.newStanceFilter()
  local outs, byp = {}, {}
  for _, seg in ipairs(aSeq) do
    for _ = 1, seg.n do
      local raw = seg.raw
      if raw == nil then raw = seg.det == 3 end
      local o, b = CM.stanceFilterStep(st, aCfg, raw, seg.det, seg.speed or 0, DT)
      outs[#outs + 1], byp[#byp + 1] = o, b
    end
  end
  return outs, byp
end
local function seg(det, n, speed, raw) return { det = det, n = n, speed = speed, raw = raw } end
local function firstIs(aOuts, aValue, aFrom) for i = aFrom or 1, #aOuts do if aOuts[i] == aValue then return i end end end
local function flips(aOuts) local n = 0 for i = 2, #aOuts do if aOuts[i] ~= aOuts[i - 1] then n = n + 1 end end return n end
do
  local cfg = CM.config()
  local o, b = filterRun(cfg, { seg(1, 30, 0), seg(3, 30, 0) })
  check("filter: a clean Stand -> Crouch edge in place passes on its own frame (bypassed)", firstIs(o, true) == 31 and b[31] == true, firstIs(o, true))
  o, b = filterRun(cfg, { seg(3, 30, 0), seg(1, 30, 0) })
  check("filter: a clean Crouch -> Stand edge passes on its own frame (bypassed)", firstIs(o, false, 2) == 31 and b[31] == true, firstIs(o, false, 2))
  o = filterRun(cfg, { seg(3, 30, 0), seg(30, 30, 0, false) })
  local d = (firstIs(o, false, 2) - 30) * DT
  check("filter: Crouch 3 -> CrouchSprint 30 (stand) keeps the filter time (0.15 s)", d >= 0.15 - 1e-6 and d <= 0.15 + DT + 1e-6, d)
  o = filterRun(cfg, { seg(3, 30, 2.8), seg(30, 1, 2.8, false), seg(3, 6, 2.8), seg(30, 1, 2.8, false), seg(3, 6, 2.8), seg(30, 1, 2.8, false), seg(3, 20, 2.8) })
  check("filter: single-frame CrouchSprint 30 spikes inside the crouch do not reach the pivot (flips)", flips(o) == 0, flips(o))
  local train = { seg(3, 30, 2.8) }
  for _ = 1, 12 do train[#train + 1] = seg(30, 1, 2.8, false); train[#train + 1] = seg(3, 6, 2.8) end
  o = filterRun(cfg, train)
  check("filter: the recorded 0.1 s Crouch / 0.017 s CrouchSprint train does not reach the pivot (flips)", flips(o) == 0, flips(o))
  local off = CM.config({ stanceFilter = 0 })
  o = filterRun(off, train)
  check("filter (mutation): without the filter every spike flips the stance twice (24 flips)", flips(o) == 24, flips(o))
  -- stand -> Crouch 3 for 0.1 s -> CrouchSprint 30 at the run speed (feed t 278.786 / 280.435): never a crouch
  o = filterRun(cfg, { seg(1, 30, 3.85), seg(3, 6, 3.85), seg(30, 60, 2.8, false) })
  check("filter: Stand -> Crouch 0.1 s -> CrouchSprint at 3.85 m/s never shows a crouch (flips)", flips(o) == 0 and o[#o] == false, flips(o))
  local nolimit = CM.config({ stanceCleanMaxSpeed = 1e9 })
  o = filterRun(nolimit, { seg(1, 30, 3.85), seg(3, 6, 3.85), seg(30, 60, 2.8, false) })
  check("filter (mutation): without the speed limit the blip shows as a crouch", flips(o) >= 2, flips(o))
  o = filterRun(cfg, { seg(1, 30, 3.85), seg(3, 30, 3.85) })
  d = (firstIs(o, true) - 31 + 1) * DT
  check("filter: a crouch entered at the run speed keeps the filter time", d >= 0.15 - 1e-6 and d <= 0.15 + DT + 1e-6, d)
  o = filterRun(cfg, { seg(1, 30, 2.7), seg(3, 30, 2.7) })
  check("filter: a crouch entered at the crouch walk speed (2.7 m/s) passes on its own frame", firstIs(o, true) == 31, firstIs(o, true))
  -- min-hold of a bypassed edge
  o = filterRun(cfg, { seg(1, 30, 0), seg(3, 3, 0), seg(1, 2, 0), seg(3, 30, 0) })
  check("filter: Stand -> Crouch (bypass) -> 2 stand frames -> Crouch: the return does not reach the pivot (flips)", flips(o) == 1, flips(o))
  local nohold = CM.config({ stanceCleanHold = 0 })
  o = filterRun(nohold, { seg(1, 30, 0), seg(3, 3, 0), seg(1, 2, 0), seg(3, 30, 0) })
  check("filter (mutation): without the min-hold the 2-frame return flips twice more", flips(o) == 3, flips(o))
  -- Dodge 7 keeps the stance; slide: pass-through and the exit window
  o = filterRun(cfg, { seg(3, 30, 2.7), seg(7, 12, 2.7), seg(3, 30, 2.7) })
  check("filter: Dodge 7 from the crouch keeps the crouch (flips)", flips(o) == 0 and o[1] == true, flips(o))
  o = filterRun(cfg, { seg(5, 20, 6, true), seg(3, 18, 2.7), seg(1, 30, 2.7) })
  d = (firstIs(o, false, 39) - 38) * DT
  check("filter: a stand-up 0.3 s after a slide keeps the filter time", d >= 0.15 - 1e-6 and d <= 0.15 + DT + 1e-6, d)
  o = filterRun(cfg, { seg(5, 20, 6, true), seg(3, 50, 2.7), seg(1, 30, 2.7) })
  check("filter: a stand-up 0.83 s after a slide is clean", firstIs(o, false, 71) == 71, firstIs(o, false, 71))
  o = filterRun(cfg, { seg(4, 30, 5, false), seg(5, 20, 6, true), seg(3, 1, 6), seg(4, 30, 6, false) })
  check("filter: slide -> one Crouch frame -> Sprint: the Sprint frame replaces it at once", o[51] == true and o[52] == false, tostring(o[52]))
end

-- ================================================================ replay of the recorded session
local fixPath = (UTPP_LUA_FIXTURE_DIR or "Source/tools/lua-tests/fixtures") .. "/rec1215_psm_detail.tsv"
local rows = {}
do
  local f = io.open(fixPath, "r")
  check("replay: the fixture rec1215_psm_detail.tsv opens", f ~= nil, fixPath)
  if f then
    for line in f:lines() do
      if line:sub(1, 1) ~= "#" and #line > 0 then
        local t, det, sp, up = line:match("^(%S+)%s+(%S+)%s+(%S+)%s+(%S+)")
        rows[#rows + 1] = { t = tonumber(t), det = tonumber(det), speed = tonumber(sp), upright = tonumber(up) == 1 }
      end
    end
    f:close()
  end
end
local function locoOf(det) if det == 3 or det == 30 then return 1 elseif det == 5 then return 10 end return 0 end
-- init.lua crouchState: the first result is the stance; aDet30Stand = the fix (CrouchSprint is a stand)
local function rawCrouch(loco, det, aDet30Stand)
  local slide = (loco == 9 or loco == 10) and det ~= 1
  return ((loco == 1 and (not aDet30Stand or det ~= 30)) or loco == 11 or loco == 12 or slide), slide and det == 5
end
-- one camera pass over the session; aOpt = { det30Stand, cfg }; returns the per-frame arrays
local function replay(aOpt)
  local cfg = aOpt.cfg
  local st, pv = CM.newStanceFilter(), CM.newPivot()
  local pos = { x = 0, y = 0, z = 0 }
  local t0, t1 = rows[1].t, rows[#rows].t
  local r, n = 1, math.floor((t1 - t0) * RFPS)
  local out = { t = {}, det = {}, speed = {}, upright = {}, hh = {}, crouch = {}, raw = {}, bypassed = {} }
  local sfOut, fast = nil, false
  for i = 0, n - 1 do
    local t = t0 + i * RDT
    while rows[r + 1] and rows[r + 1].t <= t do r = r + 1 end
    local row = rows[r]
    local loco = locoOf(row.det)
    local raw, slide = rawCrouch(loco, row.det, aOpt.det30Stand)
    local crouch, bypassed = raw, false
    if aOpt.filter ~= false then crouch, bypassed = CM.stanceFilterStep(st, cfg, raw, row.det, row.speed, RDT) end
    if crouch ~= sfOut then
      sfOut = crouch
      fast = bypassed == true and row.speed < (cfg.stanceFastSpeed or 0.5)
    end
    CM.pivotStep(pv, cfg, RDT, pos, nil, crouch, slide and (cfg.slideHeight or cfg.crouchHeight) < cfg.crouchHeight, 0, fast)
    out.t[#out.t + 1], out.det[#out.det + 1], out.speed[#out.speed + 1], out.upright[#out.upright + 1] = t, row.det, row.speed, row.upright
    out.hh[#out.hh + 1], out.crouch[#out.crouch + 1], out.raw[#out.raw + 1], out.bypassed[#out.bypassed + 1] = pv.hh, crouch, raw, bypassed
  end
  return out
end
local function metrics(o, cfg)
  local m = { err30 = 0, minLate = math.huge, nLate = 0, standLate = 0, episodes = {}, switchesWin = 0, inPlace = 0, inPlaceSame = 0, spikeFlips = 0 }
  local entry, lastIn30, lastSlide = nil, -1e9, -1e9
  for i = 1, #o.t do
    local t, det = o.t[i], o.det[i]
    if det == 5 then lastSlide = t end
    if det == 30 then
      if t - lastIn30 > 0.25 then entry = t end
      lastIn30 = t
      local truth = o.upright[i] and cfg.standHeight or cfg.crouchHeight
      -- the 0.5 s pivot blend out of a slide (slideBlend, protected) is not part of the CrouchSprint stance
      if t - lastSlide >= 0.6 and math.abs(o.hh[i] - truth) > 0.2 then m.err30 = m.err30 + RDT end
      if t - entry >= 0.40 then
        m.nLate = m.nLate + 1
        m.minLate = math.min(m.minLate, o.hh[i])
        if o.hh[i] >= 1.45 then m.standLate = m.standLate + 1 end
      end
    end
    if i > 1 and o.crouch[i] ~= o.crouch[i - 1] and t >= 321.2 and t <= 325.0 then m.switchesWin = m.switchesWin + 1 end
    -- in-place stand -> crouch edges (the raw stance flips at standstill, then holds 0.3 s): does the filtered stance flip on the same frame?
    if i > 1 and o.raw[i] and not o.raw[i - 1] and o.speed[i] < 0.5 and o.det[i] == 3 and o.det[i - 1] == 1 then
      local hold = 0
      while i + hold <= #o.t and o.raw[i + hold] do hold = hold + 1 end
      local clean = hold * RDT >= 0.3
      for j = i - 1, math.max(1, i - math.floor(0.8 * RFPS)), -1 do -- stand, no slide / sprint / air / CrouchSprint in the 0.8 s before
        if o.det[j] ~= 1 and o.det[j] ~= 2 and o.det[j] ~= 3 then clean = false end
      end
      if clean then
        m.inPlace = m.inPlace + 1
        if o.crouch[i] then m.inPlaceSame = m.inPlaceSame + 1 end
      end
    end
  end
  local eps = { { 278.86, "v187.3" }, { 280.51, "v188.95" }, { 338.16, "v246.6" } }
  for _, e in ipairs(eps) do
    local mn = math.huge
    for i = 1, #o.t do if o.t[i] >= e[1] and o.t[i] <= e[1] + 1.0 then mn = math.min(mn, o.hh[i]) end end
    m.episodes[#m.episodes + 1] = mn
  end
  return m
end
if #rows > 1000 then
  local cfg = CM.config()
  local newO = replay({ cfg = cfg, det30Stand = true })
  local nm = metrics(newO, cfg)
  local oldCfg = CM.config({ stanceFilter = 0 })
  local oldO = replay({ cfg = oldCfg, det30Stand = false, filter = false })
  local om = metrics(oldO, oldCfg)
  print(string.format("replay: %d frames; wrong CrouchSprint frames old %.2f s -> new %.2f s; standing pivot 0.4 s into a CrouchSprint: old %d of %d -> new %d of %d frames; episodes new %.2f %.2f %.2f; window flips old %d -> new %d",
    #newO.t, om.err30, nm.err30, om.standLate, om.nLate, nm.standLate, nm.nLate, nm.episodes[1], nm.episodes[2], nm.episodes[3], om.switchesWin, nm.switchesWin))
  check("replay (pre-fix camera): the crouch pivot is held through CrouchSprint runs with the body upright (wrong > 8 s of 10.97 s)", om.err30 > 8.0, om.err30)
  check("replay: CrouchSprint 30 frames with a pivot more than 0.2 m off the body (upright -> 1.55 m): <= 1.0 s of 10.97 s", nm.err30 <= 1.0, nm.err30)
  check("replay (pre-fix camera): 0.40 s into a CrouchSprint the pivot is never at the standing height", om.nLate > 300 and om.standLate == 0, om.standLate)
  check("replay: 0.40 s into a CrouchSprint 30 the pivot is at the standing height (>= 1.45 m) in >= 85 % of the frames (the rest: the real crouch phases of that run)",
        nm.standLate >= 0.85 * nm.nLate, string.format("%d of %d", nm.standLate, nm.nLate))
  check("replay: the three stand -> Crouch 0.1 s -> CrouchSprint entries at the run speed (v187.3 / 188.95 / 246.6) keep the pivot >= 1.50 m",
        nm.episodes[1] >= 1.50 and nm.episodes[2] >= 1.50 and nm.episodes[3] >= 1.50, string.format("%.2f %.2f %.2f", nm.episodes[1], nm.episodes[2], nm.episodes[3]))
  check("replay: the Crouch 3 <-> CrouchSprint 30 flicker window (v229.6 - 233.4) changes the camera stance <= 4 times", nm.switchesWin <= 4, nm.switchesWin)
  check("replay: >= 15 clean in-place crouches in the session, every one flips the camera stance on its own frame", nm.inPlace >= 15 and nm.inPlaceSame == nm.inPlace,
        string.format("%d of %d", nm.inPlaceSame, nm.inPlace))
  -- mutations of the new code must each violate one bound
  local m1 = metrics(replay({ cfg = cfg, det30Stand = false }), cfg)
  check("replay (mutation): CrouchSprint 30 read as a crouch again -> the pivot is wrong for > 1 s", m1.err30 > 1.0, m1.err30)
  local cfg2 = CM.config({ stanceFilter = 0 })
  local m2 = metrics(replay({ cfg = cfg2, det30Stand = true }), cfg2)
  check("replay (mutation): no filter -> the flicker window pumps the pivot (> 20 stance changes; the diagnosis counted 37)", m2.switchesWin > 20, m2.switchesWin)
  local cfg3 = CM.config({ stanceCleanEdge = false })
  local m3 = metrics(replay({ cfg = cfg3, det30Stand = true }), cfg3)
  check("replay (mutation): no clean-edge bypass -> the in-place crouches lag by the filter time", m3.inPlaceSame < m3.inPlace, string.format("%d of %d", m3.inPlaceSame, m3.inPlace))
  local cfg4 = CM.config({ stanceCleanMaxSpeed = 1e9 })
  local m4 = metrics(replay({ cfg = cfg4, det30Stand = true }), cfg4)
  check("replay (mutation): no speed limit on the crouch entry -> the run-speed entries dip the pivot (< 1.50 m)",
        math.min(m4.episodes[1], m4.episodes[2], m4.episodes[3]) < 1.50, string.format("%.2f %.2f %.2f", m4.episodes[1], m4.episodes[2], m4.episodes[3]))
end

-- ================================================================ P7 F2: a jump out of a slide (11 in the recording)
-- The pivot of the recording dipped to 0.73 .. 0.85 m over the capsule (the crouch pivot is 1.05 m) for 0.4 s after the take-off: the 0.12 s
-- spring of the base lags the rise (~0.30 m) and the 0.5 s exit blend of the slide pivot starts from the slide height. The exit blend into the
-- air is short and the base follows the take-off meanwhile (cfg.slideAirExitBlend / slideAirExitSmooth, CM.pivotStep air).
do
  local f = io.open((UTPP_LUA_FIXTURE_DIR or "Source/tools/lua-tests/fixtures") .. "/rec1215_slide_jump.tsv", "r")
  check("F2: the fixture rec1215_slide_jump.tsv opens", f ~= nil)
  local evs, cur = {}, nil
  if f then
    for line in f:lines() do
      if line:sub(1, 6) == "@event" then
        cur = { t0 = tonumber(line:match("t=(%S+)")), rows = {} }
        evs[#evs + 1] = cur
      elseif line:sub(1, 1) ~= "#" and #line > 0 then
        local t, det, sp, cz, rel = line:match("^(%S+) (%S+) (%S+) (%S+) (%S+)")
        cur.rows[#cur.rows + 1] = { t = tonumber(t), det = tonumber(det), sp = tonumber(sp), z = tonumber(cz), rel = tonumber(rel) }
      end
    end
    f:close()
  end
  local function isAir(det) return det == 6 or (det >= 14 and det <= 22) end
  -- the camera chain of one event: filter -> pivot, per recorded feed; aAir = pass the air flag (the fix), else the pre-fix call
  local function run(ev, cfg, aAir)
    local st, pv = CM.newStanceFilter(), CM.newPivot()
    local sfOut, fast, prevT = nil, false, nil
    local res = {}
    for _, r in ipairs(ev.rows) do
      local dt = prevT and (r.t - prevT) or 1 / 60
      prevT = r.t
      if dt > 0 then
        local loco = (r.det == 3 or r.det == 30) and 1 or (r.det == 5 and 10 or 0)
        local slide = (loco == 9 or loco == 10) and r.det ~= 1
        local raw = (loco == 1 and r.det ~= 30) or slide
        local crouch, byp = CM.stanceFilterStep(st, cfg, raw, r.det, r.sp, dt)
        if crouch ~= sfOut then sfOut = crouch; fast = byp and r.sp < 0.5 end
        local piv = CM.pivotStep(pv, cfg, dt, { x = 0, y = 0, z = r.z }, nil, crouch, slide and r.det == 5, 0, fast, aAir and isAir(r.det) or nil)
        res[#res + 1] = { t = r.t, rel = piv.z - r.z, meas = r.rel }
      end
    end
    return res
  end
  local cfg = CM.config()
  local nOld, nNew, nOK, errSum, errN = 0, 0, 0, 0, 0
  local worstNew, worstOld = math.huge, -math.huge
  -- the squat after the take-off: the largest fall of the pivot over the capsule in the 0.35 s after its first peak (within 0.3 s of the take-off)
  local function squat(res, t0)
    local peak, peakT = -math.huge, 0
    for _, x in ipairs(res) do
      if x.t >= t0 and x.t <= t0 + 0.3 and x.rel > peak then peak, peakT = x.rel, x.t end
    end
    local dip = 0
    for _, x in ipairs(res) do
      if x.t >= peakT and x.t <= peakT + 0.35 then dip = math.max(dip, peak - x.rel) end
    end
    return dip
  end
  local worstSquat, worstSquatNoRamp = 0, 0
  -- 2026-11-03: the recording and the two negative controls below are the pre-fast-mode spring (cfg.pivotFast = false, CM.pivotStep fast fall):
  -- they model what the recorded build did; the fix checks (>= 0.98 m, no squat) run with the default config
  local cfgOld = CM.config({ pivotFast = false })
  local cfgNoRamp = CM.config({ slideAirExitRamp = 0, pivotFast = false })
  for _, ev in ipairs(evs) do
    worstSquat = math.max(worstSquat, squat(run(ev, cfg, true), ev.t0))
    worstSquatNoRamp = math.max(worstSquatNoRamp, squat(run(ev, cfgNoRamp, true), ev.t0))
    local old, new = run(ev, cfgOld, false), run(ev, cfg, true)
    local minOld, minNew, minMeas = math.huge, math.huge, math.huge
    for i, x in ipairs(old) do
      if x.t >= ev.t0 and x.t <= ev.t0 + 0.4 then
        minOld, minNew, minMeas = math.min(minOld, x.rel), math.min(minNew, new[i].rel), math.min(minMeas, x.meas)
      end
      if x.t >= ev.t0 - 0.5 and x.t <= ev.t0 + 1.0 then errSum, errN = errSum + math.abs(x.rel - x.meas), errN + 1 end
    end
    nOld, nNew = nOld + (minOld < 0.9 and 1 or 0), nNew + (minNew >= 0.98 and 1 or 0)
    nOK = nOK + (math.abs(minOld - minMeas) <= 0.06 and 1 or 0)
    worstNew, worstOld = math.min(worstNew, minNew), math.max(worstOld, minOld)
  end
  print(string.format("F2: %d slide -> air events; pivot over the capsule, min in 0.4 s after the take-off: pre-fix call worst-case max %.2f m (measured 0.72 .. 0.85), with the air exit min %.2f m; model error %.3f m",
    #evs, worstOld, worstNew, errN > 0 and errSum / errN or 0))
  check("F2: the recording has 11 slide -> air events", #evs == 11, #evs)
  check("F2: the pre-fix pivot call reproduces the recorded dip (min within 6 cm of the camTrace for every event, mean error <= 3 cm)", nOK == #evs and errN > 0 and errSum / errN <= 0.03,
        string.format("%d of %d, mean %.3f", nOK, #evs, errN > 0 and errSum / errN or 0))
  check("F2 (pre-fix): 0.4 s after the take-off the pivot is below 0.90 m over the capsule in every event (below the crouch pivot)", nOld == #evs, nOld)
  check("F2: with the air exit the pivot over the capsule stays >= 0.98 m in 0.4 s after the take-off in every event", nNew == #evs and worstNew >= 0.98, string.format("%d of %d, min %.2f", nNew, #evs, worstNew))
  check(string.format("F2: no squat after the take-off: the pivot falls at most 2 cm after its first peak (%.3f m)", worstSquat), worstSquat <= 0.02, worstSquat)
  check(string.format("F2 (mutation): the base spring stepping back to pivotSmooth at the end of the exit blend squats 0.08 m or more (%.3f m)", worstSquatNoRamp), worstSquatNoRamp >= 0.08, worstSquatNoRamp)
  -- the air exit is only for the slide pivot exit: a jump from the standing / crouching ground is bit for bit the same with and without the flag
  local function jump(aAir, aCrouch)
    local pv = CM.newPivot()
    local zs = {}
    local z, vz = 0, 0
    for i = 1, 120 do -- 1 s standing, then a 5.5 m/s jump under 16 m/s^2 for 1.2 s
      CM.pivotStep(pv, cfg, DT, { x = 0, y = 0, z = z }, nil, aCrouch, false, 0, false, nil)
    end
    for i = 1, 72 do
      vz = (i == 1) and 5.5 or (vz - 16 * DT)
      z = math.max(0, z + vz * DT)
      local piv = CM.pivotStep(pv, cfg, DT, { x = 0, y = 0, z = z }, nil, aCrouch, false, 0, false, aAir)
      zs[#zs + 1] = piv.z
    end
    return zs
  end
  for _, cr in ipairs({ false, true }) do
    local a, b = jump(true, cr), jump(false, cr)
    local same = true
    for i = 1, #a do if a[i] ~= b[i] then same = false end end
    check(string.format("F2: a jump from the %s ground is bit for bit the same with and without the air flag", cr and "crouching" or "standing"), same)
  end
  -- the exit blend on the ground keeps slideBlend (the flag is not set there)
  local pv1, pv2 = CM.newPivot(), CM.newPivot()
  local pos = { x = 0, y = 0, z = 0 }
  for _ = 1, 60 do CM.pivotStep(pv1, cfg, DT, pos, nil, true, true, 0, false, false); CM.pivotStep(pv2, cfg, DT, pos, nil, true, true, 0, false, nil) end
  local same = true
  for _ = 1, 60 do
    local a = CM.pivotStep(pv1, cfg, DT, pos, nil, false, false, 0, false, false)
    local b = CM.pivotStep(pv2, cfg, DT, pos, nil, false, false, 0, false, nil)
    same = same and a.z == b.z
  end
  check("F2: the slide exit on the ground (no air flag) is unchanged", same)
end

print(string.format("utpp_camera stance test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_camera stance test failed")

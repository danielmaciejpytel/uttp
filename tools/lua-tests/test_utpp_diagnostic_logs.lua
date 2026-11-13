-- utpp_core "Diagnostic logs" list (core.diagnosticLogs: Off / On for 5 minutes / On until switched off): the three states, the
-- five-minute timer that runs only while the game is played, the switch-off by itself (list back to Off, saved, pushed to Mod
-- Settings), the restart rules (the timed mode never survives, the permanent one does), the pose recording and report natives
-- (injected through core.diag.api; a missing native never stops the rest), the screen messages, the report text, and the paths
-- that must not reach a log. Global: UTPP_CORE_PATH. Mutations of the shipped source must be caught.
local passed, failed = 0, 0
local function check(name, ok, detail)
  if ok then passed = passed + 1; print("[PASS] utpp_diagnostic_logs: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_diagnostic_logs: " .. name .. (detail ~= nil and (" -- " .. tostring(detail)) or "")) end
end

local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod, open = io.open }
local settingsFile, baseFile = os.tmpname(), os.tmpname()
os.remove(settingsFile)
os.remove(baseFile)
local logs = {}

local function readFile(p)
  local f = SAVED.open(p, "r")
  if not f then return nil end
  local s = f:read("*a")
  f:close()
  return s
end
local function writeFile(p, text)
  local f = assert(SAVED.open(p, "wb"))
  f:write(text)
  f:close()
end

-- mutation = nil: the shipped module; otherwise { from, to }: the module text with one replacement. keepFile = the settings file
-- written by the test is read (restart tests); otherwise every core starts without one.
local function loadCore(mutation, keepFile)
  if not keepFile then os.remove(settingsFile); os.remove(baseFile) end
  registerForEvent, registerInput, Game = nil, nil, nil
  local realOpen = SAVED.open
  io.open = function(p, m) if p == "utpp_settings.txt" then p = settingsFile end return realOpen(p, m) end
  local core
  if mutation then
    local f = assert(realOpen(UTPP_CORE_PATH, "rb"))
    local src = f:read("*a"); f:close()
    local a, b = src:find(mutation[1], 1, true)
    assert(a and not src:find(mutation[1], b + 1, true), "mutation anchor must exist exactly once: " .. mutation[1])
    core = assert(loadstring(src:sub(1, a - 1) .. mutation[2] .. src:sub(b + 1), "=utpp_core_mutant"))()
  else
    core = dofile(UTPP_CORE_PATH)
  end
  io.open = realOpen
  logs = {}
  core.sink = function(line) logs[#logs + 1] = line end
  core.SETTINGS_FILE = settingsFile
  core.MS_BASE_FILE = baseFile
  -- the other diagnostic switches are registered by other modules after core has read its file (the game order)
  core.SetupMMSettings(function() end, function() end, {})
  core.Register({ key = "camera.trace", section = "test", type = "bool", default = false, label = "camera trace" })
  core.Register({ key = "perspective.log", section = "test", type = "bool", default = true, label = "perspective log" })
  core.Register({ key = "mm.debug", section = "test", type = "bool", default = false, label = "motion window" })
  return core
end

-- natives of the diagnostic logs as a recording mock
local function mockApi(opts)
  opts = opts or {}
  local m = { calls = {}, shown = {}, reports = {}, poseFiles = "diag/pose-a.bin|diag/pose-b.bin" }
  local function rec(s) m.calls[#m.calls + 1] = s end
  m.api = {
    setReloadDiag = function(on) rec("reload=" .. tostring(on)) end,
    setVaultLog = function(on) rec("vault=" .. tostring(on)) end,
    setFeetDiag = function(on) rec("feet=" .. tostring(on)) end,
    poseStart = function(keep) rec("poseStart=" .. tostring(keep)); return opts.poseRefuses ~= true end,
    poseStop = function() rec("poseStop"); return m.poseFiles end,
    poseStatus = opts.poseStatus and function() return opts.poseStatus end or nil,
    report = function(body)
      rec("report")
      m.reports[#m.reports + 1] = body
      if opts.reportFails then return "" end
      return "red4ext\\logs\\utpp-report-2026-11-11_12-00-00.txt"
    end,
    notify = function(text, seconds) rec("notify"); m.shown[#m.shown + 1] = text end,
  }
  function m.count(prefix)
    local n = 0
    for _, c in ipairs(m.calls) do if string.sub(c, 1, #prefix) == prefix then n = n + 1 end end
    return n
  end
  function m.line() return table.concat(m.calls, ",") end
  return m
end

-- plays n seconds in steps of 0.5 s
local function play(core, seconds, playing)
  for _ = 1, math.floor(seconds / 0.5 + 0.5) do core.DiagTick(0.5, playing ~= false) end
end

local function shipped()
  -- ---------------------------------------------------------------- the three states and the natives
  local core = loadCore()
  local m = mockApi()
  core.diag.api = m.api
  core.Set("core.diagnosticLogs", "timed")
  check("timed: the diagnostic switches are on and held", core.Get("core.log") == true and core.Get("mm.trace") == true
    and core.Get("mm.feedDump") == true and core.Get("camera.trace") == true and core.Get("perspective.log") == true)
  check("timed: reload diagnostic, vehicle vault log and feet diagnostic switched on at once, pose recording not yet",
    m.line() == "reload=true,vault=true,feet=true", m.line())
  check("timed: nothing is shown or recorded before a gameplay frame", #m.shown == 0 and m.count("poseStart") == 0)
  core.DiagTick(0.5, false)
  check("timed: a frame without gameplay (menu / pause / no player) starts no recording and shows no message",
    m.count("poseStart") == 0 and #m.shown == 0)
  core.DiagTick(0.5, true)
  check("timed: the first gameplay frame starts the pose recording for the whole session (keep 0)", m.count("poseStart=0") == 1
    and m.count("poseStart") == 1, m.line())
  check("timed: and shows the start message once", #m.shown == 1 and string.find(m.shown[1], "ON for 5 minutes", 1, true) ~= nil, m.shown[1])
  core.DiagTick(0.5, true)
  check("timed: no second start / message", m.count("poseStart") == 1 and #m.shown == 1)

  -- ---------------------------------------------------------------- the timer runs only in gameplay
  local remain = core.diag.remaining
  play(core, 600, false)
  check("timer: ten minutes of menu / pause / no player do not count", core.diag.mode == "timed" and core.diag.remaining == remain
    and core.Get("core.diagnosticLogs") == "timed", tostring(core.diag.remaining))
  play(core, 290, true)
  check("timer: 290 s of play: still on", core.diag.mode == "timed" and core.Get("core.diagnosticLogs") == "timed")
  play(core, 5, false)
  check("timer: a pause in between does not count either", core.diag.mode == "timed")
  play(core, 10, true)
  check("timer: after 5 minutes of play the list switches itself to Off", core.diag.mode == "off" and core.Get("core.diagnosticLogs") == "off",
    core.Get("core.diagnosticLogs"))
  check("timer: the held switches are back at their defaults (core.log, perspective.log on, the others off)", core.Get("core.log") == true
    and core.Get("perspective.log") == true and core.Get("mm.trace") == false and core.Get("mm.feedDump") == false
    and core.Get("camera.trace") == false)
  check("timer: natives off, pose recording stopped, one report written", m.count("reload=false") == 1 and m.count("vault=false") == 1
    and m.count("feet=false") == 1 and m.count("poseStop") == 1 and m.count("report") == 1, m.line())
  check("timer: Off is saved", string.find(readFile(settingsFile) or "", "core.diagnosticLogs=off", 1, true) ~= nil
    and string.find(readFile(settingsFile) or "", "core.diagnosticLogs=timed", 1, true) == nil)
  check("timer: Off is marked for Mod Settings (pushed to its screen)", core.msync.dirty["core.diagnosticLogs"] == true)
  check("timer: the end message is shown in the same gameplay frame and names the instruction file", #m.shown == 2
    and string.find(m.shown[2], "utpp-report-2026-11-11_12-00-00.txt", 1, true) ~= nil and string.find(m.shown[2], "OFF", 1, true) ~= nil, m.shown[2])
  local body = m.reports[1] or ""
  check("report body: mode, duration in gameplay seconds, reason, pose files, key settings", string.find(body, "Diagnostic logs: On for 5 minutes", 1, true) ~= nil
    and string.find(body, "300 s of gameplay", 1, true) ~= nil and string.find(body, "the five minutes were over", 1, true) ~= nil
    and string.find(body, "diag/pose-a.bin|diag/pose-b.bin", 1, true) ~= nil and string.find(body, "mm.enabled=", 1, true) ~= nil
    and string.find(body, "animation.pack=utpp", 1, true) ~= nil, body)
  play(core, 60, true)
  check("timer: nothing happens after the switch-off", m.count("report") == 1 and m.count("poseStop") == 1 and core.diag.mode == "off")

  -- ---------------------------------------------------------------- until switched off: by hand, window of 10 minutes
  core = loadCore()
  m = mockApi()
  core.diag.api = m.api
  core.Set("headLook.maxYaw", 80)
  core.Set("core.diagnosticLogs", "until")
  core.DiagTick(0.5, true)
  check("until: the pose recording keeps the last 10 minutes (keep 10) and the message says so", m.count("poseStart=10") == 1
    and string.find(m.shown[1] or "", "until you switch them off", 1, true) ~= nil, m.line())
  play(core, 3600, true)
  check("until: stays on however long it is played", core.diag.mode == "until" and core.Get("core.diagnosticLogs") == "until")
  core.Set("core.diagnosticLogs", "off")
  body = m.reports[1] or ""
  check("until: switching off by hand stops the recording and writes the report once", m.count("poseStop") == 1 and m.count("report") == 1)
  check("until: report: mode, switched off by hand, settings that differ from the defaults (and not the held logs)",
    string.find(body, "Diagnostic logs: On until switched off", 1, true) ~= nil and string.find(body, "switched off in the menu", 1, true) ~= nil
    and string.find(body, "headLook.maxYaw=80", 1, true) ~= nil and string.find(body, "mm.trace=", 1, true) == nil
    and string.find(body, "core.diagnosticLogs=", 1, true) == nil, body)
  core.DiagTick(0.5, false)
  check("until: the end message waits for a gameplay frame (not in a menu)", #m.shown == 1)
  core.DiagTick(0.5, true)
  check("until: the end message names the report", #m.shown == 2
    and string.find(m.shown[2] or "", "utpp-report-2026-11-11_12-00-00.txt", 1, true) ~= nil, m.shown[2])

  -- ---------------------------------------------------------------- switching between the two on modes
  core = loadCore()
  m = mockApi()
  core.diag.api = m.api
  core.Set("core.diagnosticLogs", "timed")
  core.DiagTick(0.5, true)
  play(core, 100, true)
  core.Set("core.diagnosticLogs", "until")
  core.DiagTick(0.5, true)
  check("timed -> until: the recording restarts with the 10 minute window, no report yet", m.count("poseStop") == 1
    and m.count("poseStart=10") == 1 and m.count("report") == 0, m.line())
  play(core, 600, true)
  check("timed -> until: the five-minute timer no longer applies", core.diag.mode == "until")
  core.Set("core.diagnosticLogs", "off")
  check("timed -> until -> off: one report that lists the files of both parts", m.count("report") == 1
    and select(2, string.gsub(m.reports[1], "diag/pose%-a%.bin", "")) == 2, m.reports[1])

  -- ---------------------------------------------------------------- a missing or failing native never stops the rest
  core = loadCore()
  Game = {}
  local okAll = pcall(function()
    core.Set("core.diagnosticLogs", "timed")
    core.DiagTick(0.5, true)
    play(core, 310, true)
  end)
  Game = nil
  check("missing natives: no error, the timer still ends the mode and the list is Off", okAll and core.Get("core.diagnosticLogs") == "off")
  local lines = {}
  for _, l in ipairs(logs) do if string.find(l, "FAILED", 1, true) then lines[#lines + 1] = l end end
  local perField = {}
  for _, l in ipairs(lines) do perField[l] = (perField[l] or 0) + 1 end
  local twice = false
  for _, n in pairs(perField) do if n > 1 then twice = true end end
  check("missing natives: every failure is logged once", #lines > 0 and not twice, #lines)
  core = loadCore()
  m = mockApi({ poseRefuses = true, reportFails = true })
  core.diag.api = m.api
  core.Set("core.diagnosticLogs", "until")
  core.DiagTick(0.5, true)
  core.Set("core.diagnosticLogs", "off")
  core.DiagTick(0.5, true)
  check("refused recording / unwritable report: the other logs still run and stop, the message says so", m.count("poseStop") == 0
    and core.Get("mm.trace") == false and string.find(m.shown[#m.shown] or "", "could not be written", 1, true) ~= nil, m.shown[#m.shown])
end

shipped()

-- pose recording problems show up in the report (no absolute path in the line)
do
  local function runReport(opts, files)
    local core = loadCore()
    local m = mockApi(opts)
    if files ~= nil then m.poseFiles = files end
    core.diag.api = m.api
    core.Set("core.diagnosticLogs", "until")
    core.DiagTick(0.5, true)
    core.Set("core.diagnosticLogs", "off")
    core.DiagTick(0.5, true)
    return m.reports[1] or ""
  end
  local okBody = runReport({ poseStatus = "poseCapture state=recording failed=0" })
  check("report: a healthy pose recording adds no problem line", not okBody:find("Pose recording:", 1, true), okBody)
  local noFile = runReport({ poseStatus = "poseCapture state=recording failed=0" }, "")
  check("report: no pose file written is stated", noFile:find("Pose recording: no file was written", 1, true) ~= nil, noFile)
  local failed = runReport({ poseStatus = "poseCapture state=recording failed=1 file=x" })
  check("report: a failed pose recording is stated", failed:find("Pose recording: failed (disk full or folder not writable)", 1, true) ~= nil, failed)
  check("report: the pose problem lines carry no absolute path", not failed:find("[A-Za-z]:[\/]") and not noFile:find("[A-Za-z]:[\/]"))
end

-- ---------------------------------------------------------------- restart rules (the game order: the file is read when core loads)
do
  writeFile(settingsFile, "core.diagnosticLogs=timed\nmm.trace=true\n")
  local core = loadCore(nil, true)
  local m = mockApi()
  core.diag.api = m.api
  check("restart: 'On for 5 minutes' starts as Off", core.Get("core.diagnosticLogs") == "off" and core.diag.mode == "off")
  check("restart: no native, no report", core.diag.calls == 0 and #m.calls == 0 and m.count("report") == 0)
  check("restart: the stored single switch is untouched", core.Get("mm.trace") == true)
  check("restart: Off is marked for Mod Settings", core.msync.dirty["core.diagnosticLogs"] == true)
  core.DiagTick(0.5, true)
  check("restart: no message", #m.shown == 0)

  -- Mod Settings still shows 'On for 5 minutes' (its own user.ini) and the baseline holds it: Off is pushed, not imported
  writeFile(baseFile, "core.diagnosticLogs=timed\n")
  local fields = { core_diagnosticLogs = "1", animation_pack = "0" }
  local pushed = {}
  core.msync.api = {
    available = function() return true end, menuOpen = function() return false end,
    snapshot = function()
      local out = {}
      for k, v in pairs(fields) do out[#out + 1] = k .. "=" .. v end
      table.sort(out)
      return table.concat(out, "\n") .. "\n"
    end,
    push = function(data)
      pushed[#pushed + 1] = data
      for line in string.gmatch(data, "[^\n]+") do
        local f, v = string.match(line, "^([%w_]+)=(.*)$")
        if f and fields[f] ~= nil then fields[f] = v end
      end
      return 1
    end,
  }
  for _ = 1, 6 do core.ModSettingsTick(0.3) end
  check("restart + Mod Settings: the timed value is not imported back", core.Get("core.diagnosticLogs") == "off" and core.diag.mode == "off")
  check("restart + Mod Settings: Off is pushed to its screen", fields.core_diagnosticLogs == "0" and #pushed >= 1, tostring(fields.core_diagnosticLogs))

  -- 'On until switched off' survives a restart
  writeFile(settingsFile, "core.diagnosticLogs=until\ncore.log=false\n")
  core = loadCore(nil, true)
  m = mockApi()
  core.diag.api = m.api
  check("restart: 'On until switched off' stays on, the switches are held (also for later registrations)",
    core.Get("core.diagnosticLogs") == "until" and core.diag.mode == "until" and core.Get("core.log") == true and core.Get("mm.trace") == true)
  check("restart: nothing is called before a game session exists", #m.calls == 0)
  core.DiagTick(0.5, false)
  check("restart: still nothing in a menu", #m.calls == 0 and #m.shown == 0)
  core.DiagTick(0.5, true)
  check("restart: the first gameplay frame starts the 10 minute recording and shows the message", m.count("poseStart=10") == 1
    and #m.shown == 1)
  core.Set("core.diagnosticLogs", "off")
  check("restart: switching off writes the report", m.count("report") == 1 and m.count("poseStop") == 1)
  os.remove(settingsFile)
  os.remove(baseFile)
end

-- ---------------------------------------------------------------- the file after a mode sequence: a restart leaves nothing held on
do
  local KEYS = { "core.log", "mm.trace", "mm.feedDump", "camera.trace", "perspective.log" }
  -- a new core that reads the file left by the previous one (the restart); a mode sequence runs on the first one
  local function start(fresh)
    local core = loadCore(nil, not fresh)
    core.diag.api = mockApi().api
    return core
  end
  local function state(core)
    local t = {}
    for _, k in ipairs(KEYS) do t[#t + 1] = k .. "=" .. tostring(core.Get(k)) end
    return core.Get("core.diagnosticLogs") .. " " .. table.concat(t, " ")
  end
  -- defaults: core.log and perspective.log on, the others off (nothing "held on" beyond them)
  local ALL_OFF = "off core.log=true mm.trace=false mm.feedDump=false camera.trace=false perspective.log=true"
  local function sequence(name, steps, expect)
    local core = start(true)
    steps(core)
    local restarted = start(false)
    local got = state(restarted)
    check("restart sequence: " .. name, got == expect and restarted.diag.mode == (restarted.Get("core.diagnosticLogs") == "until" and "until" or "off"),
      got)
    os.remove(settingsFile)
    os.remove(baseFile)
  end
  local function set(core, v) core.Set("core.diagnosticLogs", v); core.DiagTick(0.5, true) end

  sequence("until -> timed -> restart: everything off", function(core) set(core, "until"); set(core, "timed") end, ALL_OFF)
  sequence("until -> off -> restart: everything off", function(core) set(core, "until"); set(core, "off") end, ALL_OFF)
  sequence("until -> timed -> off -> restart: everything off", function(core) set(core, "until"); set(core, "timed"); set(core, "off") end, ALL_OFF)
  sequence("timed -> until -> timed -> restart: everything off", function(core) set(core, "timed"); set(core, "until"); set(core, "timed") end, ALL_OFF)
  sequence("timed -> restart: everything off", function(core) set(core, "timed") end, ALL_OFF)
  sequence("off -> timed -> an unrelated setting -> restart: everything off", function(core)
    set(core, "timed"); core.Set("mm.debug", true) end, ALL_OFF)
  sequence("until -> an unrelated setting -> timed -> restart: everything off", function(core)
    set(core, "until"); core.Set("mm.debug", true); set(core, "timed") end, ALL_OFF)
  sequence("the timer ends the timed mode -> restart: everything off", function(core)
    set(core, "until"); set(core, "timed"); play(core, 301) end, ALL_OFF)
  sequence("a switch set by hand -> timed -> restart: only that switch stays on", function(core)
    core.Set("mm.trace", true); set(core, "timed") end, "off core.log=true mm.trace=true mm.feedDump=false camera.trace=false perspective.log=true")
  sequence("a switch set by hand -> until -> timed -> restart: only that switch stays on", function(core)
    core.Set("mm.trace", true); set(core, "until"); set(core, "timed") end, "off core.log=true mm.trace=true mm.feedDump=false camera.trace=false perspective.log=true")
  sequence("a switch set by hand -> until -> restart -> timed -> restart: only that switch stays on", function(core)
    core.Set("mm.trace", true); set(core, "until")
    local second = start(false)
    set(second, "timed")
  end, "off core.log=true mm.trace=true mm.feedDump=false camera.trace=false perspective.log=true")
  sequence("a switch switched off by hand -> timed -> restart: it stays off", function(core)
    core.Set("core.log", false); set(core, "timed") end, "off core.log=false mm.trace=false mm.feedDump=false camera.trace=false perspective.log=true")
  sequence("until -> restart: still on, held", function(core) set(core, "until") end,
    "until core.log=true mm.trace=true mm.feedDump=true camera.trace=true perspective.log=true")
  sequence("until -> restart -> off -> restart: everything off", function(core)
    set(core, "until")
    local second = start(false)
    set(second, "off")
  end, ALL_OFF)
end

-- ---------------------------------------------------------------- no personal paths in a log line
do
  local core = loadCore()
  local S = core.ScrubPaths
  local cet = [[C:\Users\player\AppData\Local\Steam\steamapps\common\Cyberpunk 2077\bin\x64\plugins\cyber_engine_tweaks\mods\utpp_core\init.lua:123: boom]]
  check("scrub: a CET script path is cut to mods\\...", S(cet) == [[mods\utpp_core\init.lua:123: boom]], S(cet))
  local game = [[cannot open D:\Games\Cyberpunk 2077\red4ext\logs\utpp-x.txt now]]
  check("scrub: the game folder is dropped", S(game) == [[cannot open red4ext\logs\utpp-x.txt now]], S(game))
  check("scrub: a user profile path becomes ~", S([[see C:\Users\Anna\Documents\x.txt]]) == "see ~\\Documents\\x.txt", S([[see C:\Users\Anna\Documents\x.txt]]))
  check("scrub: any other drive path is replaced", string.find(S([[open D:/work/some dir/file.txt]]), "D:", 1, true) == nil
    and string.find(S([[open D:/work/some dir/file.txt]]), "work", 1, true) == nil, S([[open D:/work/some dir/file.txt]]))
  check("scrub: forward slashes and lower case mods", string.find(S("C:/Users/bob/x/mods/utpp_motion/init.lua:5: e"), "bob", 1, true) == nil
    and string.find(S("C:/Users/bob/x/mods/utpp_motion/init.lua:5: e"), "mods/utpp_motion/init.lua:5: e", 1, true) == 1)
  check("scrub: text without a path is unchanged", S("t=1.50 head look apply (camera) a:b") == "t=1.50 head look apply (camera) a:b")
  core.Log([[update error: C:\Users\bob\AppData\x\mods\utpp_core\init.lua:9: nil value]])
  check("scrub: core.Log scrubs", string.find(logs[#logs], "bob", 1, true) == nil and string.find(logs[#logs], "mods\\utpp_core\\init.lua:9", 1, true) ~= nil,
    logs[#logs])
  -- every module that writes to the UTPP log passes its lines through a scrubber
  local base = string.gsub(UTPP_CORE_PATH, "[\\/]utpp_core[\\/]init%.lua$", "")
  local modules = { "utpp_core", "utpp_motion", "utpp_body", "utpp_camera", "utpp_perspective", "utpp_vehiclecam", "utpp_bodyturn",
    "utpp_loothead", "utpp_reload_gate" }
  for _, name in ipairs(modules) do
    local f = SAVED.open(base .. "/" .. name .. "/init.lua", "rb")
    local src = f and f:read("*a") or ""
    if f then f:close() end
    local bad, n = {}, 0
    for call in string.gmatch(src, "UTPPLogLine%(([^\n]*)") do
      n = n + 1
      if not (string.find(call, "crubPaths(", 1, true)) then bad[#bad + 1] = call end
    end
    check("every UTPPLogLine call of " .. name .. " goes through the path scrubber (" .. n .. " calls)", f ~= nil and n > 0 and #bad == 0,
      table.concat(bad, " | "))
  end
  -- utpp_motion really scrubs (its log goes out through Game.UTPPLogLine)
  local out = {}
  Game = { UTPPLogLine = function(l) out[#out + 1] = l end }
  local v2 = dofile((string.gsub(UTPP_CORE_PATH, "utpp_core([\\/])init%.lua$", "utpp_motion%1init.lua")))
  Game = { UTPPLogLine = function(l) out[#out + 1] = l end }
  local saved = print
  print = function() end
  v2.sink = nil
  v2.log([[Sync failed: C:\Users\bob\AppData\Local\x\mods\utpp_motion\init.lua:77: oops]])
  print = saved
  Game = nil
  check("utpp_motion log: the user name never reaches the UTPP log", #out == 1 and string.find(out[1], "bob", 1, true) == nil
    and string.find(out[1], "mods\\utpp_motion\\init.lua:77", 1, true) ~= nil, out[1])
end

-- ---------------------------------------------------------------- mutations of the shipped source are caught
-- each scenario returns true when the behaviour is right; the shipped source passes, every mutant must fail
local function scenario(core, name)
  local m = mockApi()
  core.diag.api = m.api
  if name == "timer counts only in play" then
    core.Set("core.diagnosticLogs", "timed")
    core.DiagTick(0.5, true)
    play(core, 400, false)
    return core.diag.mode == "timed"
  elseif name == "timer ends the mode" then
    core.Set("core.diagnosticLogs", "timed")
    core.DiagTick(0.5, true)
    play(core, 310, true)
    return core.diag.mode == "off" and core.Get("core.diagnosticLogs") == "off" and m.count("report") == 1
  elseif name == "until never ends by itself" then
    core.Set("core.diagnosticLogs", "until")
    core.DiagTick(0.5, true)
    play(core, 1000, true)
    return core.diag.mode == "until"
  elseif name == "pose recording starts only in gameplay" then
    core.Set("core.diagnosticLogs", "until")
    core.DiagTick(0.5, false)
    return m.count("poseStart") == 0
  elseif name == "until keeps 10 minutes, timed all" then
    core.Set("core.diagnosticLogs", "until")
    core.DiagTick(0.5, true)
    return m.count("poseStart=10") == 1
  elseif name == "report only on a real switch-off" then
    core.Set("core.diagnosticLogs", "off")
    return m.count("report") == 0 and m.count("poseStop") == 0
  elseif name == "messages only in gameplay" then
    core.Set("core.diagnosticLogs", "until")
    core.DiagTick(0.5, false)
    return #m.shown == 0
  end
  error("unknown scenario " .. name)
end
local function restartScenario(core)
  return core.Get("core.diagnosticLogs") == "off" and core.diag.mode == "off" and core.msync.dirty["core.diagnosticLogs"] == true
end
local MUTANTS = {
  { "timer counts only in play", { "    if playing then\n      D.played = D.played + dt", "    if true then\n      D.played = D.played + dt" } },
  { "timer ends the mode", { "if D.remaining <= 0 then", "if D.remaining <= -1e9 then" } },
  { "until never ends by itself", { "if D.mode == \"timed\" then\n        D.remaining = D.remaining - dt", "if true then\n        D.remaining = D.remaining - dt" } },
  { "pose recording starts only in gameplay", { "      if D.poseDue then diagStartPose() end", "      end\n      if D.poseDue then diagStartPose() end\n      if playing then" } },
  { "until keeps 10 minutes, timed all", { "local keep = D.mode == \"until\" and M.DIAG_UNTIL_KEEP_MINUTES or 0", "local keep = 0" } },
  { "report only on a real switch-off", { "  local prev = D.mode\n  if prev == mode then", "  local prev = D.mode\n  if false then" } },
  { "messages only in gameplay", { "  if playing and D.notice then", "  if D.notice then" } },
}
-- shipped source: every scenario passes
for _, mu in ipairs(MUTANTS) do
  check("scenario [" .. mu[1] .. "] holds on the shipped source", scenario(loadCore(), mu[1]) == true)
end
for _, mu in ipairs(MUTANTS) do
  local core = loadCore(mu[2]) -- an anchor that no longer exists is a failure of the test itself
  local okS, res = pcall(scenario, core, mu[1])
  check("mutation [" .. mu[1] .. "] is caught", not (okS and res == true), tostring(res))
end
-- restart reset
do
  writeFile(settingsFile, "core.diagnosticLogs=timed\n")
  check("scenario [timed does not survive a restart] holds on the shipped source", restartScenario(loadCore(nil, true)) == true)
  writeFile(settingsFile, "core.diagnosticLogs=timed\n")
  local mutant = loadCore({ "  if v == \"timed\" then\n    S.values[M.DIAG_KEY] = \"off\"", "  if false then\n    S.values[M.DIAG_KEY] = \"off\"" }, true)
  check("mutation [timed survives a restart] is caught", restartScenario(mutant) ~= true)
  writeFile(settingsFile, "core.diagnosticLogs=timed\n")
  local mutant2 = loadCore({ "    M.msync.dirty[M.DIAG_KEY] = true\n    v = \"off\"", "    v = \"off\"" }, true)
  check("mutation [Off not pushed to Mod Settings after a restart] is caught", restartScenario(mutant2) ~= true)
  os.remove(settingsFile)
end
-- a later Save inside the timed window must not write the held switches (quit or crash inside the window)
local function timedSaveScenario(core)
  core.Set("core.diagnosticLogs", "timed")
  core.Set("mm.debug", true) -- another key: Save writes the whole registry
  local text = readFile(settingsFile) or ""
  return text:find("mm.trace=false", 1, true) ~= nil and text:find("mm.trace=true", 1, true) == nil
    and text:find("camera.trace=true", 1, true) == nil
    and core.Get("mm.trace") == true
end
do
  check("Save in the timed window does not persist the held switches", timedSaveScenario(loadCore()) == true, readFile(settingsFile))
  local mutant = loadCore({ "if M.diag and M.diag.mode ~= \"off\" and DIAG_SET[key] then", "if false then" })
  check("mutation [Save persists the timed switches] is caught", timedSaveScenario(mutant) ~= true)
  -- until mode: the list itself is persisted (the restart holds the switches again); the held values are not written as the
  -- player's own (they would stay on after a later switch to the timed mode or to Off and a restart)
  local core = loadCore()
  core.Set("core.diagnosticLogs", "until")
  core.Set("mm.debug", true)
  local untilText = readFile(settingsFile) or ""
  check("Save in 'until' persists the list and not the held switches", untilText:find("core.diagnosticLogs=until", 1, true) ~= nil
    and untilText:find("mm.trace=false", 1, true) ~= nil and untilText:find("mm.trace=true", 1, true) == nil, untilText)
  os.remove(settingsFile)
end

registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
io.open = SAVED.open
os.remove(settingsFile)
os.remove(baseFile)
print(string.format("utpp_diagnostic_logs test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_diagnostic_logs: " .. failed .. " check(s) failed")

-- utpp_core one-time settings reset (core.settingsReset): a settings file of an earlier reset level is not read - every option
-- starts on its default, the file is rewritten at once with the current level, the values last synced with Mod Settings are
-- forgotten and one message on the screen tells the player. A fresh install and a file of the current level are left alone, and
-- the next starts change nothing. The game order is used: the file is read while the core module loads, the options of the
-- other modules register afterwards. Global: UTPP_CORE_PATH. Mutations of the shipped source must be caught.
local passed, failed = 0, 0
local function check(name, ok, detail)
  if ok then passed = passed + 1; print("[PASS] utpp_settings_reset: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_settings_reset: " .. name .. (detail ~= nil and (" -- " .. tostring(detail)) or "")) end
end

local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod, open = io.open,
                print = print, Observe = Observe, ObserveAfter = ObserveAfter }
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
local function count(text, needle)
  local n, from = 0, 1
  while true do
    local a, b = string.find(text or "", needle, from, true)
    if not a then return n end
    n, from = n + 1, b + 1
  end
end
local function logCount(needle)
  local n = 0
  for _, l in ipairs(logs) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end

local SOURCE = (function()
  local f = assert(SAVED.open(UTPP_CORE_PATH, "rb"))
  local src = f:read("*a")
  f:close()
  return src
end)()

-- Loads the core the way the game does: the settings file (and the Mod Settings baseline) of the test are the files of the mod
-- folder, and the file is read while the module loads. mutation = nil: the shipped module; otherwise { from, to }: the module
-- text with one replacement. The lines logged while the module loads are kept too (there is no log sink yet at that point).
local function loadCore(mutation)
  registerForEvent, registerInput, Game = nil, nil, nil
  local realOpen = SAVED.open
  io.open = function(p, m)
    if p == "utpp_settings.txt" then p = settingsFile elseif p == "utpp_modsettings_sync.txt" then p = baseFile end
    return realOpen(p, m)
  end
  logs = {}
  print = function(line) logs[#logs + 1] = tostring(line) end
  local ok, core
  if mutation then
    local a, b = SOURCE:find(mutation[1], 1, true)
    ok, core = pcall(function() return assert(loadstring(SOURCE:sub(1, a - 1) .. mutation[2] .. SOURCE:sub(b + 1), "=utpp_core_mutant"))() end)
  else
    ok, core = pcall(dofile, UTPP_CORE_PATH)
  end
  print = SAVED.print
  io.open = realOpen
  assert(ok, core)
  core.sink = function(line) logs[#logs + 1] = line end
  core.SETTINGS_FILE = settingsFile
  core.MS_BASE_FILE = baseFile
  return core
end

-- options that other modules register after the core has read its file; "t.carried" has a value carried over from elsewhere
-- (def.initial, used when nothing is stored), "t.flag" records the value its callback saw last
local function registerLate(core)
  local rec = {}
  core.Register({ key = "t.flag", section = "test", type = "bool", default = true, label = "flag", apply = function(v) rec.flag = v end })
  core.Register({ key = "t.num", section = "test", type = "float", default = 1.5, min = 0, max = 3, label = "num" })
  core.Register({ key = "t.carried", section = "test", type = "bool", default = true, initial = false, label = "carried" })
  return rec
end

-- screen message mock: answers = list of what each call does ("ok" | "refuse" | "error"); past its end every call is "ok".
-- The game's message widget of the session exists (the module is told so) unless noWidget is set.
local function mockScreen(core, answers, noWidget)
  local m = { shown = {}, calls = 0 }
  if not noWidget then core.ResetNoticeScreen(true) end
  core.diag.api = { notify = function(text, seconds)
    m.calls = m.calls + 1
    local a = (answers or {})[m.calls] or "ok"
    if a == "error" then error("screen message native missing") end
    if a == "refuse" then return false end
    m.shown[#m.shown + 1] = { text = text, seconds = seconds }
    return true
  end }
  return m
end
-- plays (or waits in a menu) for `seconds` in steps of 0.5 s; returns how many calls answered "shown"
local function play(core, seconds, playing)
  local n = 0
  for _ = 1, math.floor(seconds / 0.5 + 0.5) do
    if core.ResetNoticeTick(0.5, playing ~= false) == true then n = n + 1 end
  end
  return n
end

local PROBE = (function()
  os.remove(settingsFile); os.remove(baseFile)
  return loadCore()
end)()
local LEVEL = PROBE.SETTINGS_RESET
local LEVEL_LINE = PROBE.SETTINGS_RESET_KEY .. "=" .. LEVEL .. "\n"
local NOTICE_LINE = PROBE.RESET_NOTICE_KEY .. "=1"
local REVISION_LINE = PROBE.DEFAULTS_REVISION_KEY .. "=" .. PROBE.DEFAULTS_REVISION
local DELAY = PROBE.RESET_NOTICE_DELAY
local FALLBACK = PROBE.RESET_NOTICE_FALLBACK
local D_YAW = PROBE.settings.defs["headLook.maxYaw"].default
local D_ZOOM = PROBE.settings.defs["lootView.zoom"].default
local D_COLOR = PROBE.settings.defs["crosshair.color"].default

-- a settings file as an earlier version left it: no reset level line, nine stored values (one of a module that is not loaded)
local OLD_FILE = "# UTPP settings (utpp_core 0.5.4): key=value, delete a line to restore its default\n"
  .. "core.defaultsRevision=1\nheadLook.maxYaw=60\nlootView.zoom=0.3\ncrosshair.color=red\nanimation.pack=game\n"
  .. "core.diagnosticLogs=until\nt.flag=false\nt.num=2.5\nt.carried=false\nother.module=42\n"
local OLD_VALUES = 9

check("the constants: reset level 1, a delay after the message widget is ready, a longer time rule without it, a duration, a text that names the mod",
  LEVEL == 1 and DELAY >= 1 and FALLBACK >= DELAY + 10 and PROBE.RESET_NOTICE_SECONDS > 0 and type(PROBE.RESET_NOTICE_TEXT) == "string"
  and PROBE.RESET_NOTICE_TEXT:find("Ultimate Third Person Project", 1, true) ~= nil and PROBE.RESET_NOTICE_TEXT:find("\n", 1, true) == nil
  and D_YAW ~= 60 and D_ZOOM ~= 0.3 and D_COLOR ~= "red", PROBE.RESET_NOTICE_TEXT)

-- Every scenario returns a list of { name, ok, detail }: on the shipped source each entry is a check; a mutation is caught when
-- any entry of its scenario fails.
local function expectations()
  local list = {}
  return list, function(name, ok, detail) list[#list + 1] = { name, ok and true or false, detail } end
end

-- ---------------------------------------------------------------- a settings file of an earlier version
local function oldFileScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local core = loadCore(mutation)
  local S = core.settings
  expect("every option that the core registers is on its default (no stored value is read)",
    core.Get("headLook.maxYaw") == D_YAW and core.Get("lootView.zoom") == D_ZOOM and core.Get("crosshair.color") == D_COLOR
    and core.Get("animation.pack") == "utpp" and core.Get("core.diagnosticLogs") == "off",
    tostring(core.Get("headLook.maxYaw")) .. " " .. tostring(core.Get("lootView.zoom")) .. " " .. tostring(core.Get("crosshair.color"))
    .. " " .. tostring(core.Get("animation.pack")) .. " " .. tostring(core.Get("core.diagnosticLogs")))
  expect("the animation pack in force is the mod's and the diagnostic logs are off", core.pack.applied == "utpp" and core.diag.mode == "off",
    tostring(core.pack.applied) .. " " .. tostring(core.diag.mode))
  expect("the configuration behind the options holds the defaults too", core.hl.cfg.maxYaw == D_YAW, core.hl.cfg.maxYaw)
  expect("nothing of the file is kept as a stored value (the value of the module that is not loaded is dropped as well)",
    S.stored["other.module"] == nil and S.stored["t.flag"] == nil and S.stored["t.num"] == nil)
  local disk = readFile(settingsFile) or ""
  expect("the file is rewritten at once: the current reset level, the note that the message is still to be shown, the defaults revision",
    count(disk, LEVEL_LINE) == 1 and count(disk, NOTICE_LINE) == 1 and count(disk, REVISION_LINE) == 1, disk)
  expect("the rewritten file holds none of the old values", disk:find("headLook.maxYaw=60", 1, true) == nil
    and disk:find("other.module", 1, true) == nil and disk:find("t.flag", 1, true) == nil and disk:find("crosshair.color=red", 1, true) == nil
    and disk:find("animation.pack=game", 1, true) == nil and disk:find("core.diagnosticLogs=until", 1, true) == nil, disk)
  expect("one log line tells it, with the number of dropped values", logCount("settings reset:") == 1
    and logCount(OLD_VALUES .. " stored values dropped") == 1, table.concat(logs, " | "))
  expect("the state: reset in force, message pending, the level of the file was 0", S.resetActive == true and S.resetNotice == true
    and S.fileReset == 0 and S.resetDropped == OLD_VALUES, tostring(S.resetActive) .. " " .. tostring(S.resetNotice) .. " "
    .. tostring(S.fileReset) .. " " .. tostring(S.resetDropped))
  local rec = registerLate(core)
  expect("an option that registers afterwards starts on its default and its callback sees the default", core.Get("t.flag") == true
    and rec.flag == true and core.Get("t.num") == 1.5, tostring(core.Get("t.flag")) .. " " .. tostring(rec.flag) .. " " .. tostring(core.Get("t.num")))
  expect("a value carried over from elsewhere is not taken after a reset: the option starts on its default", core.Get("t.carried") == true,
    tostring(core.Get("t.carried")))
  return list, core
end

-- ---------------------------------------------------------------- the message
local function messageScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local core = loadCore(mutation)
  registerLate(core)
  local m = mockScreen(core, nil, true)   -- the game's message widget does not exist yet (loading screen)
  local inMenu = play(core, FALLBACK + 5, false)
  expect("no message while a menu is open, however long", inMenu == 0 and #m.shown == 0 and m.calls == 0, m.calls)
  local loading = play(core, 5, true)
  expect("no message while the game's message widget does not exist yet (it would never be seen)",
    loading == 0 and #m.shown == 0 and m.calls == 0, m.calls)
  core.ResetNoticeScreen(true)            -- the HUD of the session is built
  local early = play(core, DELAY - 0.5, true)
  expect("no message before the delay after the widget is ready is over", early == 0 and #m.shown == 0 and m.calls == 0, m.calls)
  local now = play(core, 0.5, true)
  expect("the message is sent once the delay is over: the text and the duration of the module", now == 1 and #m.shown == 1
    and m.shown[1] ~= nil and m.shown[1].text == core.RESET_NOTICE_TEXT and m.shown[1].seconds == core.RESET_NOTICE_SECONDS, #m.shown)
  local disk = readFile(settingsFile) or ""
  expect("after the message the note is gone from the file and the reset level stays", count(disk, NOTICE_LINE) == 0
    and count(disk, LEVEL_LINE) == 1, disk)
  expect("after the message the file holds every registered option on its default (also the ones of the other modules)",
    disk:find("\nt.flag=true\n", 1, true) ~= nil and disk:find("\nt.num=1.5\n", 1, true) ~= nil
    and disk:find("\nt.carried=true\n", 1, true) ~= nil and disk:find("\nheadLook.maxYaw=" .. D_YAW .. "\n", 1, true) ~= nil, disk)
  local later = play(core, 30, true)
  expect("the message is not sent a second time", later == 0 and #m.shown == 1 and m.calls == 1, m.calls)
  expect("the log tells that the message was shown", logCount("settings reset: message shown") == 1, table.concat(logs, " | "))
  expect("the counters of the diagnostic logs are not used", core.diag.calls == 0 and core.diag.errors == 0, core.diag.calls)
  return list, core
end

-- ---------------------------------------------------------------- the message without the widget signal, and session ends
local function fallbackScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local core = loadCore(mutation)
  registerLate(core)
  local m = mockScreen(core, nil, true)   -- the signal of the game's message widget never comes
  local before = play(core, FALLBACK - 0.5, true)
  expect("without the widget signal: no message before the time rule of one session is over", before == 0 and m.calls == 0, m.calls)
  local now = play(core, 0.5, true)
  expect("without the widget signal: the message is sent when the time rule is over, once", now == 1 and #m.shown == 1
    and play(core, 60, true) == 0 and m.calls == 1, m.calls)

  writeFile(settingsFile, OLD_FILE)
  local core2 = loadCore(mutation)
  registerLate(core2)
  local m2 = mockScreen(core2, nil, true)
  play(core2, FALLBACK - 0.5, true)       -- almost the whole time rule in one session
  core2.ResetNoticeScreen(false)          -- the session ends (back to the main menu)
  local nextSession = play(core2, FALLBACK - 0.5, true)
  expect("session end: the play time of the ended session does not count in the next one", nextSession == 0 and m2.calls == 0, m2.calls)
  core2.ResetNoticeScreen(false)
  core2.ResetNoticeScreen(true)           -- the widget of a session is ready ...
  play(core2, DELAY - 0.5, true)
  core2.ResetNoticeScreen(false)          -- ... but the session ends before the delay is over
  local afterEnd = play(core2, DELAY + 3, true)
  expect("session end: the widget of the ended session does not count either", afterEnd == 0 and m2.calls == 0, m2.calls)
  core2.ResetNoticeScreen(true)
  expect("session end: the widget of the new session and the delay bring the message, once", play(core2, DELAY, true) == 1
    and #m2.shown == 1 and play(core2, 60, true) == 0, #m2.shown)
  return list
end

-- ---------------------------------------------------------------- the starts after the reset
local function nextStartScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local core = loadCore(mutation)
  registerLate(core)
  local m = mockScreen(core)
  play(core, DELAY + 1, true)
  core.Set("headLook.maxYaw", 70)   -- choices made after the reset
  core.Set("t.flag", false)
  local second = loadCore(mutation)
  local rec = registerLate(second)
  local m2 = mockScreen(second)
  expect("second start: nothing is reset, the choices made after the reset are read", second.Get("headLook.maxYaw") == 70
    and second.Get("t.flag") == false and rec.flag == false and logCount("settings reset:") == 0,
    tostring(second.Get("headLook.maxYaw")) .. " " .. tostring(second.Get("t.flag")))
  expect("second start: no reset in force and no message pending", second.settings.resetActive == false
    and second.settings.resetNotice == false and second.settings.fileReset == LEVEL)
  expect("second start: no message", play(second, DELAY + 30, true) == 0 and m2.calls == 0, m2.calls)
  second.Register({ key = "t.newcomer", section = "test", type = "bool", default = true, initial = false, label = "newcomer" })
  expect("second start: an option that is not stored takes its carried-over value again", second.Get("t.newcomer") == false)
  local disk = readFile(settingsFile) or ""
  expect("second start: the file still carries the reset level once and no note", count(disk, LEVEL_LINE) == 1 and count(disk, NOTICE_LINE) == 0, disk)
  expect("first start: the message was sent once", #m.shown == 1, #m.shown)
  return list
end

-- ---------------------------------------------------------------- the game is closed before a game session starts
local function closedEarlyScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local first = loadCore(mutation)
  registerLate(first)
  local m1 = mockScreen(first)
  play(first, 60, false)              -- main menu only
  first.Set("headLook.maxYaw", 70)    -- a change made in the menu
  local second = loadCore(mutation)
  registerLate(second)
  local m2 = mockScreen(second)
  expect("closed before a session: no message at the first start", m1.calls == 0)
  expect("closed before a session: the next start does not reset again and keeps the change made in the menu",
    second.Get("headLook.maxYaw") == 70 and logCount("settings reset:") == 0, tostring(second.Get("headLook.maxYaw")))
  expect("closed before a session: the next start still runs on the defaults of the reset (carried-over values are not taken)",
    second.settings.resetActive == true and second.Get("t.carried") == true, tostring(second.settings.resetActive))
  expect("closed before a session: the message is still pending and is sent once at the next start",
    second.settings.resetNotice == true and play(second, DELAY, true) == 1 and #m2.shown == 1 and play(second, 30, true) == 0
    and m2.calls == 1, m2.calls)
  local disk = readFile(settingsFile) or ""
  expect("closed before a session: after the message the note is gone", count(disk, NOTICE_LINE) == 0 and count(disk, LEVEL_LINE) == 1, disk)
  local third = loadCore(mutation)
  registerLate(third)
  local m3 = mockScreen(third)
  expect("closed before a session: the start after that has no message and no reset in force",
    play(third, DELAY + 30, true) == 0 and m3.calls == 0 and third.settings.resetActive == false and third.Get("headLook.maxYaw") == 70)
  return list
end

-- ---------------------------------------------------------------- a fresh install
local function freshInstallScenario(mutation)
  local list, expect = expectations()
  os.remove(settingsFile); os.remove(baseFile)
  local core = loadCore(mutation)
  registerLate(core)
  local m = mockScreen(core)
  expect("fresh install: nothing is reset and no message is pending", core.settings.resetActive == false
    and core.settings.resetNotice == false and core.settings.fileReset == nil and logCount("settings reset:") == 0)
  expect("fresh install: no message, however long the game is played", play(core, DELAY + 60, true) == 0 and m.calls == 0, m.calls)
  expect("fresh install: an option takes its carried-over value", core.Get("t.carried") == false)
  expect("fresh install: no file is written by the start itself", readFile(settingsFile) == nil)
  core.Set("headLook.maxYaw", 70)
  local disk = readFile(settingsFile) or ""
  expect("fresh install: the first saved file carries the current reset level and no note", count(disk, LEVEL_LINE) == 1
    and count(disk, NOTICE_LINE) == 0, disk)
  local second = loadCore(mutation)
  registerLate(second)
  local m2 = mockScreen(second)
  expect("fresh install: the next start reads that file (no reset, no message)", second.Get("headLook.maxYaw") == 70
    and second.settings.resetActive == false and play(second, DELAY + 30, true) == 0 and m2.calls == 0 and logCount("settings reset:") == 0)
  return list
end

-- ---------------------------------------------------------------- files of the current and of a later level
local function currentLevelScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, LEVEL_LINE .. "headLook.maxYaw=60\nother.module=42\nt.flag=false\n")
  local core = loadCore(mutation)
  local rec = registerLate(core)
  local m = mockScreen(core)
  expect("current level: the stored values are read, also by an option that registers later", core.Get("headLook.maxYaw") == 60
    and core.hl.cfg.maxYaw == 60 and core.Get("t.flag") == false and rec.flag == false and core.settings.stored["other.module"] == "42")
  expect("current level: nothing is reset, no message", core.settings.resetActive == false and core.settings.resetNotice == false
    and logCount("settings reset:") == 0 and play(core, DELAY + 30, true) == 0 and m.calls == 0)
  -- the level line may stand anywhere in the file
  writeFile(settingsFile, "headLook.maxYaw=60\nother.module=42\n" .. LEVEL_LINE)
  local last = loadCore(mutation)
  expect("current level: the level line counts wherever it stands in the file", last.Get("headLook.maxYaw") == 60
    and last.settings.resetActive == false)
  writeFile(settingsFile, PROBE.SETTINGS_RESET_KEY .. "=" .. (LEVEL + 4) .. "\nheadLook.maxYaw=60\n")
  local newer = loadCore(mutation)
  newer.Save()
  local disk = readFile(settingsFile) or ""
  expect("later level: a file of a later version is read and its level is kept when the file is saved",
    newer.Get("headLook.maxYaw") == 60 and newer.settings.resetActive == false
    and count(disk, PROBE.SETTINGS_RESET_KEY .. "=" .. (LEVEL + 4) .. "\n") == 1 and count(disk, PROBE.SETTINGS_RESET_KEY .. "=") == 1, disk)
  writeFile(settingsFile, PROBE.SETTINGS_RESET_KEY .. "=abc\nheadLook.maxYaw=60\n")
  local broken = loadCore(mutation)
  expect("a level that is not a number counts as level 0: the file is reset", broken.Get("headLook.maxYaw") == D_YAW
    and broken.settings.resetActive == true)
  -- a file that holds no value (empty, or a comment and the defaults revision only) lost nothing: no message
  for _, text in ipairs({ "", "# UTPP settings\ncore.defaultsRevision=1\n" }) do
    writeFile(settingsFile, text)
    local empty = loadCore(mutation)
    registerLate(empty)
    local m2 = mockScreen(empty)
    local disk2 = readFile(settingsFile) or ""
    expect("a file without any stored value (" .. #text .. " bytes): it gets the current level, no message is pending or shown",
      empty.settings.resetNotice == false and count(disk2, LEVEL_LINE) == 1 and count(disk2, NOTICE_LINE) == 0
      and play(empty, DELAY + 30, true) == 0 and m2.calls == 0, disk2)
  end
  return list
end

-- ---------------------------------------------------------------- the answers of the screen message call
local function answersScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local core = loadCore(mutation)
  registerLate(core)
  local m = mockScreen(core, { "refuse", "refuse" })
  local first = play(core, DELAY, true)
  local diskRefused = readFile(settingsFile) or ""
  expect("refused (no game session yet): nothing shown, the message stays pending in the file", first == 0 and m.calls == 1
    and #m.shown == 0 and core.settings.resetNotice == true and count(diskRefused, NOTICE_LINE) == 1, m.calls)
  local nextOnes = play(core, 1.0, true)
  expect("refused: asked again in the next frames until it is shown, then never again", nextOnes == 1 and m.calls == 3 and #m.shown == 1
    and play(core, 30, true) == 0 and m.calls == 3 and count(readFile(settingsFile) or "", NOTICE_LINE) == 0, m.calls)

  writeFile(settingsFile, OLD_FILE)
  local core2 = loadCore(mutation)
  registerLate(core2)
  local m2 = mockScreen(core2, { "error" })
  local shownNow = play(core2, DELAY, true)
  expect("failing call: the attempts end (the reset itself is done), the note is cleared and one line is logged",
    shownNow == 0 and m2.calls == 1 and core2.settings.resetNotice == false and count(readFile(settingsFile) or "", NOTICE_LINE) == 0
    and logCount("settings reset: the message could not be shown") == 1, m2.calls)
  expect("failing call: not called again", play(core2, 30, true) == 0 and m2.calls == 1, m2.calls)

  writeFile(settingsFile, OLD_FILE)
  local core3 = loadCore(mutation)
  core3.diag.api = {}                 -- no way to show a message
  core3.ResetNoticeScreen(true)
  local okT, res = pcall(play, core3, DELAY + 5, true)
  expect("no screen message call available: no error, the message stays pending", okT and res == 0 and core3.settings.resetNotice == true,
    tostring(res))
  return list
end

-- ---------------------------------------------------------------- Mod Settings: its values are not taken back
-- The player changed the option on the Mod Settings screen after the last sync (baseline 60, the screen shows 80). Without the
-- reset that edit is imported; after the reset nothing is taken from the screen and the default is marked for a push.
local function modSettingsScenario(mutation)
  local list, expect = expectations()
  writeFile(settingsFile, OLD_FILE)
  writeFile(baseFile, "# UTPP: values last synced with Mod Settings\nheadLook.maxYaw=60\n")
  local core = loadCore(mutation)
  local base = readFile(baseFile) or ""
  expect("Mod Settings: the values last synced are forgotten when the file is reset", base:find("headLook.maxYaw", 1, true) == nil, base)
  local imported, toPush = core.ModSettingsSync({ headLook_maxYaw = "80.000000" }, "start")
  expect("Mod Settings: the value its screen shows is not imported, the default stays and is marked for a push",
    imported == 0 and toPush == 1 and core.Get("headLook.maxYaw") == D_YAW and core.msync.dirty["headLook.maxYaw"] == true,
    tostring(imported) .. " " .. tostring(toPush) .. " " .. tostring(core.Get("headLook.maxYaw")))
  -- a fresh install has no baseline file: none is created by the start
  os.remove(settingsFile); os.remove(baseFile)
  loadCore(mutation)
  expect("Mod Settings: a fresh install creates no baseline file", readFile(baseFile) == nil)
  -- a reset without a baseline file creates none either
  writeFile(settingsFile, OLD_FILE)
  loadCore(mutation)
  expect("Mod Settings: a reset without a baseline file creates none", readFile(baseFile) == nil)
  return list
end

-- ---------------------------------------------------------------- a file of an earlier level read later in the same session
local function secondReadScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, LEVEL_LINE .. "headLook.maxYaw=60\nother.module=42\n")
  local core = loadCore(mutation)
  local rec = registerLate(core)
  core.Set("t.flag", false, { noSave = true })
  writeFile(settingsFile, "headLook.maxYaw=75\nother.module=43\n")
  core.Load()
  local disk = readFile(settingsFile) or ""
  expect("second read: every registered option goes to its default and its callback runs", core.Get("headLook.maxYaw") == D_YAW
    and core.hl.cfg.maxYaw == D_YAW and core.Get("t.flag") == true and rec.flag == true,
    tostring(core.Get("headLook.maxYaw")) .. " " .. tostring(core.Get("t.flag")))
  expect("second read: the stored values of before are dropped, the file holds none of them", core.settings.stored["other.module"] == nil
    and disk:find("other.module", 1, true) == nil and disk:find("headLook.maxYaw=75", 1, true) == nil and count(disk, LEVEL_LINE) == 1, disk)
  return list
end

-- ---------------------------------------------------------------- a later version raises the level
local function raisedLevelScenario()
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, LEVEL_LINE .. "headLook.maxYaw=60\n")
  local raised = loadCore({ "M.SETTINGS_RESET = " .. LEVEL .. "\n", "M.SETTINGS_RESET = " .. (LEVEL + 1) .. "\n" })
  local disk = readFile(settingsFile) or ""
  expect("raised level: a file of the level before is reset once more and rewritten with the new level",
    raised.SETTINGS_RESET == LEVEL + 1 and raised.Get("headLook.maxYaw") == D_YAW and raised.settings.resetNotice == true
    and count(disk, PROBE.SETTINGS_RESET_KEY .. "=" .. (LEVEL + 1) .. "\n") == 1 and disk:find("headLook.maxYaw=60", 1, true) == nil, disk)
  return list
end

-- ---------------------------------------------------------------- the game call path (the handlers that the game runs)
-- The module is loaded with the event registration of the game: onInit, then onUpdate at 60 frames per second. The screen
-- message must go through Game.UTPPShowMessage, only while a player exists, once.
local function gameWiringScenario(mutation)
  local list, expect = expectations()
  os.remove(baseFile)
  writeFile(settingsFile, OLD_FILE)
  local handlers, shown, player, observers = {}, {}, nil, {}
  local realOpen = SAVED.open
  io.open = function(p, m)
    if p == "utpp_settings.txt" then p = settingsFile elseif p == "utpp_modsettings_sync.txt" then p = baseFile end
    return realOpen(p, m)
  end
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  registerInput = function() end
  GetMod = function() return nil end
  Observe = nil
  ObserveAfter = function(cls, method, fn) observers[cls .. "." .. method] = fn end
  Game = setmetatable({
    UTPPLogLine = function() end,
    GetPlayer = function() return player end,
    UTPPShowMessage = function(text, seconds) shown[#shown + 1] = { text = text, seconds = seconds }; return true end,
  }, { __index = function() return function() return nil end end })
  print = function() end
  local ok, core
  if mutation then
    local a, b = SOURCE:find(mutation[1], 1, true)
    ok, core = pcall(function() return assert(loadstring(SOURCE:sub(1, a - 1) .. mutation[2] .. SOURCE:sub(b + 1), "=utpp_core_mutant"))() end)
  else
    ok, core = pcall(dofile, UTPP_CORE_PATH)
  end
  local function frames(n) for _ = 1, n do pcall(handlers.onUpdate, 1 / 60) end end
  local stage = "load"
  local okRun, err = pcall(function()
    assert(ok, core)
    assert(type(handlers.onInit) == "function" and type(handlers.onUpdate) == "function", "handlers not registered")
    handlers.onInit()
    local widgetBuilt = observers["WarningMessageGameController.OnInitialize"]
    expect("game path: the module observes the initialization of the game's message widget", type(widgetBuilt) == "function")
    stage = "menu"
    frames(600)                       -- main menu: no player
    expect("game path: no message in the main menu (no player)", #shown == 0, #shown)
    player = {}
    stage = "loading"
    frames(600)                       -- 10 s of a session whose HUD is not built yet (loading screen)
    expect("game path: no message while the message widget of the session does not exist", #shown == 0, #shown)
    player = nil
    frames(60)                        -- back to the main menu: the session ended
    player = {}
    frames(900)                       -- 15 s of the next session, HUD still not built
    expect("game path: the play time of an ended session does not count in the next one", #shown == 0, #shown)
    stage = "widget"
    widgetBuilt()                     -- the HUD of the session is built
    frames(math.floor(DELAY * 60) - 10)
    expect("game path: no message before the delay after the widget is ready is over", #shown == 0, #shown)
    stage = "shown"
    frames(20)
    expect("game path: the message is sent through the game's screen message call, once", #shown == 1 and shown[1] ~= nil
      and shown[1].text == core.RESET_NOTICE_TEXT and shown[1].seconds == core.RESET_NOTICE_SECONDS, #shown)
    frames(600)
    expect("game path: not sent again in the same session", #shown == 1, #shown)
    expect("game path: the note is gone from the file", count(readFile(settingsFile) or "", NOTICE_LINE) == 0
      and count(readFile(settingsFile) or "", LEVEL_LINE) == 1)
  end)
  print = SAVED.print
  io.open = realOpen
  registerForEvent, registerInput, Game, GetMod = nil, nil, nil, SAVED.GetMod
  Observe, ObserveAfter = SAVED.Observe, SAVED.ObserveAfter
  expect("game path: the handlers ran without an error (" .. stage .. ")", okRun, err)
  return list
end

local SCENARIOS = {
  { "old file", oldFileScenario }, { "message", messageScenario }, { "fallback", fallbackScenario }, { "next start", nextStartScenario },
  { "closed early", closedEarlyScenario }, { "fresh install", freshInstallScenario }, { "levels", currentLevelScenario },
  { "answers", answersScenario }, { "mod settings", modSettingsScenario }, { "second read", secondReadScenario },
  { "raised level", raisedLevelScenario }, { "game path", gameWiringScenario },
}
for _, s in ipairs(SCENARIOS) do
  for _, e in ipairs((s[2](nil))) do check(e[1], e[2], e[3]) end
end

-- ---------------------------------------------------------------- mutations of the shipped source
local MUTANTS = {
  { "an old file is read instead of reset", { "local reset = fileReset < M.SETTINGS_RESET", "local reset = false" }, oldFileScenario },
  { "the reset file is not rewritten at once", { "if #moved > 0 or reset then pcall(M.Save, path) end", "if #moved > 0 then pcall(M.Save, path) end" },
    oldFileScenario },
  { "a carried-over value is taken after a reset", { "and def.initial ~= nil and not S.resetActive then", "and def.initial ~= nil then" },
    oldFileScenario },
  { "the reset level is not written", { "  f:write(M.SETTINGS_RESET_KEY .. \"=\" .. math.max(M.SETTINGS_RESET, S.fileReset or 0) .. \"\\n\")\n", "" },
    nextStartScenario },
  { "the message is shown at every frame", { "  S.resetNotice = false\n  R.shown = ok == true", "  R.shown = ok == true" }, messageScenario },
  { "the message does not wait for the game's message widget", { "(R.ready and R.readyPlayed >= M.RESET_NOTICE_DELAY) or R.played >= M.RESET_NOTICE_FALLBACK",
      "R.played >= M.RESET_NOTICE_DELAY" }, messageScenario },
  { "the delay after the widget is ready is skipped", { "R.ready and R.readyPlayed >= M.RESET_NOTICE_DELAY", "R.ready" }, messageScenario },
  { "the widget signal is not used", { "(R.ready and R.readyPlayed >= M.RESET_NOTICE_DELAY) or ", "" }, messageScenario },
  { "the time rule without the widget signal is missing", { " or R.played >= M.RESET_NOTICE_FALLBACK", "" }, fallbackScenario },
  { "a session end keeps the play time", { "  if not R.ready then R.played = 0 end\n", "" }, fallbackScenario },
  { "a session end keeps the widget ready", { "  R.ready, R.readyPlayed = ready and true or false, 0\n", "  R.ready, R.readyPlayed = true, 0\n" }, fallbackScenario },
  { "the observer watches another method", { "\"WarningMessageGameController\", \"OnInitialize\"", "\"WarningMessageGameController\", \"OnOther\"" },
    gameWiringScenario },
  { "the game loop does not pass the session end on", { "      if inGame then M.ResetNoticeScreen(false) end -- the session ended: its HUD (and message widget) is gone\n", "" },
    gameWiringScenario },
  { "the message is sent while a menu is open", { "if not S.resetNotice or not playing then return false end", "if not S.resetNotice then return false end" },
    messageScenario },
  { "the file is not saved after the message", { "  R.shown = ok == true\n  pcall(M.Save)", "  R.shown = ok == true" }, messageScenario },
  { "the pending message is not written to the file", { "  if S.resetNotice then f:write(M.RESET_NOTICE_KEY .. \"=1\\n\") end\n", "" }, closedEarlyScenario },
  { "the pending message of the file is ignored", { "  elseif fileNotice then", "  elseif false then" }, closedEarlyScenario },
  { "a refused call ends the attempts", { "  if ok and res == false then return false end\n", "" }, answersScenario },
  { "the Mod Settings baseline is kept", { "if M.ModSettingsForgetBase then pcall(M.ModSettingsForgetBase) end", "" }, modSettingsScenario },
  { "a reset creates a baseline file", { "  if not f then return true end -- nothing was synced yet: no file to empty\n  f:close()\n", "  if f then f:close() end\n" },
    modSettingsScenario },
  { "the values of before stay stored", { "    for k in pairs(S.stored) do S.stored[k] = nil end\n", "" }, secondReadScenario },
  { "the options keep their values", { "    for _, key in ipairs(S.order) do S.values[key] = S.defs[key].default end\n", "" }, secondReadScenario },
  { "a fresh install is told about a reset", { "S.resetNotice = false   -- the message is still to be shown", "S.resetNotice = true" },
    freshInstallScenario },
  { "the game loop does not run the message step", { "    pcall(M.ResetNoticeTick, dt, not menu)\n", "" }, gameWiringScenario },
  { "an empty file is told about a reset", { "true, S.resetNotice or dropped > 0, dropped", "true, true, dropped" }, currentLevelScenario },
  { "a file with values is reset without a message", { "true, S.resetNotice or dropped > 0, dropped", "true, false, dropped" }, messageScenario },
}
for _, mu in ipairs(MUTANTS) do
  -- an anchor that no longer exists exactly once is a failure of the test itself
  local a, b = SOURCE:find(mu[2][1], 1, true)
  assert(a and not SOURCE:find(mu[2][1], b + 1, true), "mutation anchor must exist exactly once: " .. mu[2][1])
  local okS, list = pcall(mu[3], mu[2])
  local caught = not okS
  if okS then for _, e in ipairs(list) do if not e[2] then caught = true end end end
  check("mutation [" .. mu[1] .. "] is caught", caught)
end

registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
io.open = SAVED.open
print = SAVED.print
os.remove(settingsFile)
os.remove(baseFile)
print(string.format("utpp_settings_reset test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_settings_reset: " .. failed .. " check(s) failed")

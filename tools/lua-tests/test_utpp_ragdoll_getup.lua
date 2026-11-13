-- UTPP offline test: CET key ragdoll.getUp (native source pack get-up after a car-hit ragdoll, utpp_ragdoll.reds). Registered default ON,
-- developer-level placement (the Mod Settings page is unchanged), pushed through Game.UTPPRagdollSetGetUpEnabled with the session rules
-- of the other ragdoll keys (acknowledged value, one retry per second, re-push in a new session). The four-argument
-- Game.UTPPRagdollSetConfig call stays as it was. Pure logic, no game.
local results = { pass = 0, fail = 0 }
local outPrint = print
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  outPrint(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local savedPrint, savedRegister, savedGame = print, registerForEvent, Game
local tmp = os.tmpname()
local function load()
  registerForEvent = nil
  local M = dofile(UTPP_CORE_PATH)
  registerForEvent = savedRegister
  M.sink = function() end
  M.SETTINGS_FILE = tmp
  return M
end

do -- registration
  Game = nil
  os.remove(tmp)
  local X = load()
  local def = X.settings.defs["ragdoll.getUp"]
  check("ragdoll.getUp: registered bool, default ON, section ragdoll, label 'UTPP get-up animation'", def ~= nil and def.type == "bool"
    and def.default == true and def.section == "ragdoll" and def.label == "UTPP get-up animation" and X.Get("ragdoll.getUp") == true)
  check("ragdoll.getUp: description names the fallback", def ~= nil and def.desc:find("knockdown recovery", 1, true) ~= nil, def and def.desc)
  local lv, grp = X.MenuPlacement("ragdoll.getUp")
  check("ragdoll.getUp: developer level, Vehicles group (next to ragdoll.getUpPush)", lv == "developer" and grp == "vehicles"
    and select(1, X.MenuPlacement("ragdoll.getUpPush")) == "developer", tostring(lv) .. "/" .. tostring(grp))
  -- 2026-11-09 release 0.1.0 scope: ragdoll.enabled default OFF (no ragdoll archives shipped), developer level
  check("ragdoll.getUp: old keys untouched (enabled default OFF since 0.1.0, getUpPush default ON)", X.Get("ragdoll.enabled") == false
    and X.Get("ragdoll.getUpPush") == true)
  check("ragdoll.getUp: Game == nil -> no call, no error, nothing acknowledged", X.rgu.calls == 0 and X.rgu.errors == 0 and X.rgu.sent == nil)
  X.Set("ragdoll.getUp", false)
  check("ragdoll.getUp: outside a session a Set does not call", X.rgu.calls == 0 and X.rgu.sent == nil and X.Get("ragdoll.getUp") == false)
  X.Set("ragdoll.getUp", true)
  check("ragdoll.getUp: invalid value rejected, value kept", X.Set("ragdoll.getUp", "maybe") == nil and X.Get("ragdoll.getUp") == true)
end

do -- session rules
  Game = nil
  os.remove(tmp)
  local X = load()
  local calls, cfgCalls = {}, {}
  X.rgu.api = function(v) calls[#calls + 1] = v; return true end
  X.rg.api = function(...) cfgCalls[#cfgCalls + 1] = select("#", ...); return true end
  X.CrosshairResync(10, true)
  check("session: first frame pushes get-up ON exactly once", #calls == 1 and calls[1] == true and X.rgu.sent == "true")
  check("session: the four-argument ragdoll config push is separate and unchanged", #cfgCalls == 1 and cfgCalls[1] == 4)
  X.CrosshairResync(11)
  X.Set("ragdoll.getUp", true)
  check("session: later frames / same value -> no second get-up push", #calls == 1)
  X.Set("ragdoll.getUp", false)
  check("session: OFF -> one push (false), persisted", #calls == 2 and calls[2] == false and X.rgu.sent == "false"
    and X.Get("ragdoll.getUp") == false)
  check("session: changing get-up does not re-push the ragdoll config", #cfgCalls == 1)
  local errors = X.rgu.errors
  X.rgu.api = function() error("script missing") end
  X.Set("ragdoll.getUp", true)
  check("session: failing call counted, not acknowledged", X.rgu.errors == errors + 1 and X.rgu.sent == nil)
  X.rgu.api = function() return false end
  X.CrosshairResync(12)
  check("session: a call returning false counted, still not acknowledged", X.rgu.errors == errors + 2 and X.rgu.sent == nil)
  X.CrosshairResync(12.5)
  check("session: retry throttled to once per second", X.rgu.errors == errors + 2)
  X.rgu.api = function(v) calls[#calls + 1] = v; return true end
  X.CrosshairResync(13.1)
  check("session: retry after a second sends the stored value", #calls == 3 and calls[3] == true and X.rgu.sent == "true")
  local n = #calls
  X.CrosshairResync(20, true)
  check("session: new session re-pushes the stored value", #calls == n + 1 and calls[#calls] == true)
  X.Set("ragdoll.getUp", false)
  X.CrosshairResync(30, true)
  check("session: stored OFF is re-pushed as false in the next session", calls[#calls] == false and X.rgu.sent == "false")
end

do -- stored OFF at boot, round trip
  Game = nil
  local f = assert(io.open(tmp, "w")); f:write("ragdoll.getUp=false\n"); f:close()
  local Y = load()
  Y.Load()
  local yCalls = {}
  Y.rgu.api = function(v) yCalls[#yCalls + 1] = v; return true end
  Y.rg.api = function() return true end
  Y.CrosshairResync(0, true)
  Y.CrosshairResync(5)
  check("boot: stored OFF -> exactly one push (false)", #yCalls == 1 and yCalls[1] == false and Y.Get("ragdoll.getUp") == false)
  Y.Set("ragdoll.getUp", true)
  Y.Save(tmp)
  local text = io.open(tmp):read("*a")
  check("save: the key is written", text:find("ragdoll.getUp=true", 1, true) ~= nil, text)
  Y.ResetAll()
  check("reset all: back to ON", Y.Get("ragdoll.getUp") == true)
end

do -- the game call path
  local gCalls, cCalls = {}, {}
  Game = { UTPPLogLine = function() end,
           UTPPRagdollSetGetUpEnabled = function(v) gCalls[#gCalls + 1] = v; return true end,
           UTPPRagdollSetConfig = function(e, s, m, g) cCalls[#cCalls + 1] = { e, s, m, g }; return true end }
  os.remove(tmp)
  local Z = load()
  check("game path: no call at load (no session)", #gCalls == 0 and Z.rgu.sent == nil)
  Z.CrosshairResync(0, true)
  check("game path: session edge -> Game.UTPPRagdollSetGetUpEnabled(true)", #gCalls == 1 and gCalls[1] == true and Z.rgu.sent == "true")
  check("game path: Game.UTPPRagdollSetConfig(enabled, scale, seconds, getUpPush) still called with four values", #cCalls == 1
    and cCalls[1][1] == false and cCalls[1][2] == 1.0 and cCalls[1][3] == 2.5 and cCalls[1][4] == true)
  Game = { UTPPLogLine = function() end, UTPPRagdollSetConfig = function() return true end }
  local W = load()
  W.CrosshairResync(0, true)
  check("game path: an older script layer without the function -> counted, retried, no error thrown", W.rgu.errors == 1 and W.rgu.sent == nil)
end

os.remove(tmp)
Game = savedGame
print = savedPrint
print(string.format("utpp_ragdoll_getup test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_ragdoll_getup test failed")

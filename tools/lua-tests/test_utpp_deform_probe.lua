local events, arms, flushes, census = {}, 0, 0, false
local menu, paused, player, busy = false, false, {}, false
registerForEvent = function(name, callback) events[name] = callback end
GetAllBlackboardDefs = function() return { UI_System = { IsInMenu = 1 } } end
Game = {
  GetPlayer = function() return player end,
  GetBlackboardSystem = function() return { Get = function() return { GetBool = function() return menu end } end } end,
  GetSystemRequestsHandler = function() return { IsGamePaused = function() return paused end } end,
  UTPPLogLine = function() end,
  UTPPPoseCensus = function(on) census = on; return true end,
  UTPPPoseCensusDump = function() return "census.csv" end,
  UTPPBodyPoseTraceStatus = function() return "frames=200" end,
  UTPPBodyPoseTraceArm = function(n) assert(n == 200); arms = arms + 1; return true end,
  UTPPBodyPoseTraceFlush = function() flushes = flushes + 1; return not busy end
}
local M = dofile(UTPP_DEFORM_PROBE_PATH)
assert(not M.running and arms == 0)
events.onInit(); events.onUpdate(1); assert(arms == 0)
assert(M.Start() and not M.Start())
events.onOverlayOpen(); events.onUpdate(1); assert(arms == 0)
events.onOverlayClose(); events.onUpdate(0); assert(arms == 1 and census)
menu = true; events.onUpdate(1); assert(not M.capture and not census)
local elapsed = M.elapsed; events.onUpdate(1); assert(M.elapsed == elapsed)
menu = false; paused = true; events.onUpdate(1); assert(arms == 1)
paused = false; player = nil; events.onUpdate(1); assert(arms == 1)
player = {}; for i = 1, 8 do events.onUpdate(1) end; assert(arms == 2)
busy = true; assert(not M.Stop() and M.capture)
busy = false; events.onUpdate(0); assert(not M.capture)
assert(M.Start()); events.onUpdate(0)
for i = 1, 96 do events.onUpdate(1) end
assert(not M.running and not M.capture and M.cycles == 12 and arms == 14 and not census)
assert(M.Status():find("COMPLETE"))
print("deform probe targeted stub PASS")

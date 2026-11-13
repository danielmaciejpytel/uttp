-- Explicit read-only diagnostic capture. Never changes settings or pose data.
local M = { running = false, cycles = 0, elapsed = 0, overlay = false, capture = false,
  MAX_CYCLES = 12, INTERVAL = 8, FRAMES = 200, last = "DISARMED", paused = false }
local function native(name, ...)
  local args = { ... }
  return pcall(function() return Game[name](unpack(args)) end)
end
local function log(message)
  local line = "[utpp_deform_probe] " .. message
  print(line)
  native("UTPPLogLine", line)
end
local function blocked()
  if M.overlay then return true end
  local ok, player = pcall(function() return Game.GetPlayer() end)
  if not ok or player == nil then return true end
  local menuOk, menu = pcall(function()
    local d = GetAllBlackboardDefs().UI_System
    return Game.GetBlackboardSystem():Get(d):GetBool(d.IsInMenu)
  end)
  if not menuOk or menu then return true end
  local pauseOk, paused = pcall(function() return Game.GetSystemRequestsHandler():IsGamePaused() end)
  return not pauseOk or paused == true
end
local function flush(reason)
  if not M.capture then return true end
  local ok, result = native("UTPPBodyPoseTraceFlush")
  if not ok or result ~= true then
    M.last = "flush pending: " .. tostring(result)
    return false
  end
  M.capture = false
  native("UTPPPoseCensus", false)
  local dumpOk, path = native("UTPPPoseCensusDump")
  local statusOk, status = native("UTPPBodyPoseTraceStatus")
  log("END cycle=" .. M.cycles .. " reason=" .. reason .. " trace=" .. tostring(statusOk and status or "unavailable") ..
    " census=" .. tostring(dumpOk and path or "unavailable"))
  return true
end
function M.Start()
  if M.running or M.capture then return false end
  M.running, M.cycles, M.elapsed, M.paused = true, 0, M.INTERVAL, false
  M.last = "Waiting for gameplay; max 12 captures"
  log("START explicit; frames=200 cycles=12 interval=8s; no settings or pose writes")
  return true
end
function M.Stop()
  M.running = false
  local ok = flush("stop")
  M.last = ok and "STOPPED" or M.last
  return ok
end
function M.Status()
  return M.last .. " running=" .. tostring(M.running) .. " cycle=" .. M.cycles .. "/12 capture=" .. tostring(M.capture)
end
function M.Update(delta)
  if not M.running then
    if M.capture then flush("stop retry") end
    return
  end
  if blocked() then
    if not M.paused then log("PAUSE gameplay unavailable/menu/CET"); M.paused = true end
    flush("pause")
    return
  end
  if M.paused then log("RESUME"); M.paused = false end
  M.elapsed = M.elapsed + math.max(0, math.min(tonumber(delta) or 0, 1))
  if M.elapsed < M.INTERVAL then return end
  if not flush("interval") then return end
  if M.cycles >= M.MAX_CYCLES then
    M.running = false; M.last = "COMPLETE 12/12"; log(M.last); return
  end
  M.elapsed = 0
  M.cycles = M.cycles + 1
  local censusOk, census = native("UTPPPoseCensus", true)
  local ok, armed = native("UTPPBodyPoseTraceArm", M.FRAMES)
  M.capture = ok and armed == true
  if not M.capture then native("UTPPPoseCensus", false) end
  M.last = M.capture and "Recording" or ("Arm failed: " .. tostring(armed))
  log("BEGIN cycle=" .. M.cycles .. " armed=" .. tostring(M.capture) .. " census=" .. tostring(censusOk and census == true))
end
if type(registerForEvent) == "function" then
  registerForEvent("onInit", function() log("DISARMED; open CET > Start deformation capture") end)
  registerForEvent("onUpdate", M.Update)
  registerForEvent("onOverlayOpen", function() M.overlay = true end)
  registerForEvent("onOverlayClose", function() M.overlay = false end)
  registerForEvent("onShutdown", M.Stop)
  registerForEvent("onDraw", function()
    if not M.overlay then return end
    if ImGui.Begin("UTPP deformation capture") then
      ImGui.TextWrapped("Read-only diagnostics. Start once, close CET and play for about 96 seconds.")
      ImGui.TextWrapped("Show shoulders from front/back: neutral stance, Ready, Aim, Shoot, Reload; then crouch. Keep the same weapon for each sequence.")
      ImGui.TextWrapped("Capture pauses in menus and while CET is open. No settings or poses are changed.")
      ImGui.TextWrapped(M.Status())
      if not M.running and not M.capture then
        if ImGui.Button("Start deformation capture") then M.Start() end
      elseif ImGui.Button("Stop and flush") then M.Stop() end
    end
    ImGui.End()
  end)
end
return M

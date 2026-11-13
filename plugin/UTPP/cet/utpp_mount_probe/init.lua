-- Finite read-only capture; no controller input, feature writes or saved settings.
local M={active=false,samples=0,capture=0,elapsed=0,nextSample=0}
local function log(s)
  -- The native UTPP logger truncates long route rows. CET's scripting log keeps
  -- the complete snapshot, so calibration must read that copy.
  print('[utpp_mount_route] '..s)
  pcall(function()Game.UTPPLogLine('[utpp_mount_route] '..s)end)
end
function M.Stop(reason)
  if M.active then log('DONE capture='..M.capture..' samples='..M.samples..' reason='..tostring(reason)) end
  M.active=false
end
function M.Start()
  M.Stop('restart');M.capture=M.capture+1;M.samples=0;M.elapsed=0;M.nextSample=0;M.active=true
  log('START capture='..M.capture);return true
end
function M.Tick(dt)
  if not M.active then return end
  if type(dt)~='number' or dt~=dt or dt<0 or dt==math.huge then M.Stop('invalid-dt');return end
  M.elapsed=M.elapsed+dt
  if M.elapsed<M.nextSample then return end
  M.nextSample=M.elapsed+.1
  local ok,row=pcall(function()return Game.UTPPNueMountRoute()end)
  if not ok then log('ERROR '..tostring(row));M.Stop('native-error');return end
  if type(row)~='string' then log('ERROR result-type='..type(row)..' value='..tostring(row));M.Stop('invalid-result');return end
  if row:match('^unavailable:') then log('ERROR '..row);M.Stop('unavailable');return end
  M.samples=M.samples+1
  log('DATA capture='..M.capture..' sample='..M.samples..' '..row)
  if M.samples>=40 or M.elapsed>=5 then M.Stop('limit') end
end
if registerForEvent then
  registerForEvent('onUpdate',M.Tick)
  registerForEvent('onShutdown',function()M.Stop('shutdown')end)
end
return M

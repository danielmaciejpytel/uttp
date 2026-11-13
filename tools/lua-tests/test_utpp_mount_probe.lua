local events,logs={},{}
registerForEvent=function(n,f)events[n]=f end
Game={UTPPLogLine=function(s)logs[#logs+1]=s end,UTPPNueMountRoute=function()return 'playerTemplate=test|weaponRightSlots=1'end}
local M=dofile(UTPP_MOUNT_PROBE_PATH)
assert(not M.active);events.onUpdate(.2);assert(#logs==0)
M.Start();for i=1,60 do events.onUpdate(.1)end
assert(not M.active and M.samples==40 and logs[#logs]:match('DONE'))
M.Start();Game.UTPPNueMountRoute=function()return 'unavailable:drawn-nue-default'end;events.onUpdate(.1);assert(not M.active and M.samples==0)
M.Start();events.onUpdate(0/0);assert(not M.active)
M.Start();Game.UTPPNueMountRoute=function()error('missing native')end;events.onUpdate(.1);assert(not M.active)
M.Start();events.onShutdown();assert(not M.active)
print('mount capture lifecycle PASS: idle, cap, unavailable, invalid dt, missing bridge, shutdown')

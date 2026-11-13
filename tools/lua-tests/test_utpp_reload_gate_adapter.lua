package.path=UTPP_RELOAD_GATE_DIR..'/?.lua;'..package.path
local events,hooks,calls={},{},{}
registerForEvent=function(n,f)events[n]=f end
local timing={}
Observe=function()error('ambiguous Observe must not be used')end
ObserveAfter=function(c,n,f)assert(c=='ReloadEvents');hooks[n]=f;timing[n]='after' end
ObserveBefore=function(c,n,f)assert(c=='ReloadEvents');hooks[n]=f;timing[n]='before' end
local M=dofile(UTPP_RELOAD_GATE_DIR..'/init.lua')
local engineApi=M.api
local count=0;local function check(x)assert(x,'check '..(count+1));count=count+1 end
local o={enabled=true,assetsReady=true,player=true,female=true,tpp=true,weapon='Items.Preset_Nue_Default',empty=true,action=true,aim=true,grip=2,elapsed=0,speed=1,owner='player/weapon1',session='session1'}
local owner='player/weapon1';local fail=false;local readFail=false
local refreshCalls=0;local refreshAccepted=true
M.api={refresh=function()refreshCalls=refreshCalls+1;return refreshAccepted end,context=function()if readFail then error('context error')end;return o end,owner=function()return owner end,snapshot=function()if readFail then error('read error')end;return o end,
 publish=function(on,aim,twoHand)calls[#calls+1]={on,aim,twoHand};if fail then error('publish error')end;return true end}
local iface={executionOwner={IsPlayer=function()return true end}}
local npc={executionOwner={IsPlayer=function()return false end}}
-- Release default (2026-10-22): the bundle flags are ready, the gate is enabled by onInit once its observers are installed.
check(not M.enabled and not M.ready and M.assetsReady and M.normalAssetsReady and M.twoHandAssetsReady)
check(not M.SetEnabled(true)) -- not ready before onInit
events.onInit();check(M.ready and hooks.OnForcedExit and M.enabled)
check(timing.OnEnter=='after' and timing.OnUpdate=='after')
check(timing.OnExit=='before' and timing.OnForcedExit=='before')
M.SetEnabled(false);check(not M.enabled)
M.Handle('enter',{},iface);check(#calls==0) -- disabled
check(M.SetEnabled(true));M.SetBundleReady(false);M.Handle('enter',{},iface);check(#calls==0) -- no bundle
M.SetBundleReady(true);hooks.OnEnter({},nil,iface);check(M.state.active and #calls==1 and calls[1][1] and calls[1][2])
o.aim=false;o.elapsed=.1;hooks.OnUpdate({},.1,nil,iface);check(#calls==1 and M.state.aim)
hooks.OnExit({},nil,npc);check(M.state.active and #calls==1)
owner='player/old';hooks.OnExit({},nil,iface);check(M.state.active)
owner='player/weapon1';hooks.OnForcedExit({},nil,iface);check(not M.state.active and #calls==2 and not calls[2][1])
M.Handle('sample',{},iface);check(#calls==2) -- no late reentry
hooks.OnEnter({},nil,iface);check(M.state.active and #calls==3 and not calls[3][2])
o.tpp=false;events.onUpdate(.01);check(not M.state.active and not calls[#calls][1]);o.tpp=true;M.Handle('sample',{},iface);check(not M.state.active)
hooks.OnEnter({},nil,iface);check(M.state.active);events.onUpdate(.26);check(not M.state.active)
hooks.OnEnter({},nil,iface);check(M.state.active);readFail=true;events.onUpdate(.01);check(not M.state.active);readFail=false
fail=true;hooks.OnEnter({},nil,iface);check(not M.state.active and M.pendingClear);fail=false;events.onUpdate(.01);check(not M.pendingClear and not calls[#calls][1])
hooks.OnEnter({},nil,iface);check(M.state.active);M.SetBundleReady(false);check(not M.state.active and not calls[#calls][1])
M.SetBundleReady(true);hooks.OnEnter({},nil,iface);check(M.state.active);events.onShutdown();check(not M.enabled and not M.state.active and not calls[#calls][1])
-- Borrowed native callback arguments can lose methods between callbacks.
M.SetBundleReady(true);check(M.SetEnabled(true));hooks.OnEnter({},nil,iface)
local savedSnapshot=M.api.snapshot
M.api.snapshot=function()error('borrowed interface expired')end
events.onUpdate(.01);check(M.state.active) -- fresh context, no borrowed reads
local savedContext=M.api.context
M.api.context=function()local c={};for k,v in pairs(o)do c[k]=v end;c.owner='player/weapon2';return c end
events.onUpdate(.01);check(not M.state.active and not calls[#calls][1])
M.api.context=savedContext;M.api.snapshot=savedSnapshot
check(M.observed.enter>0 and M.observed.sample>0 and M.lastSnapshot==o)
check(M.lastReadError==nil)
-- An ordinary exit must not switch to the FPP reload underneath MM's 200ms
-- handback fade. Late samples and duplicate exits cannot prolong the tail.
o.elapsed=0;hooks.OnEnter({},nil,iface)
o.elapsed=3.4;hooks.OnUpdate({},.1,nil,iface);local beforeExit=#calls
hooks.OnExit({},nil,iface);events.onUpdate(.10)
check(M.state.active and #calls==beforeExit)
hooks.OnExit({},nil,iface);hooks.OnUpdate({},.1,nil,iface);events.onUpdate(.10)
check(M.state.active and #calls==beforeExit)
events.onUpdate(.051);check(not M.state.active and not calls[#calls][1])
-- Recorded case: native gameplay ended after ~2.27s at speed 1.251704, but
-- the 3.333s magazine clip still needed ~0.39s to emit reload_end. A fixed
-- 0.25s tail would select the original reload again before that event.
o.elapsed=0;o.speed=1.251704;hooks.OnEnter({},nil,iface)
o.elapsed=2.27;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
beforeExit=#calls;events.onUpdate(.251)
check(M.state.active and #calls==beforeExit)
hooks.OnExit({},nil,iface);hooks.OnUpdate({},.1,nil,iface)
events.onUpdate(.40);check(not M.state.active and not calls[#calls][1])
-- Slow reload retains the branch until its own end; fast reload does not
-- inherit the previous episode's clock or wait for the unscaled duration.
o.elapsed=0;o.speed=.5;hooks.OnEnter({},nil,iface)
o.elapsed=4;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
events.onUpdate(2.5);check(M.state.active)
events.onUpdate(.42);check(not M.state.active)
o.elapsed=0;o.speed=2;hooks.OnEnter({},nil,iface)
o.elapsed=1.5;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
events.onUpdate(.2);check(M.state.active)
events.onUpdate(.22);check(not M.state.active)
-- Changing speed during a reload must not retroactively apply the new speed
-- to its whole duration (which would prematurely finish a slow-first clip).
o.elapsed=0;o.speed=.5;hooks.OnEnter({},nil,iface)
o.elapsed=2;hooks.OnUpdate({},.1,nil,iface)
o.speed=2;hooks.OnUpdate({},.1,nil,iface)
o.elapsed=2.5;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
events.onUpdate(.5);check(M.state.active)
events.onUpdate(.42);check(not M.state.active)
o.speed=1;o.elapsed=0
-- Safety boundaries still clear immediately during the exit fade.
hooks.OnEnter({},nil,iface);hooks.OnExit({},nil,iface)
o.tpp=false;events.onUpdate(.01);check(not M.state.active and M.exitTail==0);o.tpp=true
hooks.OnEnter({},nil,iface);hooks.OnExit({},nil,iface)
M.api.context=function()local c={};for k,v in pairs(o)do c[k]=v end;c.owner='player/weapon2';return c end
events.onUpdate(.01);check(not M.state.active and M.exitTail==0);M.api.context=savedContext
hooks.OnEnter({},nil,iface);hooks.OnExit({},nil,iface)
hooks.OnForcedExit({},nil,iface);check(not M.state.active and M.exitTail==0)
-- A new partial reload replaces the old episode instead of inheriting its flag.
hooks.OnEnter({},nil,iface);hooks.OnExit({},nil,iface);o.empty=false
hooks.OnEnter({},nil,iface);check(not M.state.active and M.exitTail==0 and not calls[#calls][1]);o.empty=true
hooks.OnEnter({},nil,iface);hooks.OnExit({},nil,iface);events.onShutdown()
check(not M.enabled and not M.state.active and M.exitTail==0)
-- NORMAL has an independent readiness opt-in and an 80/30 clip clock.
M.SetBundleReady(true);check(M.SetEnabled(true));o.empty=false;o.elapsed=0;o.speed=1
hooks.OnEnter({},nil,iface);check(not M.state.active and not M.normalAssetsReady)
M.SetNormalBundleReady(true);o.normalAssetsReady=true
hooks.OnEnter({},nil,iface);check(M.state.active and M.state.empty==false)
o.elapsed=2.4;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
events.onUpdate(.50);check(M.state.active)
events.onUpdate(.02);check(not M.state.active) -- EMPTY would remain active here
o.elapsed=0;o.speed=2;hooks.OnEnter({},nil,iface)
o.elapsed=1.2;hooks.OnUpdate({},.1,nil,iface);hooks.OnExit({},nil,iface)
events.onUpdate(.38);check(M.state.active);events.onUpdate(.01);check(not M.state.active)
o.elapsed=0;o.speed=1;hooks.OnEnter({},nil,iface)
M.SetNormalBundleReady(false);check(not M.state.active and M.exitTail==0)
o.normalAssetsReady=false;hooks.OnEnter({},nil,iface);check(not M.state.active)
M.SetNormalBundleReady(true);o.normalAssetsReady=true;hooks.OnEnter({},nil,iface)
o.empty=true;o.elapsed=.1;hooks.OnUpdate({},.1,nil,iface);check(not M.state.active) -- no mid-episode type swap
o.elapsed=0;hooks.OnEnter({},nil,iface);check(M.state.active and M.state.empty)
o.empty=false;o.elapsed=.1;hooks.OnUpdate({},.1,nil,iface);check(not M.state.active)
events.onShutdown();check(not M.enabled and not M.state.active)
-- Never select a 2H leaf from an older 1H-only package.
M.SetBundleReady(true);check(not M.twoHandAssetsReady);check(M.SetEnabled(true))
o.empty=true;o.elapsed=0;o.aim=true;o.grip=3;o.twoHandAssetsReady=false
hooks.OnEnter({},nil,iface);check(not M.state.active)
M.SetTwoHandBundleReady(true);o.twoHandAssetsReady=true
hooks.OnEnter({},nil,iface);check(M.state.active and M.state.grip==3 and calls[#calls][3]==true)
local receivedAim=nil
M.api.snapshot=function(_,_,latchedAim)receivedAim=latchedAim;return o end
o.aim=false;o.elapsed=.1;hooks.OnUpdate({},.1,nil,iface)
check(receivedAim==true and M.state.aim==true and M.state.active)
o.grip=2;events.onUpdate(.01);check(not M.state.active and calls[#calls][3]==false)
o.grip=3;o.elapsed=0;hooks.OnEnter({},nil,iface);check(M.state.active)
M.SetTwoHandBundleReady(false);check(not M.state.active and not calls[#calls][1] and not calls[#calls][3])
M.SetTwoHandBundleReady(true);M.SetBundleReady(true);check(not M.twoHandAssetsReady)
events.onShutdown();check(not M.enabled and not M.state.active)
-- Exercise the actual adapter: actual ADS may change while the reload family
-- must still be resolved from its latched entry state (Ready 1H / Aim 2H).
local readAim=nil;local nativeCall=nil
local player={IsPlayer=function()return true end,GetResolvedGenderName=function()return 'Female' end,GetEntityID=function()return {hash=1}end}
local weapon={GetEntityID=function()return {hash=2}end,GetItemID=function()return 'Items.Preset_Nue_Default'end}
Game={GetPlayer=function()return player end,GetTransactionSystem=function()return {GetItemInSlot=function()return weapon end}end,
 UTPPNueReloadNativeInputs=function()return 'empty=true;loop=false;speed=1' end,
 UTPPNueReloadGrip=function(aim)readAim=aim;return aim and 3 or 2 end,
 UTPPPublishNueReloadGate=function(on,aim,twoHand)nativeCall={on,aim,twoHand};return true end}
NameToString=function(v)return v end;ItemID={GetTDBID=function(v)return v end};TDBID={ToStringDEBUG=function(v)return v end};TweakDBID={new=function(v)return v end}
GetMod=function()return {Desired=function()return {perspective='TPP'}end,Status=function()return 'session=live'end,IsAiming=function()return true end}end
local borrowed={executionOwner=player,owner=weapon}
local sample=engineApi.snapshot({GetInStateTime=function()return .1 end},borrowed,false)
check(sample.aim==true and sample.grip==2 and readAim==false)
sample=engineApi.snapshot({GetInStateTime=function()return 0 end},borrowed,nil)
check(sample.grip==3 and readAim==true)
M.state.aim=false;check(engineApi.context().grip==2 and readAim==false)
check(engineApi.publish(true,true,true) and nativeCall[3]==true)
check(engineApi.publish(false,false,false) and nativeCall[3]==false)
print('reload gate adapter '..count..'/'..count..' PASS')

-- Permission refresh does not republish graph features, and fails closed.
M.SetBundleReady(true);M.SetEnabled(true);o.elapsed=0;o.grip=2;o.tpp=true
hooks.OnEnter({},nil,iface)
local featureCalls=#calls;local refreshBefore=refreshCalls
events.onUpdate(.01)
check(M.state.active and refreshCalls==refreshBefore+1 and #calls==featureCalls)
refreshAccepted=false;events.onUpdate(.01)
check(not M.state.active and not calls[#calls][1])
print('permission refresh regression PASS')
-- onInit with a failing observer registration keeps the gate off (fails closed, the game's own reload stays).
do
 local ev2={}
 registerForEvent=function(n,f)ev2[n]=f end
 ObserveAfter=function()error('observer missing')end
 ObserveBefore=function()end
 local M2=dofile(UTPP_RELOAD_GATE_DIR..'/init.lua')
 check(not M2.enabled and not M2.ready);ev2.onInit();check(not M2.ready and not M2.enabled and not M2.SetEnabled(true))
end
print('reload gate onInit failure PASS')
-- onInit with the Game Animations pack in force (utpp_core PackIsGame): the gate stays off (the game's own reload), the pack switch
-- turns it on and off, a missing / failing core never raises and leaves the release default (enabled).
do
 local function boot(core)
  local ev={}
  registerForEvent=function(n,f)ev[n]=f end
  ObserveAfter=function()end
  ObserveBefore=function()end
  GetMod=core
  local M=dofile(UTPP_RELOAD_GATE_DIR..'/init.lua')
  ev.onInit()
  return M
 end
 local game=true
 local M4=boot(function(n) if n=='utpp_core' then return {PackIsGame=function() return game end} end end)
 check(M4.ready and not M4.enabled)
 check(M4.SetEnabled(true) and M4.enabled)
 check(not M4.SetEnabled(false) and not M4.enabled)
 game=false
 check(boot(function(n) if n=='utpp_core' then return {PackIsGame=function() return game end} end end).enabled)
 check(boot(nil).enabled)
 check(boot(function() error('no core') end).enabled)
 check(boot(function(n) if n=='utpp_core' then return {} end end).enabled)
 GetMod=nil
end
print('reload gate animation pack PASS')

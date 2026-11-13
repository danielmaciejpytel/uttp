-- Offline test of the world-position time series, tail sampler and core.diagnosticLogs auto start of utpp_reload_probe.
local events,hooks,logs={},{},{}
registerForEvent=function(n,f) events[n]=f end
Observe=function(c,n,f) assert(c=='ReloadEvents');hooks[n]=f end
local allLogs=false
GetMod=function(name)
  if name=='utpp_core' then return {Effective=function(k) assert(k=='core.diagnosticLogs');return allLogs and 'timed' or 'off' end} end
  return {Status=function() return 'desired=TPP reason=user' end}
end
Game={UTPPMotionGlueStatus=function() return 'glue enabled=true body=0 tail=xyz' end,
  UTPPLogLine=function(s) logs[#logs+1]=s end,NameToString=function(n) return n end}
TDBID={ToStringDEBUG=function(x)return x end};ItemID={GetTDBID=function(x)return x end}
WorldPosition={ToVector4=function(p)return p end}
WorldTransform={new=function() return {Position={x=0,y=0,z=0}} end}
local path=UTPP_RELOAD_PROBE_PATH
local M=dofile(path)
local n=0
local function check(x,msg) if not x then error('check '..(n+1)..(msg and (': '..msg) or '')) end;n=n+1 end
local function count(pat,from)
  local c=0
  for i=from or 1,#logs do if logs[i]:find(pat) then c=c+1 end end
  return c
end
local function lastWith(pat) for i=#logs,1,-1 do if logs[i]:find(pat) then return logs[i] end end end

-- fixtures -------------------------------------------------------------------------------------
local function comp(name,x,y,z)
  local c={name=name,worldTransform={Position={x=x,y=y,z=z},Orientation={i=0,j=0,k=0,r=1}}}
  c.GetName=function() return c.name end
  return c
end
local mag=comp('Magazine',1,2,3)
local magReload=comp('MagazineReload',1.5,2,3)
local recv=comp('Receiver',0,0,0)
local barrel=comp('Barrel',0,1,0)
local clip=comp('ExtraClip_01',9,9,9)
local other=comp('Shell',5,5,5)
local weaponPos={x=0,y=0,z=0}
local owner={GetComponents=function() return {mag,magReload,recv,barrel,clip,other} end,
  GetEntityID=function() return 'e1' end,GetItemID=function() return 'Items.Preset_Nue_Default' end,
  GetWorldPosition=function() return weaponPos end,
  GetWeaponRecord=function() return {ItemType=function() return {Name=function() return 'Wea_Handgun' end} end} end}
local slotXf={WeaponRight={Position={x=0,y=0,z=1}},WeaponLeft={Position={x=0,y=0,z=2}}}
local slotComp={slots={{slotName='WeaponRight',boneName='WeaponRight'},{slotName='Leg',boneName='Leg'},{slotName='WeaponLeft',boneName='WeaponLeft'}}}
function slotComp:GetSlotTransform(name,xf) local t=slotXf[name];if not t then return false,xf end;return true,{Position=t.Position} end
local playerPos={x=10,y=10,z=10}
local player={IsPlayer=function() return true end,GetComponents=function() return {slotComp,{}} end,GetWorldPosition=function() return playerPos end}
local iface={executionOwner=player,owner=owner}
local elapsed=0
local self={GetInStateTime=function() return elapsed end,m_animReloadData={emptyReload=false,loopDuration=3,continueLoop=false},m_animReloadSpeed={reloadSpeed=1,emptyReloadSpeed=1}}
local STEP=0.1001

-- ready / auto start --------------------------------------------------------------------------
events.onInit();check(M.ready and hooks.OnEnter and hooks.OnUpdate and hooks.OnExit and hooks.OnForcedExit and events.onUpdate)
M.ready=false;allLogs=true;events.onUpdate(1.5);check(not M.enabled,'not ready: no auto start');M.ready=true
allLogs=false;events.onUpdate(1.1);check(not M.enabled)
allLogs=true
events.onUpdate(0.5);check(not M.enabled,'poll about once per second')
events.onUpdate(0.6);check(M.enabled and M.auto and M.unlimited and lastWith('auto%-started by core.diagnosticLogs'))
events.onUpdate(1.1);check(M.enabled,'still on')
allLogs=false;events.onUpdate(1.1);check(not M.enabled and lastWith('auto%-stopped'))
check(count('auto%-started')==1)
-- an error/limit stop in auto mode is not restarted in a loop while the flag stays on
allLogs=true;events.onUpdate(1.1);check(M.enabled);M.Stop();events.onUpdate(1.1);check(not M.enabled,'latched')
allLogs=false;events.onUpdate(1.1);allLogs=true;events.onUpdate(1.1);check(M.enabled,'restarts after a new switch-on')
M.Stop();allLogs=false;events.onUpdate(1.1)
-- a manual session is not stopped by the flag
check(M.Start() and not M.auto and not M.unlimited);events.onUpdate(1.1);check(M.enabled);M.Stop()

-- full 3 s episode ----------------------------------------------------------------------------
allLogs=true -- auto session: the 1 s poll must keep it running while the option stays on
logs={};weaponPos={x=0,y=0,z=0}
M.Start({unlimited=true,auto=true})
elapsed=0;hooks.OnEnter(self,{},iface)
for i=1,30 do elapsed=i*STEP;hooks.OnUpdate(self,STEP,{},iface) end
local firstPos=nil
for i=1,#logs do if logs[i]:find('pos ep=1 t=0.00 ph=enter comp=Magazine ',1,true) then firstPos=logs[i] end end
check(firstPos,'enter pos line')
check(firstPos:find('xyz=1.0000,2.0000,3.0000 ijkr=0.00000,0.00000,0.00000,1.00000',1,true))
-- Magazine minus WeaponRight (0,0,1) = 1,2,2 |d|=3 ; minus WeaponLeft (0,0,2) = 1,2,1 ; minus weapon entity (0,0,0)
check(firstPos:find('dmag_wr=1.0000,2.0000,2.0000,3.0000',1,true),firstPos)
check(firstPos:find('dmag_wl=1.0000,2.0000,1.0000,2.4495',1,true))
check(firstPos:find('dmag_wpn=1.0000,2.0000,3.0000,3.7417',1,true))
check(count('comp=Magazine xyz=')>=31 and count('comp=MagazineReload ')>=31,'Magazine every sample')
check(count('what=slot:WeaponRight xyz=0.0000,0.0000,1.0000 dwpn_wr=0.0000,0.0000,%-1.0000,1.0000')>=31,'WeaponRight ref every sample')
-- secondary burst: samples 0 and 20 -> Receiver, Barrel, extra clip (name contains clip), no unrelated component
check(count('comp=Receiver ')==2 and count('comp=Barrel ')==2 and count('comp=ExtraClip_01 ')==2,'burst twice')
check(count('comp=Shell')==0,'unrelated component ignored')
check(count('what=weapon xyz=')==2 and count('what=player xyz=10.0000,10.0000,10.0000')==2 and count('what=slot:WeaponLeft xyz=0.0000,0.0000,2.0000')==2)
check(count('what=slot:Leg')==0,'only wanted slots')
check(count('refs%-method=cet%-slots')==1 and M.refsMethod=='cet-slots','method logged once')
check(count('^%[utpp_reload_phase_cet%] ctx ep=1 t=')>=15 and lastWith('ctx ep=1'):find('rec=Items.Preset_Nue_Default kind=Wea_Handgun glue=glue enabled=true',1,true))
-- rate: no 1 s window of state time carries more than 45 lines
local perSecond={}
for i=1,#logs do
  local t=logs[i]:match(' t=(%d+%.%d+)')
  if t and not logs[i]:find('routes',1,true) then local s=math.floor(tonumber(t)+0.0001);perSecond[s]=(perSecond[s] or 0)+1 end
end
for s,c in pairs(perSecond) do check(c<=48,'lines in second '..s..' = '..c) end
check(M.enabled and M.samples>=31,'unlimited: no stop')

-- magazine moves relative to weapon: values follow the re-read worldTransform every sample
logs={}
mag.worldTransform.Position={x=1,y=2,z=4};elapsed=elapsed+STEP;hooks.OnUpdate(self,STEP,{},iface)
check(lastWith('comp=Magazine '):find('xyz=1.0000,2.0000,4.0000',1,true) and lastWith('comp=Magazine '):find('dmag_wr=1.0000,2.0000,3.0000,3.7417',1,true))

-- exit + 0.7 s tail
logs={};elapsed=elapsed+STEP;hooks.OnExit(self,{},iface)
check(count('ph=exit comp=Magazine ')==1)
local exitT=elapsed
for i=1,60 do events.onUpdate(0.016) end -- 0.96 s of frames
check(count('ph=tail comp=Magazine ')==7,'7 tail samples: '..count('ph=tail comp=Magazine '))
check(count('tail%-end ep=1 samples=7')==1)
local lastTail=lastWith('ph=tail comp=Magazine ')
check(lastTail:find(string.format('t=%.2f',exitT+0.7),1,true),lastTail)
local before=#logs;for i=1,40 do events.onUpdate(0.016) end;check(#logs==before,'no samples after the tail')
-- a new enter cancels a pending tail
elapsed=0;hooks.OnEnter(self,{},iface);elapsed=STEP;hooks.OnExit(self,{},iface);events.onUpdate(0.05)
elapsed=0;hooks.OnEnter(self,{},iface);local c0=count('ph=tail');for i=1,20 do events.onUpdate(0.05) end;check(count('ph=tail')==c0 and count('ph=tail')<=0+count('ph=tail'))
allLogs=false
allLogs=false
M.Stop();local c1=#logs;events.onUpdate(0.5);check(#logs==c1,'stopped probe stays silent')

-- failure paths log their reason once and never throw ---------------------------------------------
logs={};M.Start()
mag.worldTransform=nil -- component unreadable
slotComp.GetSlotTransform=function() error('out param form unsupported') end
Game.UTPPReloadRefSlots=nil
elapsed=0;check(pcall(hooks.OnEnter,self,{},iface))
for i=1,5 do elapsed=i*STEP;check(pcall(hooks.OnUpdate,self,STEP,{},iface)) end
check(M.enabled,'failures do not stop the probe')
check(count('comp=Magazine xyz=unavailable reason=')==1,'component failure logged once: '..count('comp=Magazine xyz=unavailable'))
check(count('comp=MagazineReload xyz=')>=6 and count('xyz=unavailable')>=2)
check(count('refs%-method=none reason=')==1 and count('what=slot:WeaponRight xyz=unavailable reason=')==1)
-- script bridge fallback
logs={};M.Stop();M.Start()
Game.UTPPReloadRefSlots=function(p) assert(p==player);return 'WeaponRight;0,0,1|WeaponLeft;0,0,2|Head;0,0,3' end
mag.worldTransform={Position={x=1,y=2,z=3},Orientation={i=0,j=0,k=0,r=1}}
elapsed=0;hooks.OnEnter(self,{},iface)
check(count('refs%-method=script%-bridge')==1 and M.refsMethod=='script-bridge')
check(lastWith('comp=Magazine '):find('dmag_wr=1.0000,2.0000,2.0000,3.0000',1,true))
check(count('what=slot:Head xyz=0.0000,0.0000,3.0000')==1)
Game.UTPPReloadRefSlots=function() return 'unavailable:no-slots' end
M.Stop();M.Start();logs={};elapsed=0
check(pcall(hooks.OnEnter,self,{},iface) and count('refs%-method=none reason=.*bridge%-empty')==1)
Game.UTPPReloadRefSlots=function() error('not reflected') end
M.Stop();M.Start();logs={};elapsed=0
check(pcall(hooks.OnEnter,self,{},iface) and count('refs%-method=none reason=.*bridge:')==1)
-- missing entity positions / transform values
weaponPos=nil;playerPos=nil
owner.GetWorldPosition=function() error('gone') end
M.Stop();M.Start();logs={};elapsed=0
check(pcall(hooks.OnEnter,self,{},iface) and count('what=weapon xyz=unavailable reason=')==1 and count('what=player xyz=')>=0)
mag.worldTransform={Position={x=0/0,y=0,z=0},Orientation={i=0,j=0,k=0,r=1}}
check(pcall(hooks.OnUpdate,self,STEP,{},iface))
-- EmitPositions with no episode / broken owner
check(pcall(M.EmitPositions,'upd',0,0,nil,nil))
check(pcall(M.OnUpdate,nil) and pcall(M.OnUpdate,-1) and pcall(M.OnUpdate,0/0))
check(pcall(M.PollAllLogs))
GetMod=function() error('no mods') end
check(pcall(M.PollAllLogs))
M.Stop()

-- episode cap (manual) and its lifting (all-logs mode) -----------------------------------------
GetMod=function(name)
  if name=='utpp_core' then return {Effective=function() return allLogs and 'until' or 'off' end} end
  return {Status=function() return 'desired=TPP reason=user' end}
end
owner.GetWorldPosition=function() return weaponPos or {x=0,y=0,z=0} end
mag.worldTransform={Position={x=1,y=2,z=3},Orientation={i=0,j=0,k=0,r=1}}
M.epLimit=5;M.Start();logs={};elapsed=0
hooks.OnEnter(self,{},iface)
for i=1,12 do elapsed=i*STEP;hooks.OnUpdate(self,STEP,{},iface) end
check(count('ph=%a+ comp=Magazine ')==5 and count('episode cap reached ep=1 limit=5')==1,'cap: '..count('ph=%a+ comp=Magazine '))
check(M.enabled)
M.Stop();M.Start({unlimited=true,auto=true});logs={};elapsed=0
hooks.OnEnter(self,{},iface)
for i=1,12 do elapsed=i*STEP;hooks.OnUpdate(self,STEP,{},iface) end
check(count('ph=%a+ comp=Magazine ')==13 and count('episode cap')==0,'unlimited')
-- session cap still stops a manual probe
M.Stop();M.epLimit=50;M.limit=3;M.Start();logs={};elapsed=0
hooks.OnEnter(self,{},iface)
for i=1,6 do elapsed=i*STEP;hooks.OnUpdate(self,STEP,{},iface) end
check(not M.enabled and lastWith('STOP limit'))
-- routes: only the first routeEpisodes episodes unless unlimited
M.limit=1200;M.Stop();M.Start();logs={}
for ep=1,6 do elapsed=0;hooks.OnEnter(self,{},iface) end
check(count('routes configured%-only')==4)
M.Stop();M.Start({unlimited=true});logs={}
for ep=1,6 do elapsed=0;hooks.OnEnter(self,{},iface) end
check(count('routes configured%-only')==6)
M.Stop()
print('Reload probe positions: '..n..' passed')

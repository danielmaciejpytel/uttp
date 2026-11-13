local events,hooks,logs={},{},{}
registerForEvent=function(n,f) events[n]=f end
Observe=function(c,n,f) assert(c=='ReloadEvents');hooks[n]=f end
local tpp=true
GetMod=function() return {Status=function() return tpp and 'desired=TPP reason=user' or 'desired=FPP reason=user' end} end
Game={UTPPMotionGlueStatus=function() return 'glue enabled=true body=0' end,UTPPLogLine=function(s) logs[#logs+1]=s end}
TDBID={ToStringDEBUG=function(x)return x end};ItemID={GetTDBID=function(x)return x end}
local M=dofile(UTPP_RELOAD_PROBE_PATH)
local n=0
local function check(x) assert(x,'check '..(n+1));n=n+1 end
check(not M.Start());events.onInit();check(M.ready and hooks.OnUpdate and hooks.OnEnter)
check(M.Start() and M.enabled)
local player={IsPlayer=function()return true end}
local iface={executionOwner=player,owner={GetEntityID=function()return 'entity10'end,GetItemID=function()return 'Items.Nue'end}}
local elapsed=0
local self={GetInStateTime=function()return elapsed end,m_animReloadData={emptyReload=false,loopDuration=2.6,continueLoop=false},m_animReloadSpeed={reloadSpeed=1,emptyReloadSpeed=1}}
hooks.OnEnter(self,{},iface);check(M.episodes==1 and M.samples==1)
elapsed=.2;hooks.OnUpdate(self,.2,{},iface);check(M.samples==2)
elapsed=.21;hooks.OnUpdate(self,.01,{},iface);check(M.samples==2)
tpp=false;elapsed=.4;hooks.OnUpdate(self,.2,{},iface);check(M.samples==2);tpp=true
iface.executionOwner={IsPlayer=function()return false end};hooks.OnUpdate(self,.2,{},iface);check(M.samples==2);iface.executionOwner=player
elapsed=.5;hooks.OnExit(self,{},iface);check(M.samples==3)
self.GetInStateTime=function()error('not accessible')end;hooks.OnUpdate(self,.2,{},iface);check(not M.enabled and logs[#logs]:find('STOP unavailable',1,true))
self.GetInStateTime=function()return 1 end;M.limit=1;check(M.Start());hooks.OnEnter(self,{},iface);check(not M.enabled and logs[#logs]:find('STOP limit',1,true))
M.limit=160;M.Start();self.m_animReloadSpeed.reloadSpeed=0;hooks.OnEnter(self,{},iface);check(not M.enabled)
check(M.Stop() and not M.enabled)
M.Start();self.m_animReloadData=nil;self.m_animReloadSpeed=nil
hooks.OnEnter(self,{},iface);check(M.enabled and M.samples==1 and logs[#logs]:find('metadata=false',1,true))
elapsed=1.2;self.GetInStateTime=function()return elapsed end;hooks.OnUpdate(self,.2,{},iface)
check(M.enabled and M.samples==2 and logs[#logs]:find('speed=unavailable',1,true))
check(M.Stop() and not M.enabled)
local context={GetPermanentBoolParameter=function()return {valid=true,value=false} end,GetPermanentFloatParameter=function()return {valid=true,value=2} end}
iface.owner.GetWeaponRecord=function()return {BaseReloadTime=function()return 3 end,BaseEmptyReloadTime=function()return 4 end}end
M.Start();hooks.OnEnter(self,context,iface)
check(M.enabled and logs[#logs]:find('source=state-context',1,true) and logs[#logs]:find('speed=1.5',1,true))
context.GetPermanentBoolParameter=function()return {valid=true,value=true} end
local d,s=M.ContextMetadata(context,iface.owner);check(d.emptyReload and s.emptyReloadSpeed==2)
context.GetPermanentFloatParameter=function()return {valid=true,value=0} end
check(M.ContextMetadata(context,iface.owner)==nil)
context.GetPermanentFloatParameter=function()return {valid=false,value=2} end
check(M.ContextMetadata(context,iface.owner)==nil)
M.Stop()
local rows,available=M.ComponentRoutes({});check(not available and #rows==0)
ResourceRef={GetPath=function(ref)return ref end};ResourceAsyncRef=ResourceRef;ResRef={ToString=function(ref)return ref end}
local component={GetName=function()return 'MagazineReload'end,rig='rig\\nue',graph='graph\\weapon',animations={gameplay={{animSet='anims\\generic',priority=200}}}}
rows,available=M.ComponentRoutes({GetComponents=function()return {component,{GetName=function()error('missing')end}} end})
check(available and #rows==1 and rows[1].name=='MagazineReload' and rows[1].sets[1].priority==200)
check(rows[1].rig=='rig\\nue' and rows[1].sets[1].path=='anims\\generic')
component.animations=nil;component.rig=nil
rows,available=M.ComponentRoutes({GetComponents=function()return {component} end})
check(available and #rows==1 and #rows[1].sets==0 and rows[1].rig=='unavailable')
WorldPosition={ToVector4=function(p)return p end}
component.worldTransform={Position={x=1,y=2,z=3},Orientation={i=0,j=0,k=0,r=1}}
check(M.TransformSnapshot(component)=='1,2,3,0,0,0,1')
component.worldTransform.Orientation.r=2
check(M.TransformSnapshot(component)=='unavailable')
component.worldTransform.Position.x=0/0
check(M.TransformSnapshot(component)=='unavailable')
check(M.TransformSnapshot({})=='unavailable')
-- CET may provide a fresh userdata wrapper on every callback for one state.
-- Throttling must follow OnEnter/phase, never Lua wrapper identity.
M.limit=160;M.Start()
local function wrapped(t)
  return {GetInStateTime=function()return t end,
    m_animReloadData={emptyReload=false,loopDuration=2},m_animReloadSpeed={reloadSpeed=1}}
end
hooks.OnEnter(wrapped(0),{},iface);check(M.samples==1)
hooks.OnUpdate(wrapped(.2),.2,{},iface);check(M.samples==2)
hooks.OnUpdate(wrapped(.21),.01,{},iface);check(M.samples==2)
hooks.OnUpdate(wrapped(.31),.1,{},iface);check(M.samples==3)
hooks.OnExit(wrapped(.32),{},iface);check(M.samples==4)
M.Stop()
ResourceRef.GetHash=function()return '12345678901234567890ULL' end
local path,hash,reason=M.ResourceIdentity('known',false)
check(path=='known' and hash=='12345678901234567890ULL' and reason=='none')
ResourceRef.GetPath=function()error('getter unavailable\nsecond line')end
path,hash,reason=M.ResourceIdentity('known',false)
check(path=='unavailable' and hash=='12345678901234567890ULL' and reason:find('getter unavailable',1,true) and not reason:find('\n',1,true))
ResourceRef.GetPath=function()return ''end
path,hash,reason=M.ResourceIdentity('known',true)
check(path=='unavailable' and reason=='empty-or-nonstring-path')
ResourceRef.GetHash=function()error('missing')end
path,hash,reason=M.ResourceIdentity('known',true)
check(hash=='unavailable')
check(M.ScriptIdentity(component,0,0)=='unavailable:script-bridge')
Game.UTPPReloadResourceIdentity=function(c,kind,index)
  assert(c==component);return 'hash=123;path=fixture/'..kind..'/'..index
end
check(M.ScriptIdentity(component,2,3)=='hash=123;path=fixture/2/3')
Game.UTPPReloadResourceIdentity=function()error('missing')end
check(M.ScriptIdentity(component,0,0)=='unavailable:script-bridge')
check(M.BranchMetadata({})=='unavailable:branch-bridge')
Game.UTPPReloadBranchMetadata=function(state)assert(state==self);return 'amountToReload=10;empty=false;continueLoop=false'end
check(M.BranchMetadata(self)=='amountToReload=10;empty=false;continueLoop=false')
Game.UTPPReloadBranchMetadata=function()error('not reflected')end
check(M.BranchMetadata(self)=='unavailable:branch-bridge')
print('Reload CET phase probe: '..n..' passed')

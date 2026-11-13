-- Passive opt-in phase observer. Never overrides a method or changes poses.
--
-- Records (all lines prefixed '[utpp_reload_phase_cet] '):
--   * the original route/phase lines (routes at ENTER, branch-inputs, stage=...)
--   * a WORLD-SPACE POSITION TIME SERIES while a ReloadEvents episode runs and 0.7 s after it ends:
--       pos ep=N t=0.30 ph=upd comp=Magazine xyz=x,y,z ijkr=i,j,k,r dmag_wr=dx,dy,dz,|d| dmag_wl=... dmag_wpn=...
--       ref ep=N t=0.30 ph=upd what=slot:WeaponRight xyz=x,y,z dwpn_wr=dx,dy,dz,|d|
--       ref ep=N t=0.30 ph=upd what=weapon|player|slot:<name> xyz=x,y,z      (secondary refs, one burst per 2 s)
--       ctx ep=N t=0.30 rec=Items.X kind=pistol|rifle|... glue=<tail of UTPPMotionGlueStatus>   (every 0.2 s)
--   * 'refs-method=...' once per session: which way the hand/weapon slot world positions were obtained.
--
-- Everything is pcall-wrapped and read-only. ~40 lines/s budget: per 0.1 s sample the primary lines are
-- pos Magazine, pos MagazineReload, ref WeaponRight (3) + stage line every 2nd sample + branch line every
-- 5th; the remaining components/refs go out in one burst per 2 s (every 20th sample).
--
-- HOW THE PROBE STARTS
--   M.ready becomes true in the CET 'onInit' event once Observe('ReloadEvents', OnEnter|OnUpdate|OnExit|
--   OnForcedExit) succeeded for all four methods (the class and the four protected methods exist in the 2.5
--   script dump: ReloadEvents extends WeaponEventsTransition, OnUpdate(timeDelta, stateContext,
--   scriptInterface), the others (stateContext, scriptInterface)). M.Start() refuses while not ready.
--   Auto mode: the 'onUpdate' event polls utpp_core.Effective('core.diagnosticLogs') about once per second and
--   starts the probe (unlimited sample caps) while the list is not 'off' and stops it when it turns off.
local M = { enabled=false, ready=false, samples=0, episodes=0, limit=1200, epLimit=50,
  stageEvery=2, branchEvery=5, burstEvery=20, routeEpisodes=4, unlimited=false, auto=false,
  tailSeconds=0.7, autoPollSeconds=1.0, errors=0, errorLimit=1, refsMethod=nil }
local nextSample = 0
local E = nil      -- per-episode cache/counters
local tail = nil   -- post-exit sampler state
local pollAcc, autoLatched, refsLogged = 0, false, false
local WANT_SLOTS = { WeaponRight=true, WeaponLeft=true, LeftHand=true, RightHand=true, Head=true, TppHead=true }
local SLOT_ORDER = { 'WeaponRight', 'WeaponLeft', 'LeftHand', 'RightHand', 'Head', 'TppHead' }

local function log(line)
  pcall(function() Game.UTPPLogLine('[utpp_reload_phase_cet] '..line) end)
end
local function short(e) return (tostring(e):gsub('[\r\n]',' '):sub(1,120)) end
local function fin(v) return type(v)=='number' and v==v and math.abs(v)~=math.huge end

function M.Stop()
  M.enabled=false; tail=nil; E=nil; return true
end
function M.Start(opts)
  if not M.ready then return false end
  M.samples=0; M.episodes=0; M.errors=0; nextSample=0; E=nil; tail=nil; refsLogged=false; M.refsMethod=nil
  M.unlimited=opts and opts.unlimited==true or false
  M.auto=opts and opts.auto==true or false
  M.errorLimit=M.auto and 20 or 1 -- auto mode survives transient failures (e.g. no player yet)
  M.enabled=true
  log('START'); return true
end
function M.ContextMetadata(context, owner)
  -- Same inputs as retail RefreshReloadPermanentFloats/GetReloadAnimSpeed.
  -- Access remains optional until confirmed on a live state context.
  local ok, data, speed=pcall(function()
    local emptyResult=context:GetPermanentBoolParameter('EmptyReload')
    local durationResult=context:GetPermanentFloatParameter('ReloadLogicalDuration')
    if not emptyResult or emptyResult.valid~=true or not durationResult or durationResult.valid~=true then error('invalid context result') end
    local empty,duration=emptyResult.value,durationResult.value
    if type(empty)~='boolean' or type(duration)~='number' or duration~=duration or duration<=0 or duration==math.huge then error('unavailable context metadata') end
    local record=owner:GetWeaponRecord()
    local base=empty and record:BaseEmptyReloadTime() or record:BaseReloadTime()
    if type(base)~='number' or base~=base or base==math.huge then error('unavailable base reload time') end
    local rate=base>0 and base/duration or 1 -- exact native fallback for nonpositive base
    return {emptyReload=empty,loopDuration=duration,emptyDuration=duration},
      {reloadSpeed=rate,emptyReloadSpeed=rate}
  end)
  if ok then return data,speed end
  return nil,nil
end
-- Read-only component configuration, not proof of the selected graph animation.
-- Missing reflection/Codeware APIs must not stop native phase observations.
function M.TransformSnapshot(component)
  local ok,value=pcall(function()
    local t=component.worldTransform
    local p=WorldPosition.ToVector4(t.Position)
    local q=t.Orientation
    local values={p.x,p.y,p.z,q.i,q.j,q.k,q.r}
    for _,v in ipairs(values) do
      if type(v)~='number' or v~=v or math.abs(v)==math.huge then error('invalid transform') end
    end
    if #values~=7 then error('incomplete transform') end
    local norm=q.i*q.i+q.j*q.j+q.k*q.k+q.r*q.r
    if math.abs(norm-1)>.001 then error('invalid orientation') end
    return string.format('%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g',unpack(values))
  end)
  return ok and value or 'unavailable'
end
function M.ResourceIdentity(ref,async)
  local path,hash,reason='unavailable','unavailable','none'
  local ok,value=pcall(function()
    local p=async and ResourceAsyncRef.GetPath(ref) or ResourceRef.GetPath(ref)
    return ResRef.ToString(p)
  end)
  if ok and type(value)=='string' and value~='' then path=value
  else reason=(ok and 'empty-or-nonstring-path' or tostring(value)):gsub('[\r\n]',' '):sub(1,160) end
  -- Keep identity separate from a resolved textual path. A hash never proves
  -- which configured animation is currently selected by the graph.
  local hashOk,h=pcall(function()
    return async and ResourceAsyncRef.GetHash(ref) or ResourceRef.GetHash(ref)
  end)
  if hashOk and h~=nil then hash=tostring(h) end
  return path,hash,reason
end
function M.ScriptIdentity(component,kind,index)
  local ok,value=pcall(function() return Game.UTPPReloadResourceIdentity(component,kind,index) end)
  return ok and type(value)=='string' and value or 'unavailable:script-bridge'
end
function M.BranchMetadata(state)
  local ok,value=pcall(function() return Game.UTPPReloadBranchMetadata(state) end)
  return ok and type(value)=='string' and value or 'unavailable:branch-bridge'
end
function M.ComponentRoutes(owner)
  local rows={}
  local ok=pcall(function()
    for _,component in ipairs(owner:GetComponents() or {}) do
      local good=pcall(function()
        local raw=component:GetName()
        local name=type(raw)=='string' and raw or Game.NameToString(raw)
        if name~='Magazine' and name~='MagazineReload' and name~='Barrel' and name~='Receiver' then return end
        if #rows>=8 then return end
        local function path(ref,async)
          local value,hash,reason=M.ResourceIdentity(ref,async)
          return value,hash,reason
        end
        local row={name=name,rig='unavailable',graph='unavailable',sets={},world=M.TransformSnapshot(component),
          rigIdentity=M.ScriptIdentity(component,0,0),graphIdentity=M.ScriptIdentity(component,1,0)}
        pcall(function() row.rig,row.rigHash,row.rigError=path(component.rig,false) end)
        pcall(function() row.graph,row.graphHash,row.graphError=path(component.graph,false) end)
        pcall(function()
          for i,entry in ipairs(component.animations.gameplay or {}) do
            if i>16 then break end
            local value,hash,reason=path(entry.animSet,true)
            row.sets[#row.sets+1]={path=value,hash=hash,error=reason,priority=entry.priority,identity=M.ScriptIdentity(component,2,i-1)}
          end
        end)
        rows[#rows+1]=row
      end)
    end
  end)
  return rows,ok
end

---------------------------------------------------------------------------------------------------
-- World position time series
---------------------------------------------------------------------------------------------------
local function compName(component)
  local raw=component:GetName()
  return type(raw)=='string' and raw or Game.NameToString(raw)
end
-- components of the weapon entity that matter for the magazine/weapon placement
local function wantedComponent(name)
  if name=='Magazine' or name=='MagazineReload' or name=='Receiver' or name=='Barrel' then return true end
  local l=tostring(name):lower()
  return l:find('agazine',1,true)~=nil or l:find('clip',1,true)~=nil
end
function M.WeaponComponents(owner)
  local rows={}
  pcall(function()
    for _,component in ipairs(owner:GetComponents() or {}) do
      pcall(function()
        local name=compName(component)
        if #rows<16 and wantedComponent(name) then rows[#rows+1]={name=name,comp=component} end
      end)
    end
  end)
  return rows
end
-- position+orientation of a component (worldTransform is re-read on every call); nil,reason on failure
function M.ReadTransform(component)
  local ok,v=pcall(function()
    local t=component.worldTransform
    local p=WorldPosition.ToVector4(t.Position)
    local q=t.Orientation
    local r={x=p.x,y=p.y,z=p.z,i=q.i,j=q.j,k=q.k,r=q.r}
    for _,k in ipairs({'x','y','z','i','j','k','r'}) do if not fin(r[k]) then error('non-finite '..k) end end
    return r
  end)
  if ok then return v end
  return nil,short(v)
end
function M.ReadEntityPos(entity)
  local ok,v=pcall(function()
    local p=entity:GetWorldPosition()
    local r={x=p.x,y=p.y,z=p.z}
    for _,k in ipairs({'x','y','z'}) do if not fin(r[k]) then error('non-finite '..k) end end
    return r
  end)
  if ok then return v end
  return nil,short(v)
end
-- Method 1: the player's SlotComponent(s) directly from CET. SlotComponent.slots[i].slotName/boneName and
-- GetSlotTransform(slotName, out WorldTransform) exist in the 2.5 dump (also used by utpp_mount_probe.reds).
-- The out-parameter calling form differs between CET versions, so both the returned and the mutated
-- WorldTransform are accepted.
local function slotCacheCet(player)
  local list={}
  pcall(function()
    for _,component in ipairs(player:GetComponents() or {}) do
      pcall(function()
        local slots=component.slots
        if slots==nil then return end
        for _,s in ipairs(slots) do
          pcall(function()
            local sn,bn=s.slotName,s.boneName
            sn=type(sn)=='string' and sn or Game.NameToString(sn)
            bn=type(bn)=='string' and bn or Game.NameToString(bn)
            local key=WANT_SLOTS[sn] and sn or (WANT_SLOTS[bn] and bn or nil)
            if key and #list<12 then list[#list+1]={key=key,comp=component,slot=s.slotName} end
          end)
        end
      end)
    end
  end)
  return list
end
local function readSlotsCet(cache)
  local out,reason={},nil
  for _,e in ipairs(cache) do
    if not out[e.key] then
      local ok,v=pcall(function()
        local xf=WorldTransform.new()
        local r1,r2=e.comp:GetSlotTransform(e.slot,xf)
        if r1==false then error('slot not found') end
        local t=r2~=nil and r2 or xf
        local p=WorldPosition.ToVector4(t.Position)
        local r={x=p.x,y=p.y,z=p.z}
        for _,k in ipairs({'x','y','z'}) do if not fin(r[k]) then error('non-finite') end end
        return r
      end)
      if ok then out[e.key]=v else reason=short(v) end
    end
  end
  return out,reason
end
-- Method 2: scripts\utpp_reload_probe.reds UTPPReloadRefSlots(player) -> "key;x,y,z|key;x,y,z|..."
local function readSlotsBridge(player)
  local ok,s=pcall(function() return Game.UTPPReloadRefSlots(player) end)
  if not ok then return nil,'bridge:'..short(s) end
  if type(s)~='string' then return nil,'bridge-not-string' end
  local out,n={},0
  for key,x,y,z in s:gmatch('([%w_]+);([%-%d%.eE+]+),([%-%d%.eE+]+),([%-%d%.eE+]+)') do
    local r={x=tonumber(x),y=tonumber(y),z=tonumber(z)}
    if fin(r.x) and fin(r.y) and fin(r.z) and not out[key] then out[key]=r; n=n+1 end
  end
  if n==0 then return nil,'bridge-empty:'..s:sub(1,80) end
  return out
end
function M.ReadSlots(player)
  if not E then return {} ,'no-episode' end
  local reasons={}
  if E.slotCache==nil then E.slotCache=slotCacheCet(player) end
  if #E.slotCache>0 then
    local out,r=readSlotsCet(E.slotCache)
    if next(out) then M.NoteRefsMethod('cet-slots'); return out end
    reasons[#reasons+1]='cet-slots:'..tostring(r)
  else
    reasons[#reasons+1]='cet-slots:no-slot-component-or-slots'
  end
  local out,r=readSlotsBridge(player)
  if out then M.NoteRefsMethod('script-bridge'); return out end
  reasons[#reasons+1]=r
  M.NoteRefsMethod(nil,table.concat(reasons,' / '))
  return {},table.concat(reasons,' / ')
end
function M.NoteRefsMethod(method,why)
  if refsLogged then return end
  if method then refsLogged=true; M.refsMethod=method; log('refs-method='..method) return end
  if not E or E.refsFail then return end
  E.refsFail=true; log('refs-method=none reason='..tostring(why)) -- retried next episode
end

local function xyzs(p) return string.format('%.4f,%.4f,%.4f',p.x,p.y,p.z) end
local function ijkrs(q) return string.format('%.5f,%.5f,%.5f,%.5f',q.i,q.j,q.k,q.r) end
local function delta(a,b)
  if not (a and b) then return 'n/a' end
  local dx,dy,dz=a.x-b.x,a.y-b.y,a.z-b.z
  return string.format('%.4f,%.4f,%.4f,%.4f',dx,dy,dz,math.sqrt(dx*dx+dy*dy+dz*dz))
end
local function shortName(name)
  if name=='Magazine' then return 'mag' end
  if name=='MagazineReload' then return 'magreload' end
  return (tostring(name):lower():gsub('[^%w]',''))
end
-- failures are logged once per episode and key so a broken API does not flood the log
local function failOnce(key,line)
  if not E then return end
  E.failed=E.failed or {}
  if E.failed[key] then return end
  E.failed[key]=true; log(line)
end

function M.WeaponKind(owner,record)
  local kind
  -- ItemType_Record.Name() -> CName exists in the 2.5 dump (e.g. Wea_Rifle/Wea_Handgun); the exact value is
  -- unverified without the game, hence the name-based fallback below.
  pcall(function()
    local raw=owner:GetWeaponRecord():ItemType():Name()
    kind=type(raw)=='string' and raw or Game.NameToString(raw)
  end)
  if type(kind)=='string' and kind~='' and kind~='None' then return kind end
  local r=tostring(record):lower()
  if r:find('rifle',1,true) or r:find('ar_',1,true) or r:find('assault',1,true) then return 'name-guess:rifle' end
  if r:find('pistol',1,true) or r:find('nue',1,true) or r:find('unity',1,true) then return 'name-guess:pistol' end
  return 'unknown'
end
function M.EmitContext(ep,t,ph,owner)
  local record='unavailable'
  pcall(function() record=tostring(TDBID.ToStringDEBUG(ItemID.GetTDBID(owner:GetItemID()))) end)
  local glue='unavailable'
  pcall(function() glue=tostring(Game.UTPPMotionGlueStatus()):gsub('[\r\n]',' '); glue=glue:sub(-80) end)
  log(string.format('ctx ep=%d t=%.2f ph=%s rec=%s kind=%s glue=%s',ep,t,ph,record,M.WeaponKind(owner,record),glue))
end
-- One position sample: reads everything, emits the primary lines (+ the burst when due).
-- n = 0-based sample index inside the episode; t = state time in seconds; ph = enter|upd|exit|forced-exit|tail
function M.EmitPositions(ph,t,n,owner,player)
  if not E then return end
  local ep=E.ep
  local ok,err=pcall(function()
    if E.comps==nil or (#E.comps==0 and n%10==0) then E.comps=M.WeaponComponents(owner) end
    local comps,reads={},{}
    for _,row in ipairs(E.comps) do
      local tr,why=M.ReadTransform(row.comp)
      if tr then reads[#reads+1]={name=row.name,tr=tr} else failOnce('c:'..row.name,string.format('pos ep=%d t=%.2f ph=%s comp=%s xyz=unavailable reason=%s',ep,t,ph,row.name,tostring(why))) end
      comps[row.name]=tr
    end
    local wpn,wwhy=M.ReadEntityPos(owner)
    local ply,pwhy=M.ReadEntityPos(player)
    if not wpn then failOnce('wpn',string.format('ref ep=%d t=%.2f ph=%s what=weapon xyz=unavailable reason=%s',ep,t,ph,tostring(wwhy))) end
    if not ply then failOnce('ply',string.format('ref ep=%d t=%.2f ph=%s what=player xyz=unavailable reason=%s',ep,t,ph,tostring(pwhy))) end
    local slots,swhy=M.ReadSlots(player)
    local wr,wl=slots.WeaponRight,slots.WeaponLeft
    if not wr then failOnce('slot:WeaponRight',string.format('ref ep=%d t=%.2f ph=%s what=slot:WeaponRight xyz=unavailable reason=%s',ep,t,ph,tostring(swhy or 'slot-not-found'))) end
    local function posLine(r)
      local sn=shortName(r.name)
      log(string.format('pos ep=%d t=%.2f ph=%s comp=%s xyz=%s ijkr=%s d%s_wr=%s d%s_wl=%s d%s_wpn=%s',ep,t,ph,r.name,
        xyzs(r.tr),ijkrs(r.tr),sn,delta(r.tr,wr),sn,delta(r.tr,wl),sn,delta(r.tr,wpn)))
    end
    -- primary lines every sample
    for _,r in ipairs(reads) do if r.name=='Magazine' or r.name=='MagazineReload' then posLine(r) end end
    if wr then
      log(string.format('ref ep=%d t=%.2f ph=%s what=slot:WeaponRight xyz=%s dwpn_wr=%s',ep,t,ph,xyzs(wr),delta(wpn,wr)))
    end
    -- secondary burst (first sample of the episode and then every M.burstEvery samples)
    if n%M.burstEvery==0 then
      for _,r in ipairs(reads) do if not (r.name=='Magazine' or r.name=='MagazineReload') then posLine(r) end end
      if wpn then log(string.format('ref ep=%d t=%.2f ph=%s what=weapon xyz=%s',ep,t,ph,xyzs(wpn))) end
      if ply then log(string.format('ref ep=%d t=%.2f ph=%s what=player xyz=%s',ep,t,ph,xyzs(ply))) end
      for _,key in ipairs(SLOT_ORDER) do
        if key~='WeaponRight' and slots[key] then
          log(string.format('ref ep=%d t=%.2f ph=%s what=slot:%s xyz=%s',ep,t,ph,key,xyzs(slots[key])))
        end
      end
    end
  end)
  if not ok then failOnce('emit','pos ep='..ep..' sample-error='..short(err)) end
end

function M.Sample(stage, self, iface, context)
  if not M.enabled then return end
  local ok, err=pcall(function()
    if not iface or not iface.executionOwner or not iface.executionOwner:IsPlayer() then return end
    local perspective=GetMod('utpp_perspective')
    if not perspective or not perspective.Status():match('desired=TPP%f[%W]') then return end
    if not tostring(Game.UTPPMotionGlueStatus()):match('enabled=true%f[%W]') then return end
    -- CET wrapper identity is not native state identity. Only OnEnter resets
    -- the episode throttle; a new userdata wrapper must not bypass the cap.
    local elapsed=self:GetInStateTime()
    if type(elapsed)~='number' or elapsed~=elapsed or elapsed<0 or elapsed==math.huge then error('invalid native state time') end
    if stage=='enter' then
      M.episodes=M.episodes+1; nextSample=0; tail=nil
      E={ep=M.episodes,n=0}
      if M.unlimited or M.episodes<=M.routeEpisodes then
        local rows,available=M.ComponentRoutes(iface.owner)
        log('routes configured-only episode='..M.episodes..' available='..tostring(available)..' count='..#rows)
        for _,row in ipairs(rows) do
          log('component='..row.name..' rig='..row.rig..' graph='..row.graph)
          log('resource-identity component='..row.name..' rigHash='..tostring(row.rigHash)..' graphHash='..tostring(row.graphHash)..' rigError='..tostring(row.rigError)..' graphError='..tostring(row.graphError))
          log('script-resource-identity component='..row.name..' rig='..row.rigIdentity..' graph='..row.graphIdentity)
          log('component-world enter-only not-synchronized component='..row.name..' xyz-ijkr='..row.world)
          for _,entry in ipairs(row.sets) do
            log('configured-set component='..row.name..' priority='..tostring(entry.priority)..' path='..entry.path..' hash='..entry.hash..' pathError='..entry.error..' scriptIdentity='..entry.identity)
          end
        end
      end
    end
    if not E then E={ep=math.max(M.episodes,1),n=0} end -- probe started in the middle of an episode
    if stage=='sample' then
      if elapsed<nextSample then return end
      nextSample=elapsed+0.1
    end
    -- per-episode cap (lifted in all-logs mode); the session cap below still applies unless unlimited
    if not M.unlimited and E.n>=M.epLimit then
      if not E.capLogged then E.capLogged=true; log('episode cap reached ep='..E.ep..' limit='..M.epLimit) end
      return
    end
    local n=E.n; E.n=n+1
    local first=n==0
    -- positions first and in their own pcall so a metadata failure below never hides them
    M.EmitPositions(stage=='sample' and 'upd' or stage,elapsed,n,iface.owner,iface.executionOwner)
    local data, speed=self.m_animReloadData, self.m_animReloadSpeed
    local metadataSource='script-fields'
    if data==nil or speed==nil then
      data,speed=M.ContextMetadata(context,iface.owner);metadataSource='state-context'
    end
    -- Retail CET can expose GetInStateTime while hiding script class fields.
    -- Preserve phase evidence but NEVER invent speed/empty/duration for playback.
    local metadata=data~=nil and speed~=nil
    local rate=metadata and (data.emptyReload and speed.emptyReloadSpeed or speed.reloadSpeed) or nil
    local duration=metadata and (data.emptyReload and data.emptyDuration or data.loopDuration) or nil
    if metadata and (type(rate)~='number' or rate~=rate or rate<=0 or rate==math.huge) then error('invalid native speed') end
    M.samples=M.samples+1
    local record=TDBID.ToStringDEBUG(ItemID.GetTDBID(iface.owner:GetItemID()))
    local always=stage~='sample' or not metadata
    if always or n%M.stageEvery==0 then M.EmitContext(E.ep,elapsed,stage=='sample' and 'upd' or stage,iface.owner) end
    if always or first or n%M.branchEvery==0 then
      log('branch-inputs episode='..M.episodes..' state='..tostring(elapsed)..' '..M.BranchMetadata(self))
    end
    if always or n%M.stageEvery==0 then
      log(string.format('stage=%s episode=%d entity=%s record=%s state=%.6f metadata=%s source=%s speed=%s logical=%s empty=%s loop=%s',
        stage,M.episodes,tostring(iface.owner:GetEntityID()),tostring(record),elapsed,tostring(metadata),metadata and metadataSource or 'unavailable',
        metadata and tostring(rate) or 'unavailable',metadata and tostring(duration) or 'unavailable',
        metadata and tostring(data.emptyReload) or 'unavailable',metadata and data.continueLoop~=nil and tostring(data.continueLoop) or 'unavailable'))
    end
    M.errors=0
    if not M.unlimited and M.samples>=M.limit then M.Stop(); log('STOP limit'); return end
    -- after the state ends keep sampling positions for M.tailSeconds from the CET onUpdate event
    if stage=='exit' or stage=='forced-exit' then
      tail={owner=iface.owner,player=iface.executionOwner,t0=elapsed,acc=0,k=0,ep=E.ep}
    end
  end)
  if not ok then
    M.errors=M.errors+1
    if M.errors>=M.errorLimit then M.Stop(); log('STOP unavailable: '..tostring(err)) end
  end
end

-- CET 'onUpdate' (delta seconds): tail sampler after OnExit + about-once-per-second core.diagnosticLogs poll.
function M.OnUpdate(dt)
  if type(dt)~='number' or dt~=dt or dt<0 or dt>5 then dt=0 end
  if tail and M.enabled and E then
    tail.acc=tail.acc+dt
    local steps=0
    while tail and tail.acc>=0.1 and steps<8 do
      tail.acc=tail.acc-0.1; tail.k=tail.k+1; steps=steps+1
      local k=tail.k
      M.EmitPositions('tail',tail.t0+0.1*k,k,tail.owner,tail.player)
      if k*0.1>=M.tailSeconds-1e-6 then
        log('tail-end ep='..tail.ep..' samples='..k)
        tail=nil
      end
    end
  end
  pollAcc=pollAcc+dt
  if pollAcc>=M.autoPollSeconds then
    pollAcc=0
    M.PollAllLogs()
  end
end
function M.PollAllLogs()
  local flag=false
  pcall(function()
    local core=GetMod('utpp_core')
    local mode=core~=nil and core.Effective('core.diagnosticLogs') or nil
    flag=mode=='timed' or mode=='until'
  end)
  if flag then
    -- start once per switch-on edge: a limit/error stop must not restart in a loop
    if not M.enabled and not autoLatched and M.ready then
      if M.Start({unlimited=true,auto=true}) then autoLatched=true; log('auto-started by core.diagnosticLogs') end
    end
  else
    autoLatched=false
    if M.enabled and M.auto then M.Stop(); M.auto=false; log('auto-stopped') end
  end
end

if registerForEvent then
  registerForEvent('onInit',function()
    local success=true
    for _, spec in ipairs({{'OnEnter','enter'},{'OnUpdate','sample'},{'OnExit','exit'},{'OnForcedExit','forced-exit'}}) do
      local stage=spec[2]
      local ok=pcall(function()
        Observe('ReloadEvents',spec[1],function(self, a, b, c)
          M.Sample(stage,self,stage=='sample' and c or b,stage=='sample' and b or a)
        end)
      end)
      success=success and ok
    end
    M.ready=success; log('registered='..tostring(success))
  end)
  registerForEvent('onUpdate',function(dt) pcall(M.OnUpdate,dt) end)
  registerForEvent('onShutdown',M.Stop)
end
return M

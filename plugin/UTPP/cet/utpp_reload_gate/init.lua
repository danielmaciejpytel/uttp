-- Authored Nue reload (female V, Items.Preset_Nue_Default, third person). The body + magazine + graph bundle ships with
-- the mod, so the bundle flags default to ready and onInit enables the gate once its observers are installed; every
-- single episode is still decided by policy.lua (fails closed to the game's own reload).
local Policy=require('policy')
local M={enabled=false,assetsReady=true,normalAssetsReady=true,twoHandAssetsReady=true,ready=false,state=Policy.New(),pendingClear=false,age=0,exitTail=0,clipPhase=0,clipSpeed=0}
-- MotionDriver returns native upper ownership over 0.20s. Keep the authored
-- graph branch through that fade, with 0.05s allowance for the callback/feed edge.
local EXIT_TAIL_SECONDS=0.25
-- Both authored EMPTY magazine clips contain 100 intervals at 30 fps. The
-- retail weapon graph leaves Reload on clip-end (reload_end), not EndReload.
-- Switching back before that event activates the native leaf at time zero.
local EMPTY_CLIP_SECONDS=100/30
local NORMAL_CLIP_SECONDS=80/30
-- Bounded read-only diagnostics: distinguish no callback from rejected input.
M.observed={};M.lastSnapshot=nil;M.lastReadError=nil
-- One line per reload entry / stop (2.5 diagnosis): which eligibility input rejected the episode.
-- no file system paths in the log: a Lua error text names the script file with its full path (user name included); a path in
-- the CET mods folder is cut to "mods\...", the game folder prefix is dropped, a user profile path becomes "~" (same rules as
-- utpp_core M.ScrubPaths)
local function scrubPaths(s)
  s = tostring(s)
  if not string.find(s, "%a:[\\/]") then return s end
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-([Mm]ods[\\/])", "%1")
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-Cyberpunk 2077[\\/]", "")
  s = string.gsub(s, "%a:[\\/][Uu]sers[\\/][^\\/%s\"']+", "~")
  return (string.gsub(s, "%a:[\\/][^%s\"']*", "<path>"))
end
local function log(line) pcall(function() Game.UTPPLogLine(scrubPaths('[utpp_reload_gate] '..line)) end) end
local function why(o)
  if type(o)~='table' then return 'snapshot=nil' end
  local r={}
  local function chk(ok,name) if not ok then r[#r+1]=name end end
  chk(o.enabled==true,'enabled');chk(o.tpp==true,'tpp');chk(o.female==true,'female');chk(o.player==true,'player')
  chk(o.weapon=='Items.Preset_Nue_Default','weapon='..tostring(o.weapon))
  chk(o.empty==true or (o.empty==false and o.normalAssetsReady==true),'empty/normalAssets')
  chk(o.action==true,'action(loop)');chk(o.assetsReady==true,'assets');chk(type(o.aim)=='boolean','aim')
  chk(o.grip==2 or (o.grip==3 and o.twoHandAssetsReady==true),'grip='..tostring(o.grip))
  chk(type(o.speed)=='number' and o.speed>0,'speed='..tostring(o.speed))
  chk(type(o.owner)=='string' and o.owner~='','owner');chk(type(o.session)=='string' and o.session~='','session='..tostring(o.session))
  if #r==0 then return 'ok' end
  return table.concat(r,',')
end
M.why=why

local function identity(iface)
  local owner=iface.owner:GetEntityID().hash
  local playerID=iface.executionOwner:GetEntityID().hash
  if not owner or not playerID then return nil end
  return tostring(playerID)..'/'..tostring(owner)
end
local function snapshot(state,iface,latchedAim)
  local player=iface.executionOwner
  if not player or not player:IsPlayer() then return nil end
  local perspective=GetMod('utpp_perspective')
  local decision=perspective.Desired()
  local native=Game.UTPPNueReloadNativeInputs(state)
  local empty=native:match('^empty=([^;]+);')
  local loop=native:match(';loop=([^;]+);')
  local speed=tonumber(native:match(';speed=([^;]+)$'))
  if (empty~='true' and empty~='false') or (loop~='true' and loop~='false') then return nil end
  local owner=identity(iface)
  if not owner then return nil end
  local session=perspective.Status():match('session=([^%s]+)')
  local aim=perspective.IsAiming()
  local gripAim=aim
  if type(latchedAim)=='boolean' then gripAim=latchedAim end
  return {enabled=M.enabled,assetsReady=M.assetsReady,normalAssetsReady=M.normalAssetsReady,twoHandAssetsReady=M.twoHandAssetsReady,player=true,
    female=NameToString(player:GetResolvedGenderName())=='Female',tpp=decision.perspective=='TPP',
    weapon=TDBID.ToStringDEBUG(ItemID.GetTDBID(iface.owner:GetItemID())),empty=empty=='true',
    action=loop=='false',aim=aim,grip=Game.UTPPNueReloadGrip(gripAim),elapsed=state:GetInStateTime(),speed=speed,
    owner=owner,session=session}
end
-- Callback interfaces are borrowed; never retain them for CET onUpdate.
local function context()
  local player=Game.GetPlayer()
  if not player then return nil end
  local weapon=Game.GetTransactionSystem():GetItemInSlot(player,TweakDBID.new('AttachmentSlots.WeaponRight'))
  if not weapon then return nil end
  local perspective=GetMod('utpp_perspective')
  return {tpp=perspective.Desired().perspective=='TPP',
    grip=Game.UTPPNueReloadGrip(M.state.aim),
    female=NameToString(player:GetResolvedGenderName())=='Female',
    weapon=TDBID.ToStringDEBUG(ItemID.GetTDBID(weapon:GetItemID())),
    owner=tostring(player:GetEntityID().hash)..'/'..tostring(weapon:GetEntityID().hash),
    session=perspective.Status():match('session=([^%s]+)')}
end
M.api={refresh=function(aim,twoHand) return Game.UTPPRefreshNueReloadPermission(aim,twoHand) end,snapshot=snapshot,context=context,owner=identity,publish=function(on,aim,twoHand) return Game.UTPPPublishNueReloadGate(on,aim,twoHand) end}
local published,publishedAim,publishedTwoHand=false,false,false
local function send(on,aim)
  local twoHand=on and M.state.grip==3
  if not on and not published and not M.pendingClear then return true end
  if on==published and aim==publishedAim and twoHand==publishedTwoHand and not M.pendingClear then return true end
  local ok,result=pcall(M.api.publish,on,aim,twoHand)
  if ok and result==true then
    if on~=published then log(string.format('publish on=%s aim=%s twoHand=%s',tostring(on),tostring(aim),tostring(twoHand))) end
    published,publishedAim,publishedTwoHand=on,aim,twoHand;M.pendingClear=false;return true
  end
  log(string.format('publish FAILED on=%s ok=%s result=%s',tostring(on),tostring(ok),tostring(result)))
  -- A failed call may already have applied part of a feature pair. Retry OFF.
  M.pendingClear=true;Policy.Clear(M.state);return false
end
function M.Stop()
  Policy.Clear(M.state);M.age=0;M.exitTail=0;M.clipPhase=0;M.clipSpeed=0
  return send(false,false)
end
function M.SetEnabled(on)
  local cleared=M.Stop();M.enabled=on==true and M.ready and cleared;return M.enabled
end
function M.SetBundleReady(on)
  M.Stop();M.assetsReady=on==true;M.normalAssetsReady=false -- every new bundle needs a separate NORMAL opt-in
  M.twoHandAssetsReady=false
end
function M.SetNormalBundleReady(on)
  M.Stop();M.normalAssetsReady=on==true -- separate opt-in; old EMPTY packages remain compatible
end
function M.SetTwoHandBundleReady(on)
  M.Stop();M.twoHandAssetsReady=on==true -- old 1H bundles cannot select missing 2H leaves
end
function M.Handle(stage,state,iface)
  M.observed[stage]=(M.observed[stage] or 0)+1
  if stage=='exit' or stage=='forced-exit' then
    -- Ignore NPC reload exits: their state machines run alongside the player.
    local ok,isPlayer=pcall(function()return iface.executionOwner:IsPlayer()end)
    if ok and isPlayer then
      local identified,owner=pcall(M.api.owner,iface)
      if M.pendingClear then M.Stop()
      elseif identified and owner~=nil and owner==M.state.owner then
        if stage=='exit' and M.state.active then
          -- Native OnExit sends EndReload; the graph still contributes while
          -- fading out. Clearing the switch here reintroduces the FPP clip.
          if M.exitTail==0 then
            local duration=M.state.empty and EMPTY_CLIP_SECONDS or NORMAL_CLIP_SECONDS
            local remaining=math.max(0,duration-M.clipPhase)/M.clipSpeed
            if remaining~=remaining or remaining==math.huge then M.Stop();return end
            M.exitTail=remaining+EXIT_TAIL_SECONDS
          end
        else M.Stop() end
      end
    end
    return
  end
  if not M.enabled or not M.assetsReady then return end
  if stage=='sample' and M.exitTail>0 then return end -- do not reopen/extend a finished episode
  local ok,isPlayer=pcall(function()return iface.executionOwner:IsPlayer()end)
  if not ok or not isPlayer then return end
  local latchedAim=nil
  if stage=='sample' and M.state.active then latchedAim=M.state.aim end
  local read,o=pcall(M.api.snapshot,state,iface,latchedAim)
  if not read then M.lastReadError=tostring(o);M.lastSnapshot=nil;if stage=='enter' then log('enter snapshot error '..tostring(o)) end;M.Stop();return end
  M.lastReadError=nil;M.lastSnapshot=o
  local on,aim
  local previousElapsed=M.state.last
  if stage=='enter' then
    if not M.Stop() then return end;on,aim=Policy.Enter(M.state,o)
    log(string.format('enter on=%s aim=%s grip=%s empty=%s speed=%s why=%s',tostring(on),tostring(aim),tostring(o and o.grip),
      tostring(o and o.empty),tostring(o and o.speed),why(o)))
  else
    local was=M.state.active
    on,aim=Policy.Update(M.state,o)
    if was and not on then log('update dropped why='..why(o)) end
  end
  if on then
    if stage=='enter' then M.clipSpeed=o.speed
    else
      -- Callback elapsed is game time. Use the lower endpoint speed so a speed
      -- change cannot overestimate completion; never retain borrowed objects.
      M.clipPhase=M.clipPhase+(o.elapsed-previousElapsed)*math.min(M.clipSpeed,o.speed)
      M.clipSpeed=o.speed
    end
  end
  M.age=0
  if not send(on,aim) then M.Stop() end
end
function M.Tick(dt)
  if M.pendingClear then send(false,false) end
  if not M.state.active then return end
  if type(dt)~='number' or dt~=dt or dt<0 or dt==math.huge then M.Stop();return end
  if M.exitTail>0 then
    M.exitTail=M.exitTail-dt
    if M.exitTail<=0 then M.Stop();return end
  else
    M.age=M.age+dt
    if M.age>.25 then log('tick stop: no native update for 0.25 s');M.Stop();return end -- no native player update: fail closed
  end
  local ok,o=pcall(M.api.context)
  if not ok then log('tick stop: context error '..tostring(o));M.Stop();return end
  if type(o)~='table' or not M.enabled or not M.assetsReady
    or (M.state.empty==false and not M.normalAssetsReady)
    or (M.state.grip==3 and not M.twoHandAssetsReady) or o.grip~=M.state.grip
    or o.tpp~=true or o.female~=true or o.weapon~='Items.Preset_Nue_Default'
    or o.owner~=M.state.owner or o.session~=M.state.session then
    log(string.format('tick stop: context grip=%s/%s tpp=%s weapon=%s owner=%s session=%s/%s',tostring(type(o)=='table' and o.grip),tostring(M.state.grip),
      tostring(type(o)=='table' and o.tpp),tostring(type(o)=='table' and o.weapon),tostring(type(o)=='table' and o.owner==M.state.owner),
      tostring(type(o)=='table' and o.session),tostring(M.state.session)))
    M.Stop();return end
  -- Native remap/ownership resets may clear permission while graph stays active.
  -- Revalidate the published owner without resubmitting/restarting its features.
  local refreshed,accepted=pcall(M.api.refresh,M.state.aim,M.state.grip==3)
  if not refreshed or accepted~=true then log('tick stop: refresh '..tostring(refreshed)..'/'..tostring(accepted));M.Stop();return end
  -- Native callbacks alone advance elapsed and validate reload data.
end
if registerForEvent then
  registerForEvent('onInit',function()
    local success=true
    for _,spec in ipairs({{'OnEnter','enter'},{'OnUpdate','sample'},{'OnExit','exit'},{'OnForcedExit','forced-exit'}}) do
      local stage=spec[2]
      -- Native OnEnter initializes emptyReload. Observe is BEFORE and sees stale data.
      -- Clear exits before teardown, but consume native entry/update outputs AFTER.
      local observe=(stage=='enter' or stage=='sample') and ObserveAfter or ObserveBefore
      local ok=pcall(function()observe('ReloadEvents',spec[1],function(self,a,b,c)
        M.Handle(stage,self,stage=='sample' and c or b)
      end)end)
      success=success and ok
    end
    M.ready=success
    log('init observers='..tostring(success))
    -- the Game Animations pack (utpp_core) keeps the game's own reload; core switches the gate again when the pack changes
    local okC,core=pcall(function() return GetMod('utpp_core') end)
    local gamePack=okC and type(core)=='table' and type(core.PackIsGame)=='function' and core.PackIsGame()==true
    if success then M.SetEnabled(not gamePack) else M.enabled=false;M.Stop() end
  end)
  registerForEvent('onUpdate',M.Tick)
  registerForEvent('onShutdown',function()M.enabled=false;M.Stop()end)
end
return M

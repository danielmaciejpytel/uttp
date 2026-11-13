local P=dofile(UTPP_RELOAD_GATE_DIR..'/policy.lua')
local count=0
local function check(x) assert(x,'check '..(count+1));count=count+1 end
local function sample()
 return {enabled=true,tpp=true,female=true,player=true,weapon='Items.Preset_Nue_Default',empty=true,action=true,
 assetsReady=true,aim=true,grip=2,elapsed=0,speed=1.37,owner='player/weapon1',session='session1'}
end
local s=P.New();check(not s.active)
local o=sample();local on,aim=P.Enter(s,o);check(on and aim)
o.elapsed=.4;o.aim=false;on,aim=P.Update(s,o);check(on and aim) -- Aim latched at entry.
o.tpp=false;check(not P.Update(s,o));o.tpp=true;check(not P.Update(s,o)) -- No late reentry.
check(P.Enter(s,sample()));check(not P.Clear(s));check(not P.Update(s,sample()))
for _,key in ipairs({'enabled','tpp','female','player','empty','action','assetsReady'}) do
 o=sample();o[key]=false;check(not P.Enter(s,o));o=sample();check(P.Enter(s,o));o[key]=nil;check(not P.Update(s,o))
end
for _,pair in ipairs({{'weapon','Items.Other'},{'owner',''},{'session',''},{'aim','yes'},{'elapsed',-1},{'speed',0},{'speed',math.huge},{'elapsed',0/0}}) do
 o=sample();o[pair[1]]=pair[2];check(not P.Enter(s,o))
end
for _,key in ipairs({'owner','session'}) do
 o=sample();check(P.Enter(s,o));o[key]='other';check(not P.Update(s,o));o=sample();check(not P.Update(s,o))
end
o=sample();o.elapsed=.5;check(P.Enter(s,o));o.elapsed=.1;check(not P.Update(s,o))
o=sample();o.aim=false;on,aim=P.Enter(s,o);check(on and not aim);o.aim=true;o.elapsed=.2;on,aim=P.Update(s,o);check(on and not aim)
check(not P.Enter(s,nil));check(not P.Update(s,nil))
o=sample();o.empty=false;check(not P.Enter(s,o))
o.normalAssetsReady=true;check(P.Enter(s,o) and s.empty==false)
o.elapsed=.1;o.aim=false;on,aim=P.Update(s,o);check(on and aim)
o.empty=true;check(not P.Update(s,o));check(not P.Update(s,o))
o=sample();o.normalAssetsReady=true;check(P.Enter(s,o));o.empty=false;check(not P.Update(s,o))
o=sample();o.empty=false;o.normalAssetsReady=true;check(P.Enter(s,o));o.normalAssetsReady=false;check(not P.Update(s,o))
o=sample();o.normalAssetsReady=true;o.empty=nil;check(not P.Enter(s,o))
o.empty='false';check(not P.Enter(s,o))
o=sample();o.grip=3;check(not P.Enter(s,o))
o.twoHandAssetsReady=true;check(P.Enter(s,o) and s.grip==3)
o.aim=false;o.elapsed=.1;check(P.Update(s,o) and s.aim) -- latch Aim, not ADS changes
o.grip=2;check(not P.Update(s,o));check(not P.Update(s,o))
o=sample();o.grip=nil;check(not P.Enter(s,o));o.grip=1;check(not P.Enter(s,o))
o=sample();o.grip=3;o.twoHandAssetsReady=true;check(P.Enter(s,o));o.twoHandAssetsReady=false;check(not P.Update(s,o))
print('reload gate policy '..count..'/'..count..' PASS')

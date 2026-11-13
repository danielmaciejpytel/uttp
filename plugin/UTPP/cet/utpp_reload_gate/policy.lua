-- Decision policy only; engine publisher/asset registration are separate.
-- No late entry: cancellation remains latched until a new native OnEnter.
local P = {}
local function finite(v) return type(v)=='number' and v==v and v>=0 and v<math.huge end
local function eligible(o)
  return type(o)=='table' and o.enabled==true and o.tpp==true and o.female==true
    and o.player==true and o.weapon=='Items.Preset_Nue_Default'
    and (o.empty==true or (o.empty==false and o.normalAssetsReady==true))
    and o.action==true and o.assetsReady==true and type(o.aim)=='boolean'
    and (o.grip==2 or (o.grip==3 and o.twoHandAssetsReady==true))
    and finite(o.elapsed) and finite(o.speed) and o.speed>0
    and type(o.owner)=='string' and o.owner~='' and type(o.session)=='string' and o.session~=''
end
function P.New() return {active=false,aim=false,grip=nil,empty=nil,owner=nil,session=nil,last=0} end
function P.Clear(s)
  s.active=false; s.aim=false; s.grip=nil; s.empty=nil; s.owner=nil; s.session=nil; s.last=0
  return false,false
end
function P.Enter(s,o)
  P.Clear(s)
  if not eligible(o) then return false,false end
  s.active=true; s.aim=o.aim; s.grip=o.grip; s.empty=o.empty; s.owner=o.owner; s.session=o.session; s.last=o.elapsed
  return true,s.aim
end
function P.Update(s,o)
  if not s.active then return false,false end
  if not eligible(o) or o.grip~=s.grip or o.empty~=s.empty or o.owner~=s.owner or o.session~=s.session or o.elapsed<s.last then
    return P.Clear(s)
  end
  s.last=o.elapsed
  return true,s.aim
end
return P

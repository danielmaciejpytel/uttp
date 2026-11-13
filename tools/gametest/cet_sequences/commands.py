"""Single-line CET console commands for the Male-deformation A/B (scratch generator, offline only)."""

# no double quotes, no '--', no ';;', no '$' and no backticks inside any command (see gt_input.auto_marker_ok / --cet split)

PRE = ("local p=Game.GetPlayer() local es=Game.GetScriptableSystemsContainer():Get('EquipmentSystem') "
       "local pd=es:GetPlayerData(p) ")
NAME_FN = ("local function n(id) if id~=nil and ItemID.IsValid(id) then return TDBID.ToStringDEBUG(ItemID.GetTDBID(id)) end "
           "return 'none' end ")


def echo(tag):
    return " Game.UTPPLogLine('[night] " + tag + " ok='..tostring(ok)..' '..tostring(e))"


def report_weapon(tag="EQUIP_W"):
    return ("local ok,e=pcall(function() " + PRE + NAME_FN +
            "local W=Enum.new('gamedataEquipmentArea','Weapon') local s={} "
            "for i=0,pd:GetNumberOfSlots(W,true)-1 do s[#s+1]=i..'='..n(pd:GetItemInEquipSlot(W,i)) end "
            "local hand='none' local it=Game.GetTransactionSystem():GetItemInSlot(p,TweakDBID.new('AttachmentSlots.WeaponRight')) "
            "if it~=nil then hand=n(it:GetItemID()) end "
            "local okN,nd=pcall(function() return n(pd:GetLastUsedOrFirstAvailableRangedWeapon()) end) "
            "return 'active='..n(es:GetActiveItem(p,W))..' slots['..table.concat(s,',')..'] hand='..hand..' nextDraw='..tostring(nd) end)"
            + echo(tag))


def report_clothes(tag="EQUIP_C"):
    return ("local ok,e=pcall(function() " + PRE + NAME_FN +
            "local o={} for _,a in ipairs({'OuterChest','InnerChest','Outfit'}) do "
            "local A=Enum.new('gamedataEquipmentArea',a) local v='?' pcall(function() v=n(es:GetActiveVisualItem(p,A)) end) "
            "o[#o+1]=a..'='..n(es:GetActiveItem(p,A))..'(vis '..v..')' end "
            "for _,s in ipairs({'Torso','Chest'}) do local it=Game.GetTransactionSystem():GetItemInSlot(p,TweakDBID.new('AttachmentSlots.'..s)) "
            "o[#o+1]='slot'..s..'='..(it~=nil and n(it:GetItemID()) or 'none') end return table.concat(o,' ') end)"
            + echo(tag))


FIND_SLOT = ("local A=Enum.new('gamedataEquipmentArea','OuterChest') local si=-1 local id=nil "
             "for i=0,pd:GetNumberOfSlots(A,true)-1 do local x=es:GetItemInEquipSlot(p,A,i) "
             "if ItemID.IsValid(x) then id=x si=i break end end ")


def saratoga_equip(tag="SARATOGA"):
    return ("local ok,e=pcall(function() UTPPEXP=UTPPEXP or {} " + PRE +
            "local W=Enum.new('gamedataEquipmentArea','Weapon') local id=ItemID.FromTDBID(TweakDBID.new('Items.Preset_Saratoga_Maelstrom')) "
            "local cur=pd:GetActiveItem(W) if UTPPEXP.w==nil and ItemID.IsValid(cur) and "
            "TDBID.ToStringDEBUG(ItemID.GetTDBID(cur))~='Items.Preset_Saratoga_Maelstrom' then UTPPEXP.w=cur "
            "UTPPEXP.wn=TDBID.ToStringDEBUG(ItemID.GetTDBID(cur)) end "
            "local r=GameplayEquipRequest.new() r.owner=p r.itemID=id r.slotIndex=-1 r.addToInventory=true "
            "r.blockUpdateWeaponActiveSlots=true r.forceEquipWeapon=true es:QueueRequest(r) "
            "return 'queued, original='..tostring(UTPPEXP.wn) end)" + echo(tag))


def saratoga_equip_direct(tag="SARATOGA_DIRECT"):
    return ("local ok,e=pcall(function() UTPPEXP=UTPPEXP or {} " + PRE +
            "local W=Enum.new('gamedataEquipmentArea','Weapon') local ts=Game.GetTransactionSystem() "
            "local id=ItemID.FromTDBID(TweakDBID.new('Items.Preset_Saratoga_Maelstrom')) local cur=pd:GetActiveItem(W) "
            "if UTPPEXP.w==nil and ItemID.IsValid(cur) and TDBID.ToStringDEBUG(ItemID.GetTDBID(cur))~='Items.Preset_Saratoga_Maelstrom' "
            "then UTPPEXP.w=cur UTPPEXP.wn=TDBID.ToStringDEBUG(ItemID.GetTDBID(cur)) end "
            "if not ts:HasItem(p,id) then ts:GiveItem(p,id,1) end pd:EquipItem(id,true,true) "
            "return 'direct equipped='..tostring(pd:IsEquipped(id))..' original='..tostring(UTPPEXP.wn) end)" + echo(tag))


def holster(tag="HOLSTER"):
    # identical body to gt_night.FISTS_OFF_CMD (the only equip request proven on 2.5), other tag
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local r=EquipmentSystemWeaponManipulationRequest.new() "
            "r.requestType=Enum.new('EquipmentManipulationAction','UnequipWeapon') r.owner=p "
            "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'eqsystem' end)" + echo(tag))


def weapon_restore(tag="WEAPON_RESTORE"):
    return ("local ok,e=pcall(function() " + PRE +
            "if UTPPEXP==nil or UTPPEXP.w==nil then error('nothing saved') end local r=GameplayEquipRequest.new() r.owner=p "
            "r.itemID=UTPPEXP.w r.slotIndex=-1 r.addToInventory=false r.blockUpdateWeaponActiveSlots=true r.forceEquipWeapon=true "
            "es:QueueRequest(r) return 'queued '..tostring(UTPPEXP.wn) end)" + echo(tag))


def vest_off(tag="VEST_OFF"):
    return ("local ok,e=pcall(function() UTPPEXP=UTPPEXP or {} " + PRE + FIND_SLOT +
            "if id==nil then if UTPPEXP.c~=nil then return 'already off, saved '..UTPPEXP.cn end error('no item in OuterChest') end "
            "UTPPEXP.c=id UTPPEXP.cn=TDBID.ToStringDEBUG(ItemID.GetTDBID(id)) local r=UnequipRequest.new() r.owner=p "
            "r.areaType=A r.slotIndex=si r.force=false es:QueueRequest(r) return 'queued slot '..si..' '..UTPPEXP.cn end)" + echo(tag))


def vest_on(record, tag="VEST_ON"):
    return ("local ok,e=pcall(function() UTPPEXP=UTPPEXP or {} " + PRE + FIND_SLOT +
            "if id~=nil and UTPPEXP.c==nil then UTPPEXP.c=id UTPPEXP.cn=TDBID.ToStringDEBUG(ItemID.GetTDBID(id)) end "
            "local r=GameplayEquipRequest.new() r.owner=p r.itemID=ItemID.FromTDBID(TweakDBID.new('" + record + "')) "
            "r.slotIndex=-1 r.addToInventory=true es:QueueRequest(r) return 'queued " + record + ", original='..tostring(UTPPEXP.cn) end)"
            + echo(tag))


def clothes_restore(tag="CLOTHES_RESTORE"):
    return ("local ok,e=pcall(function() " + PRE +
            "if UTPPEXP==nil or UTPPEXP.c==nil then error('nothing saved') end local r=GameplayEquipRequest.new() r.owner=p "
            "r.itemID=UTPPEXP.c r.slotIndex=-1 r.addToInventory=false es:QueueRequest(r) return 'queued '..tostring(UTPPEXP.cn) end)"
            + echo(tag))


def vest_find(tag="VEST_FIND"):
    return ("local ok,e=pcall(function() local hits={} for _,r in ipairs(TweakDB:GetRecords('gamedataClothing_Record')) do "
            "local a,b='','' pcall(function() a=Game.NameToString(r:AppearanceName()) b=Game.NameToString(r:EntityName()) end) "
            "if string.find(string.lower(a..' '..b),'tactic_collar',1,true) then "
            "hits[#hits+1]=TDBID.ToStringDEBUG(r:GetID())..'['..a..']' end end "
            "return string.sub(#hits..' '..table.concat(hits,' '),1,420) end)" + echo(tag))


def diag_on(tag="DIAG_ON"):
    return ("local ok,e=pcall(function() return GetMod('utpp_core').Set('core.diagnosticLogs','timed') end)" + echo(tag))


def diag_off(tag="DIAG_OFF"):
    return ("local ok,e=pcall(function() return GetMod('utpp_core').Set('core.diagnosticLogs','off') end)" + echo(tag))


ALL = {
    "EQUIP_W": report_weapon(), "EQUIP_C": report_clothes(),
    "EQUIP_W_AFTER": report_weapon("EQUIP_W_AFTER"), "EQUIP_C_AFTER": report_clothes("EQUIP_C_AFTER"),
    "SARATOGA": saratoga_equip(), "SARATOGA_DIRECT": saratoga_equip_direct(), "HOLSTER": holster(),
    "WEAPON_RESTORE": weapon_restore(), "VEST_OFF": vest_off(), "VEST_ON": vest_on("__VEST__"),
    "CLOTHES_RESTORE": clothes_restore(), "VEST_FIND": vest_find(), "DIAG_ON": diag_on(), "DIAG_OFF": diag_off(),
}

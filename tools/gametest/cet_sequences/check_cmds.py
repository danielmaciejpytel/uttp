"""Offline check of the commands: Lua 5.1 syntax (luaL_loadstring) + a mocked run of the logic. Does not start the game.
Usage: python -I check_cmds.py <commands.py dir> <txt to scan or ''>
"""
import ctypes
import importlib.util
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from utpp_paths import project_root  # noqa: E402

ROOT = project_root()
dll = ctypes.CDLL(os.path.join(ROOT, "Tools", "lua-5.1.5", "lua51.dll"))
dll.luaL_newstate.restype = ctypes.c_void_p
dll.luaL_openlibs.argtypes = [ctypes.c_void_p]
dll.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
dll.luaL_loadstring.restype = ctypes.c_int
dll.lua_pcall.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
dll.lua_pcall.restype = ctypes.c_int
dll.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
dll.lua_tolstring.restype = ctypes.c_char_p
dll.lua_settop.argtypes = [ctypes.c_void_p, ctypes.c_int]
L = dll.luaL_newstate()
dll.luaL_openlibs(L)


def run(src, expect_ok=True):
    src_b = src.encode("utf-8")
    rc = dll.luaL_loadstring(L, src_b)
    if rc != 0:
        msg = dll.lua_tolstring(L, -1, None)
        dll.lua_settop(L, -2)
        return False, "SYNTAX: " + msg.decode()
    rc = dll.lua_pcall(L, 0, 0, 0)
    if rc != 0:
        msg = dll.lua_tolstring(L, -1, None)
        dll.lua_settop(L, -2)
        return False, "RUNTIME: " + msg.decode()
    return True, ""


def getlog():
    run("LOGTXT=table.concat(LOG,'\\n') LOG={}")
    dll.luaL_loadstring(L, b"return LOGTXT")
    dll.lua_pcall(L, 0, 1, 0)
    s = dll.lua_tolstring(L, -1, None)
    dll.lua_settop(L, -2)
    return s.decode() if s else ""


MOCK = r"""
LOG={} REQ={}
Game={}
function Game.UTPPLogLine(s) LOG[#LOG+1]=s end
function Game.NameToString(n) return tostring(n) end
Enum={new=function(t,v) return {t=t,v=v} end}
TweakDBID={new=function(s) return {s=s} end}
TDBID={ToStringDEBUG=function(t) return t.s end}
ItemID={IsValid=function(id) return id~=nil and id.rec~=nil end,
  GetTDBID=function(id) return {s=id.rec} end, FromTDBID=function(t) return {rec=t.s} end}
STATE={weapons={{rec='Items.Preset_Nue_Default'},nil,nil}, activeW=1, outer={{rec='Items.Vest_FAKE_01'}}, hand=nil}
local pd={}
function pd:GetNumberOfSlots(A,b) if A.v=='Weapon' then return 3 end return 1 end
function pd:GetItemInEquipSlot(A,i) if A.v=='Weapon' then return STATE.weapons[i+1] end if A.v=='OuterChest' then return STATE.outer[i+1] end return nil end
function pd:GetActiveItem(A) if A.v=='Weapon' then return STATE.weapons[STATE.activeW] end return STATE.outer[1] end
function pd:GetLastUsedOrFirstAvailableRangedWeapon() return STATE.weapons[STATE.activeW] end
function pd:EquipItem(id,b,f) STATE.weapons[2]=id end
function pd:IsEquipped(id) return true end
local es={}
function es:GetPlayerData(p) return pd end
function es:GetActiveItem(p,A) return pd:GetActiveItem(A) end
function es:GetItemInEquipSlot(p,A,i) return pd:GetItemInEquipSlot(A,i) end
function es:GetActiveVisualItem(p,A) return pd:GetActiveItem(A) end
function es:QueueRequest(r) REQ[#REQ+1]=r end
local ts={}
function ts:GetItemInSlot(p,slot) if slot.s=='AttachmentSlots.WeaponRight' then if STATE.hand then return {GetItemID=function() return STATE.hand end} end return nil end
  if slot.s=='AttachmentSlots.Torso' then return {GetItemID=function() return STATE.outer[1] end} end return nil end
function ts:HasItem(p,id) return false end
function ts:GiveItem(p,id,n) return true end
function Game.GetPlayer() return {} end
function Game.GetScriptableSystemsContainer() return {Get=function(self,n) assert(n=='EquipmentSystem') return es end} end
function Game.GetTransactionSystem() return ts end
GameplayEquipRequest={new=function() return {kind='equip'} end}
UnequipRequest={new=function() return {kind='unequip'} end}
EquipmentSystemWeaponManipulationRequest={new=function() return {kind='manip'} end}
TweakDB={GetRecords=function(self,t) assert(t=='gamedataClothing_Record')
  return {{AppearanceName=function() return 't2_072_pma_vest__tactic_collar' end, EntityName=function() return 'x' end, GetID=function() return {s='Items.Vest_XX'} end},
          {AppearanceName=function() return 'other' end, EntityName=function() return 'y' end, GetID=function() return {s='Items.Other'} end}} end}
function GetMod(n) return {Set=function(k,v) SETCALL=k..'='..v return v end} end
"""


def load_cmds(d):
    spec = importlib.util.spec_from_file_location("commands", os.path.join(d, "commands.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    d = sys.argv[1]
    m = load_cmds(d)
    fails = 0
    # 1. syntax of every command and of the same strings found in the output text
    all_cmds = dict(m.ALL)
    all_cmds["VEST_ON_REAL"] = m.vest_on("Items.Vest_FAKE_02")
    for k, c in all_cmds.items():
        ok, msg = run(c.replace("Game.UTPPLogLine", "Game.UTPPLogLine"), True) if False else (None, None)
        rc = dll.luaL_loadstring(L, c.encode())
        if rc != 0:
            fails += 1
            print("SYNTAX FAIL", k, dll.lua_tolstring(L, -1, None))
            dll.lua_settop(L, -2)
        else:
            dll.lua_settop(L, -2)
        bad = [x for x in ("\n", ";;", "--", "$", "`", '"') if x in c]
        print(f"{k:16s} len={len(c):4d} syntax={'ok' if rc == 0 else 'FAIL'} forbidden={bad}")
        if bad:
            fails += 1
    # 2. mocked logic run
    ok, msg = run(MOCK)
    if not ok:
        print("MOCK LOAD FAIL", msg)
        return 1
    for k in ("EQUIP_W", "EQUIP_C", "SARATOGA", "HOLSTER", "SARATOGA_DIRECT", "WEAPON_RESTORE", "VEST_OFF", "VEST_OFF", "CLOTHES_RESTORE",
              "VEST_ON_REAL", "VEST_FIND", "DIAG_ON", "DIAG_OFF"):
        ok, msg = run(all_cmds[k])
        print("RUN", k, "pcall-level:", "ok" if ok else msg)
        print("   log:", getlog())
    run("local s='' for i,r in ipairs(REQ) do s=s..i..':'..r.kind..' ' for _,f in ipairs({'slotIndex','addToInventory','blockUpdateWeaponActiveSlots','forceEquipWeapon','force'}) do if r[f]~=nil then s=s..f..'='..tostring(r[f])..' ' end end end LOG[#LOG+1]=s")
    print("REQUESTS:", getlog())
    run("LOG[#LOG+1]=tostring(SETCALL)")
    print("SET call:", getlog())
    # 3. every command in the output text must equal one generated here
    if len(sys.argv) > 2 and sys.argv[2]:
        text = open(sys.argv[2], encoding="utf-8").read()
        lines = [ln[len("LUA> "):] for ln in text.splitlines() if ln.startswith("LUA> ")]
        known = set(all_cmds.values())
        miss = [ln[:60] for ln in lines if ln not in known]
        print("LUA> lines in file:", len(lines), "not generated by commands.py:", len(miss), miss[:3])
        for ln in lines:
            rc = dll.luaL_loadstring(L, ln.encode())
            dll.lua_settop(L, -2)
            if rc != 0:
                fails += 1
                print("SYNTAX FAIL in file line", ln[:60])
    print("FAILS:", fails)
    return 1 if fails else 0


sys.exit(main())

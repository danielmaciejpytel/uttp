"""Pure helpers for the unattended night runner (night.py): no input, no processes, no file writes here.

- scenario sets (`--set`) and their validation against scenarios.json;
- CET command texts (game quit, vehicle spawn, position probe, marker) with the script evidence they rely on;
- ffmpeg command building (gdigrab window / desktop rect / ddagrab, h264_nvenc or libx264) and stderr parsing;
- recording clock model: video start wall time, video_sheets --video-end / --clock-offset and the residual error;
- quit fallback decision (CET ExitGame -> taskkill -> taskkill /F);
- gameplay detection in the UTPP log, vehicle approach stick, night summary metrics on parsed UTPP log lines.
"""
from __future__ import annotations

import json
import math
import os
import re
from datetime import datetime, timedelta

# ---------------------------------------------------------------------------------------------- scenario sets

LOCO_SET = ['slow_walk_half_stick', 'run_arc_left', 'run_arc_right', 'slalom_stick', 'flick_180_walk',
            'flick_180_run', 'flick_180_sprint', 'camera_whip_180', 'crouch_walk_turns', 'sprint_start_stop',
            'idle_then_start_sideways']
VEHICLE_PARTS = ['veh_call', 'veh_enter', 'veh_drive', 'veh_exit']
WEAPON_PARTS = ['weapon_draw', 'weapon_aim', 'weapon_holster', 'weapon_walk']
MELEE_PARTS = ['melee_draw', 'melee_punch', 'melee_punch_walk', 'melee_block', 'melee_holster']
ENEMY_PARTS = ['melee_draw', 'melee_punch', 'melee_block', 'melee_holster']   # fist fight on a real target
RECOVERY_PARTS = ['wheel_probe']
# executed by night.py (CET + pad + log waits); parts are plain pad scenarios in scenarios.json
AIM_PARTS = ['aim_probe_hold', 'weapon_draw']
MACROS = {'vehicle_mount_exit': VEHICLE_PARTS + RECOVERY_PARTS, 'weapon_draw_aim_holster': WEAPON_PARTS + RECOVERY_PARTS,
          'melee_fists': MELEE_PARTS, 'melee_fists_target': ENEMY_PARTS, 'aim_probe': AIM_PARTS, 'shoulder_key': []}
SETS = {
    'loco': LOCO_SET,
    'sprint_dash': ['sprint_dash'],
    'weapon_draw_aim_holster': ['weapon_draw_aim_holster'],
    'melee_fists': ['melee_fists'],
    'melee_fists_target': ['melee_fists_target'],   # a real opponent (Codeware NPC); not in 'all'
    'shoulder_key': ['shoulder_key'],   # keyboard L shoulder swap (not in 'all')
    'aim_probe': ['aim_probe'],      # not in 'all' (needs the utpp_probe AimProbe)
    'vehicle_mount_exit': ['vehicle_mount_exit'],
    'crouch_turns': ['crouch_walk_turns'],
    # keyboard-and-mouse-like input: digital stick edges, key taps, camera turned while a key is held (not in 'all')
    'kbd': ['kbd_taps', 'kbd_switch_90', 'kbd_switch_180', 'kbd_diagonals', 'kbd_strafe_weave', 'kbd_mouse_turns'],
}
# The car scenario runs FIRST: both test saves start at (or in) the car, so V is next to it. At the end of the set
# the walking scenarios had taken V 50-80 m away and the summoned car parked wherever the game chose (2026-11-12
# 13:09: 80 m away, V stuck on the way, time limit). night.py then walks V away from the car (leave_car).
SETS['all'] = ['vehicle_mount_exit'] + SETS['loco'] + ['sprint_dash', 'weapon_draw_aim_holster', 'melee_fists']


def resolve_sets(spec: str) -> list[str]:
    """'loco,sprint_dash' -> ordered unique scenario/macro names. Unknown set names may be scenario names."""
    out: list[str] = []
    for part in [p.strip() for p in spec.split(',') if p.strip()]:
        for n in SETS.get(part, [part]):
            if n not in out:
                out.append(n)
    return out


def validate_sets(names: list[str], scenarios: dict) -> list[str]:
    """Problems (empty = OK): every name must be a scenario or a macro whose parts exist."""
    errs = []
    for n in names:
        if n in MACROS:
            errs += [f'{n}: macro part {p!r} missing in scenarios.json' for p in MACROS[n] if p not in scenarios]
        elif n not in scenarios:
            errs.append(f'unknown scenario/set {n!r}')
    return errs


# ---------------------------------------------------------------------------------------------- CET commands
# Evidence (Extracted\game\scripts\vanilla-2.31):
#   inkMenuScenario.GetSystemRequestsHandler() -> wref<inkISystemRequestsHandler>   orphans.script:45472
#   inkISystemRequestsHandler.ExitGame()                                             orphans.script:45222
#   CET usage of GetSingleton('inkMenuScenario'):GetSystemRequestsHandler()  RedHotTools\libs\GameSession.lua:127
#   VehicleSystem.SpawnActivePlayerVehicle(opt vehicleType) -> Bool          core\systems\vehicleSystem.script:6
#   GameInstance.GetVehicleSystem                                            orphans.script:11447
#   (game's own call: quickSlotsManager.script:870)
#   VehicleSummonDataDef.SummonedVehicleEntityID                             core\blackboard\blackboardDefinitions.script:1005
#   GameInstance.FindEntityByID                                              orphans.script:11549
#   Entity.GetWorldPosition / GetWorldRight                                  core\entity\entity.script:44 / :52
#   CameraSystem.GetActiveCameraForward                                      orphans.script:26736
#   UTPPLogLine(line: String) -> 'UTPP CET <line>' in the UTPP log (ms)      Source\plugin\UTPP\scripts\utpp.reds:356

#   Codeware DynamicEntitySpec / DynamicEntitySystem.CreateEntity / GetTagged / DeleteTagged, GetDynamicEntitySystem()
#     red4ext\plugins\Codeware\Scripts\Codeware.Global.reds:43419 / :43436 / :43449 / :43452 / :43460 (installed, read-only)
#   VehicleSystem.GetActivePlayerVehicle -> PlayerVehicle.recordID          vehicleSystem.script:44, orphans.script:41457
#   Entity.GetWorldOrientation / GetWorldForward                            core\entity\entity.script:46 / :50
# Pad/keyboard bindings (r6\config\inputUserMappings.xml, read-only): Choice1 = IK_Pad_X_SQUARE / IK_F (interact,
# 'Enter' vehicle), ExitVehicle_Button = IK_Pad_B_CIRCLE / IK_F, CallVehicle_Button = IK_Pad_DigitRight / IK_V (hold V =
# vehicle selector), Dodge_Button = IK_Pad_B_CIRCLE, SwitchItem_Button = IK_Pad_Y_TRIANGLE, RangedADS = IK_Pad_LeftTrigger.

QUIT_CMD = ("local ok,e=pcall(function() GetSingleton('inkMenuScenario'):GetSystemRequestsHandler():ExitGame() end) "
            "pcall(function() Game.UTPPLogLine('[night] QUIT exitgame ok='..tostring(ok)..' '..tostring(e)) end)")
# the motion driver status line (frames, applied, feedP95us, budget, skip) written into the UTPP log at the end of a run
PERF_STATUS_CMD = ("local ok,s=pcall(function() return Game.UTPPGetMotionStatus() end) "
                   "pcall(function() Game.UTPPLogLine('[night] PERF '..tostring(ok)..' '..tostring(s)) end)")
NIGHT_TAG = 'UTPPNight'
FALLBACK_VEHICLE = os.environ.get('UTPP_NIGHT_VEHICLE', 'Vehicle.v_standard2_archer_hella_player')
# UTPP_NIGHT_VEHICLE=<record> forces that vehicle record (e.g. a car while the active player vehicle is a motorcycle)
_VEH_ID_LUA = ("local id=TweakDBID.new('" + FALLBACK_VEHICLE + "') " if os.environ.get('UTPP_NIGHT_VEHICLE') else
               "local id=Game.GetVehicleSystem():GetActivePlayerVehicle().recordID "
               "if not id or id.hash==0 then id=TweakDBID.new('" + FALLBACK_VEHICLE + "') end ")
# primary: Codeware dynamic entity of the player's active vehicle record, 3 m right / 2 m ahead of V (not persisted)
SPAWN_CODEWARE_CMD = (
    "local ok,e=pcall(function() local des=Game.GetDynamicEntitySystem() des:DeleteTagged('" + NIGHT_TAG + "') "
    "local p=Game.GetPlayer() local a=p:GetWorldPosition() local r=p:GetWorldRight() local f=p:GetWorldForward() "
    + _VEH_ID_LUA +
    "local s=DynamicEntitySpec.new() s.recordID=id "
    "s.position=Vector4.new(a.x+r.x*3+f.x*2,a.y+r.y*3+f.y*2,a.z+0.3,1) s.orientation=EulerAngles.new(0,0,f:ToRotation().yaw):ToQuat() "
    "s.alwaysSpawned=true s.persistState=false s.persistSpawn=false s.tags={'" + NIGHT_TAG + "'} "
    "return des:CreateEntity(s) end) Game.UTPPLogLine('[night] SPAWN codeware ok='..tostring(ok)..' '..tostring(e))")
# fallback 1: the game's own summon (the car drives up to V)
SPAWN_SUMMON_CMD = ("local ok,r=pcall(function() return Game.GetVehicleSystem():SpawnActivePlayerVehicle() end) "
                    "Game.UTPPLogLine('[night] SPAWN summon ok='..tostring(ok)..' r='..tostring(r))")
SPAWN_CMD = SPAWN_CODEWARE_CMD
# Scripted exit = the game's own ExitWithTeleport (vehicleTransition.script:967-997): UnmountingRequest with the
# player's MountingInfo (IMountingFacility.GetMountingInfoSingleWithObjects, orphans.script:22320) and an instant
# MountEventData (psmImports.script:2-8), IMountingFacility.Unmount (orphans.script:22332, UnmountingRequest :35890),
# then TeleportationFacility.Teleport (orphans.script:31935) 2.5 m to the left of the vehicle. Last exit fallback only.
UNMOUNT_CMD = (
    "local ok,e=pcall(function() local p=Game.GetPlayer() local veh=p:GetMountedVehicle() "
    "local mf=Game.GetMountingFacility() local r=UnmountingRequest.new() "
    "r.lowLevelMountingInfo=mf:GetMountingInfoSingleWithObjects(p) local d=MountEventData.new() d.isInstant=true "
    "r.mountData=d mf:Unmount(r) if veh then local b=veh:GetWorldPosition() local rr=veh:GetWorldRight() "
    "Game.GetTeleportationFacility():Teleport(p,Vector4.new(b.x-rr.x*2.5,b.y-rr.y*2.5,b.z+0.3,1),"
    "EulerAngles.new(0,0,p:GetWorldYaw())) end end) "
    "Game.UTPPLogLine('[night] UNMOUNT ok='..tostring(ok)..' '..tostring(e))")
# Scripted mount = the counterpart of ExitWithTeleport: MountingRequest as vehicleComponent.script:3350 (MountEntityToSlot) /
# gameVehicleMountableComponent.script:40-80 build it (MountingInfo childId/parentId/slotId.id, MountEventData slotName /
# mountParentEntityId / isInstant, MountEventOptions; orphans.script:37105 MountingRequest, psmImports.script:2
# MountEventData), IMountingFacility.Mount (orphans.script:22330). The player's driver seat is seat_front_left (the
# PSM seat switch, vehicleTransition.script:1455-1520, uses the same slot names). isInstant=false: the game plays the enter
# animation (our vehiclecam path fires); MOUNT_INSTANT_CMD (isInstant=true) is the second attempt. The vehicle is the
# night-tagged Codeware car; fallbacks: the summoned vehicle (blackboard) and the object V looks at.
NIGHT_SEAT = os.environ.get('UTPP_NIGHT_SEAT', 'seat_front_left')  # passenger probe: seat_front_right


def mount_cmd(instant: bool = False) -> str:
    flag = 'true' if instant else 'false'
    return (
        "local ok,e=pcall(function() local p=Game.GetPlayer() local v=nil local src='tag' "
        "pcall(function() local t=Game.GetDynamicEntitySystem():GetTagged('" + NIGHT_TAG + "') if t and t[1] then v=t[1] end end) "
        "if not v then src='summon' pcall(function() local d=GetAllBlackboardDefs().VehicleSummonData "
        "v=Game.FindEntityByID(Game.GetBlackboardSystem():Get(d):GetEntityID(d.SummonedVehicleEntityID)) end) end "
        "if not v then src='look' pcall(function() local o=Game.GetTargetingSystem():GetLookAtObject(p) "
        "if o and o:IsVehicle() then v=o end end) end "
        "if not v then error('no vehicle') end "
        "local i=MountingInfo.new() local s=MountingSlotId.new() s.id=CName.new('" + NIGHT_SEAT + "') "
        "i.childId=p:GetEntityID() i.parentId=v:GetEntityID() i.slotId=s "
        "local d=MountEventData.new() d.isInstant=" + flag + " d.slotName=CName.new('" + NIGHT_SEAT + "') "
        "d.mountParentEntityId=v:GetEntityID() local o=MountEventOptions.new() o.alive=false "
        "o.occupiedByNonFriendly=false d.mountEventOptions=o local r=MountingRequest.new() "
        "r.lowLevelMountingInfo=i r.preservePositionAfterMounting=false r.mountData=d "
        "Game.GetMountingFacility():Mount(r) return src..' instant=" + flag + "' end) "
        "Game.UTPPLogLine('[night] MOUNT ok='..tostring(ok)..' '..tostring(e))")


MOUNT_CMD = mount_cmd(False)
MOUNT_INSTANT_CMD = mount_cmd(True)
# Fists by script (cycle 21: the test profile starts with a pistol in hand, so an RT tap does not raise fists and LT
# aims). Evidence (Extracted\\game\\scripts\\vanilla-2.31\\cyberpunk\\managers\\quickSlotsManager.script): RequestEquipFists()
# :921 queues EquipmentSystemWeaponManipulationRequest{requestType=RequestFists (19), owner=player} on the
# EquipmentSystem; HideWeapon() :837 does the same with UnequipWeapon (21); player.script:1495 GetQuickSlotsManager().
# FISTS: the quick slots method (nested pcall: a missing method or an error falls back to the raw request).
FISTS_CMD = (
    "local ok,e=pcall(function() local p=Game.GetPlayer() local how='quickslots' local done=false "
    "pcall(function() p:GetQuickSlotsManager():RequestEquipFists() done=true end) "
    "if not done then how='eqsystem' local r=EquipmentSystemWeaponManipulationRequest.new() "
    "r.requestType=Enum.new('EquipmentManipulationAction','RequestFists') r.owner=p "
    "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) end return how end) "
    "Game.UTPPLogLine('[night] FISTS ok='..tostring(ok)..' '..tostring(e))")
FISTS_OFF_CMD = (
    "local ok,e=pcall(function() local p=Game.GetPlayer() "
    "local r=EquipmentSystemWeaponManipulationRequest.new() "
    "r.requestType=Enum.new('EquipmentManipulationAction','UnequipWeapon') r.owner=p "
    "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'eqsystem' end) "
    "Game.UTPPLogLine('[night] FISTS_OFF ok='..tostring(ok)..' '..tostring(e))")


def night_echo(text: str, tag: str) -> tuple[bool | None, str]:
    """(ok, line) of the last '[night] <tag> ok=<true|false> ...' echo in `text`; (None, '') = no echo (command lost)."""
    found = ''
    for line in (text or '').splitlines():
        if f'[night] {tag} ok=' in line:
            found = line
    if not found:
        return None, ''
    info = found.split('[night]', 1)[1].strip()[:160]
    return f'[night] {tag} ok=true' in found, info


# ---------------------------------------------------------------------------------------------- fist fight target
# A real opponent for the melee scenario `melee_fists_target` (nobody had seen the third-person melee on a target: no
# `src=lock` line in any log). Evidence (game scripts 2.31 read-only, Codeware 1.21.2 installed):
#   Codeware DynamicEntitySpec / CreateEntity / GetTagged / DeleteTagged: as the vehicle spawn above; spawnInView / active
#     as the project's ragdoll probe (plugin\UTPP\scripts\utpp_ragdoll_probe.reds UTPPRdEngSpawnNpc: Character record, 2.5 m ahead)
#   Character records: names of the installed game's TweakDB (CET tweakdb\tweakdbstr.kark, decoded read-only):
#     humanoid_tutorial_idle_ma (melee tutorial dummy, "idle"), q000_tutorial_course_04_fistfigher_idle, dummy_test_npc.
#     Only the names are known, not the values (attitude, AI): the state echo (`att=`, `act=`, `hp=`) reports them.
#   The game's melee target: DefaultTransition.GetTargetObject (defaultTransition.script:171-203) = the active puppet closest
#     to the crosshair that is NOT Friendly (:196) - a Friendly target is never locked (so no `src=lock`).
#   AttitudeAgent.SetAttitudeTowards(agent, EAIAttitude) orphans.script:22493; EAIAttitude Friendly 0 / Neutral 1 / Hostile 2 :67.
#   GodModeSystem.AddGodMode / RemoveGodMode orphans.script:20708-20710, gameGodModeType Immortal = 1 (:531): damage is clamped
#     (statPoolsManager.script:309), Invulnerable would block it and the health could never drop.
#   StatPoolsSystem.GetStatPoolValue(objID, type, perc) orphans.script:20547, RequestSettingStatPoolValue :20569.
#   TeleportationFacility.Teleport(obj, Vector4, EulerAngles) orphans.script:31935 (as UNMOUNT_CMD); GameObject.IsActive :1371.
# Tag: ENEMY_TAG, NOT NIGHT_TAG - NightPos / mount_cmd / DESPAWN_CMD read the NIGHT_TAG entity as "the vehicle". The macro
# points NightAuto at ENEMY_TAG for its duration, so every POS line carries the target position in `v=` (src=tag).
ENEMY_TAG = 'UTPPFightTarget'
ENEMY_RECORDS = tuple(r.strip() for r in os.environ.get('UTPP_NIGHT_ENEMY', '').split(',') if r.strip()) or (
    'Character.humanoid_tutorial_idle_ma', 'Character.q000_tutorial_course_04_fistfigher_idle',
    'Character.dummy_test_npc')
ENEMY_SPAWN_DIST = 6.0       # first spawn: beyond the 5 m melee lock range, so the draw tap cannot hit it
ENEMY_POS_PERIOD = 0.2       # NightAuto period during the macro (distance resolution around a punch)
ENEMY_ORBIT_TOL = 8.0        # camera orbit tolerance (deg)
ENEMY_MIN_DROP = 0.5         # health drop (absolute, else percent points) that counts as "the punch hit"
# (phase, target distance along the camera forward in m, health drop required, camera orbit deg, pad part)
# near and orbit are the close-range phases (1.2 m): their punches must connect; far (2.0 m) and block are report only
ENEMY_PHASES = (('far', 2.0, False, 0.0, 'melee_punch'), ('near', 1.2, True, 0.0, 'melee_punch'),
                ('orbit', 1.2, True, 60.0, 'melee_punch'), ('block', 1.5, False, 0.0, 'melee_block'))


def enemy_spawn_cmd(record: str, dist: float = ENEMY_SPAWN_DIST) -> str:
    """Console: Codeware entity of Character record `record`, `dist` m along the flattened camera forward, facing V,
    not persisted, tagged ENEMY_TAG (an older one is deleted first). Echo `[night] ENEMY_SPAWN ok=..`: only the request is
    proven (CreateEntity is asynchronous) - the state echo shows `found=1`. `record missing` = TweakDB has no such record.
    Kept short: the console is typed at ~12 ms per character."""
    d = f'{dist:.2f}'
    return (
        "local ok,e=pcall(function() local d=Game.GetDynamicEntitySystem() d:DeleteTagged('" + ENEMY_TAG + "') "
        "local id=TweakDBID.new('" + record + "') if TweakDB:GetRecord(id)==nil then error('record missing') end "
        "local a=Game.GetPlayer():GetWorldPosition() local c=Game.GetCameraSystem():GetActiveCameraForward() "
        "local m=math.sqrt(c.x*c.x+c.y*c.y) local x,y=c.x/m,c.y/m local s=DynamicEntitySpec.new() s.recordID=id "
        "s.position=Vector4.new(a.x+x*" + d + ",a.y+y*" + d + ",a.z+0.1,1) "
        "s.orientation=EulerAngles.new(0,0,Vector4.new(-x,-y,0,0):ToRotation().yaw):ToQuat() "
        "s.persistState=false s.persistSpawn=false s.alwaysSpawned=true s.spawnInView=true s.active=true "
        "s.tags={'" + ENEMY_TAG + "'} d:CreateEntity(s) end) "
        "Game.UTPPLogLine('[night] ENEMY_SPAWN ok='..tostring(ok)..' '..tostring(e))")


# UTPPENEMY(dist, heal) -> info string (global, defined once by the first state command): reads the target BEFORE it changes
# anything (horizontal distance to V, health absolute and percent, attitude towards V 0/1/2, IsActive), then optionally
# restores the health to 100 % and teleports the target `dist` m along the camera forward (facing V). The reads are pcall'd
# (a failed read leaves the field out instead of losing the line); the target position itself is in the POS lines (v=).
ENEMY_DEFINE = (
    "UTPPENEMY=function(D,H) local p=Game.GetPlayer() local t=Game.GetDynamicEntitySystem():GetTagged('" + ENEMY_TAG + "') "
    "local n=t and t[1] if not n then return 'found=0' end local a=p:GetWorldPosition() local b=n:GetWorldPosition() "
    "local s='found=1 pre_d2='..string.format('%.2f',math.sqrt((a.x-b.x)^2+(a.y-b.y)^2)) "
    "local i,q,h=n:GetEntityID(),Game.GetStatPoolsSystem(),Enum.new('gamedataStatPoolType','Health') "
    "pcall(function() s=s..' pre_hv='..string.format('%.1f',q:GetStatPoolValue(i,h,false))"
    "..' pre_hp='..string.format('%.1f',q:GetStatPoolValue(i,h,true)) end) "
    "pcall(function() s=s..' att='..EnumInt(n:GetAttitudeTowards(p)) end) "
    "pcall(function() s=s..' act='..tostring(n:IsActive()) end) "
    "if H then s=s..' heal='..tostring(pcall(function() q:RequestSettingStatPoolValue(i,h,100,p,true) end)) end "
    "if D then local c=Game.GetCameraSystem():GetActiveCameraForward() local m=math.sqrt(c.x*c.x+c.y*c.y) "
    "local x,y=c.x/m,c.y/m s=s..' moved='..tostring(pcall(function() Game.GetTeleportationFacility():Teleport(n,"
    "Vector4.new(a.x+x*D,a.y+y*D,a.z+0.05,1),EulerAngles.new(0,0,Vector4.new(-x,-y,0,0):ToRotation().yaw)) end)) end "
    "return s end ")


def enemy_cmd(label: str, dist: float | None = None, heal: bool = False, define: bool = False) -> str:
    """Console: one state read (+ the optional heal / teleport of UTPPENEMY). Echo `[night] ENEMY ok=.. label=<label>
    found=1 pre_d2=..`. `define=True` carries the UTPPENEMY definition (first call, or after an echo that says the global
    is gone)."""
    arg = 'nil' if dist is None else f'{dist:.2f}'
    return ((ENEMY_DEFINE if define else '') +
            f"local ok,e=pcall(function() return UTPPENEMY({arg},{'true' if heal else 'false'}) end) "
            f"Game.UTPPLogLine('[night] ENEMY ok='..tostring(ok)..' label={label} '..tostring(e))")


def enemy_prep_cmd(neutral: bool = False) -> str:
    """Console, once after the target is found: Immortal god mode (+ the attitude towards V set to Neutral when `neutral`).
    Echo `[night] ENEMY_PREP ok=.. god=<bool> [attset=<bool>]`."""
    return (
        "local ok,e=pcall(function() local n=Game.GetDynamicEntitySystem():GetTagged('" + ENEMY_TAG + "')[1] "
        "local p=Game.GetPlayer() local s='god='..tostring(pcall(function() Game.GetGodModeSystem():AddGodMode("
        "n:GetEntityID(),Enum.new('gameGodModeType','Immortal'),CName.new('" + ENEMY_TAG + "')) end)) "
        + ("s=s..' attset='..tostring(pcall(function() n:GetAttitudeAgent():SetAttitudeTowards(p:GetAttitudeAgent(),"
           "Enum.new('EAIAttitude','AIA_Neutral')) end)) " if neutral else "") +
        "return s end) Game.UTPPLogLine('[night] ENEMY_PREP ok='..tostring(ok)..' '..tostring(e))")


def enemy_off_cmd() -> str:
    """Console: last state read (label=off), Immortal removed, the tagged entity deleted. Echo `[night] ENEMY_OFF ok=true`
    (always printed last, also when the state read failed). Everything is undone; nothing here touches the save."""
    return (
        "local ok,e=pcall(function() return UTPPENEMY(nil,false) end) "
        "Game.UTPPLogLine('[night] ENEMY ok='..tostring(ok)..' label=off '..tostring(e)) "
        "pcall(function() Game.GetGodModeSystem():RemoveGodMode(Game.GetDynamicEntitySystem():GetTagged('" + ENEMY_TAG +
        "')[1]:GetEntityID(),Enum.new('gameGodModeType','Immortal'),CName.new('" + ENEMY_TAG + "')) end) "
        "pcall(function() Game.GetDynamicEntitySystem():DeleteTagged('" + ENEMY_TAG + "') end) "
        "Game.UTPPLogLine('[night] ENEMY_OFF ok=true')")


ENEMY_ATT_NAMES = {0: 'friendly', 1: 'neutral', 2: 'hostile'}
_ENEMY_KV_RE = re.compile(r'(\w+)=(\S*)')


def _num(s):
    try:
        v = float(s)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _att_value(s) -> int | None:
    """EnumInt value '0'/'1'/'2' or the enum text ('...Friendly...') -> 0/1/2; None = not readable."""
    if s is None or s == '':
        return None
    if s.lstrip('-').isdigit():
        v = int(s)
        return v if v in ENEMY_ATT_NAMES else None
    low = s.lower()
    for v, name in ENEMY_ATT_NAMES.items():
        if name in low:
            return v
    return None


def parse_enemy(text: str, tag: str = 'ENEMY') -> dict | None:
    """The last `[night] <tag> ok=..` line of `text` (UTPPENEMY / ENEMY_PREP echo) -> {'ok', 'label', 'found', 'd2', 'hp'
    (percent), 'hv' (absolute), 'att' (0/1/2/None), 'att_name', 'act', 'heal', 'god', 'attset', 'moved', 'raw'}; d2 / hp /
    hv / att describe the state BEFORE the call changed anything. None = no such line (command lost)."""
    last = None
    for line in (text or '').splitlines():
        if f'[night] {tag} ok=' in line:
            last = line
    if last is None:
        return None
    body = last.split(f'[night] {tag} ', 1)[1].strip()
    kv = dict(_ENEMY_KV_RE.findall(body))
    flag = {'true': True, 'false': False}
    att = _att_value(kv.get('att'))
    return {'ok': kv.get('ok') == 'true', 'label': kv.get('label'), 'found': kv.get('found') == '1',
            'd2': _num(kv.get('pre_d2')), 'hp': _num(kv.get('pre_hp')), 'hv': _num(kv.get('pre_hv')), 'att': att,
            'att_name': ENEMY_ATT_NAMES.get(att), 'act': {'1': True, '0': False}.get(kv.get('act'), flag.get(kv.get('act'))),
            'heal': flag.get(kv.get('heal')), 'god': flag.get(kv.get('god')), 'attset': flag.get(kv.get('attset')),
            'moved': flag.get(kv.get('moved')), 'raw': body[:300]}


def enemy_hp_drop(before: dict | None, after: dict | None) -> float | None:
    """Health drop between two parsed echoes: `before` = the state right after a restore to 100 % (or the previous state),
    `after` = the state read at the next call. Absolute value (hv) when both have it, else percent points. None = unreadable."""
    if not before or not after:
        return None
    for key in ('hv', 'hp'):
        if before.get(key) is not None and after.get(key) is not None:
            return round(before[key] - after[key], 2)
    return None


_MELEEAIM_RE = re.compile(r'UTPP MELEEAIM src=(\w+) tiltW=(-?[\d.]+) tgtPitch=(-?[\d.]+) tgtYaw=(-?[\d.]+) '
                          r'rawPitch=(-?[\d.]+) rawYaw=(-?[\d.]+) pitch=(-?[\d.]+) yaw=(-?[\d.]+)')


def meleeaim_stats(log_text: str) -> dict:
    """`UTPP MELEEAIM src=<lock|hit|cam> ...` lines of a log window (mm.trace on, 4 Hz). Pure.
    -> {'lines', 'src': {name: count}, 'lock', 'lock_pitch' / 'lock_yaw': (min, max) | None, 'lock_raw_pitch' /
        'lock_raw_yaw': (min, max) | None, 'applied_pitch_max', 'applied_yaw_max'}.
    tgt* = the angle towards the target AFTER the clamp (yaw <= 25 deg) and the facing fade, raw* = BEFORE them = the real
    angle between V's body and the target (the camera-orbit number), applied = what the tilt did."""
    src: dict = {}
    lp, ly, lrp, lry, ap, ay = [], [], [], [], [], []
    n = 0
    for m in _MELEEAIM_RE.finditer(log_text or ''):
        n += 1
        src[m.group(1)] = src.get(m.group(1), 0) + 1
        if m.group(1) == 'lock':
            lp.append(float(m.group(3)))
            ly.append(float(m.group(4)))
            lrp.append(float(m.group(5)))
            lry.append(float(m.group(6)))
        ap.append(abs(float(m.group(7))))
        ay.append(abs(float(m.group(8))))
    return {'lines': n, 'src': src, 'lock': src.get('lock', 0),
            'lock_pitch': (min(lp), max(lp)) if lp else None, 'lock_yaw': (min(ly), max(ly)) if ly else None,
            'lock_raw_pitch': (min(lrp), max(lrp)) if lrp else None, 'lock_raw_yaw': (min(lry), max(lry)) if lry else None,
            'applied_pitch_max': max(ap) if ap else None, 'applied_yaw_max': max(ay) if ay else None}


def target_distance_stats(log_text: str) -> dict | None:
    """V <-> target distance (m, horizontal) from the NightAuto POS lines of a window that carry the target in `v=` (src=tag).
    -> {'n', 'first', 'min', 'max', 'last'} or None (no such line)."""
    ds = []
    for line in (log_text or '').splitlines():
        if '[night] POS p=' not in line:
            continue
        pos = parse_pos(line)
        if pos and pos.get('v') and pos.get('src') == 'tag':
            ds.append(math.hypot(pos['p'][0] - pos['v'][0], pos['p'][1] - pos['v'][1]))
    if not ds:
        return None
    return {'n': len(ds), 'first': round(ds[0], 2), 'min': round(min(ds), 2), 'max': round(max(ds), 2),
            'last': round(ds[-1], 2)}


def enemy_evaluate(spawn: dict | None, phases: list[dict], min_drop: float = ENEMY_MIN_DROP) -> dict:
    """Verdict of the fist fight on a target. PASS = the target was spawned and found (not Friendly), at least one
    `src=lock` line in the punch windows, the health dropped in the phases that require it (`near` and `orbit`, both at
    1.2 m; `far` and `block` are report only), no abort (raised elsewhere). Pure.
    A phase dict: {'name', 'required', 'lock', 'drop' (None = unreadable), 'placed' (False = the placement echo was lost),
    'aim': meleeaim_stats}. -> {'fails', 'warns', 'locks', 'hits': {phase: bool | None}}"""
    fails, warns = [], []
    if not spawn or not spawn.get('found'):
        fails.append('target: spawned but never found (no `found=1` state echo)' if spawn is not None else
                     'target: spawn failed (no record spawned)')
    elif spawn.get('att') == 0:
        fails.append('target is Friendly towards V (att=0): the game never locks a Friendly target '
                     '(defaultTransition.script:196); the neutral-attitude alternative did not change it')
    elif spawn.get('att') is None:
        warns.append('target attitude not readable (att missing): lock not guaranteed')
    if spawn and spawn.get('found') and spawn.get('act') is False:
        warns.append('target reports IsActive=false: GetTargetObject skips an inactive puppet')
    locks = sum(p.get('lock') or 0 for p in phases)
    lines = sum((p.get('aim') or {}).get('lines', 0) for p in phases)
    if phases and locks == 0:
        if lines == 0:
            fails.append('no `MELEEAIM` line at all in the punch windows (mm.trace off or "Melee aim tilt" disabled): '
                         'src=lock cannot be judged')
        else:
            srcs = {}
            for p in phases:
                for k, v in ((p.get('aim') or {}).get('src') or {}).items():
                    srcs[k] = srcs.get(k, 0) + v
            fails.append(f'no `src=lock` MELEEAIM line (only {srcs}): the game did not track the target '
                         '(GetTargetObject: not closest to the crosshair / out of 5 m / Friendly / inactive)')
    hits: dict = {}
    for p in phases:
        name, drop = p['name'], p.get('drop')
        hit = None if drop is None else drop >= min_drop
        hits[name] = hit
        if p.get('placed') is False:
            (fails if p.get('required') else warns).append(f'{name}: the placement command was lost (no echo)')
            continue
        if p.get('required'):
            if drop is None:
                fails.append(f'{name}: health unreadable (no hp/hv in the state echo): "health dropped" cannot be shown')
            elif not hit:
                fails.append(f'{name}: health did not drop (drop {drop}): the punches did not connect'
                             + (' - after the camera orbit the hit does not follow the camera / target' if name == 'orbit'
                                else ''))
    return {'fails': fails, 'warns': warns, 'locks': locks, 'hits': hits}


def punch_report_cmd(python: str, script: str, log_path: str) -> list[str]:
    """Command line of the offline measuring tool (Source\\tools\\analysis\\punch_direction_report.py, another worker): the
    UTPP log of the run as its only argument. The one place to adapt when its interface is fixed."""
    return [python, '-I', script, log_path]


PUNCH_REPORT_REL = os.path.join('..', 'analysis', 'punch_direction_report.py')


DESPAWN_CMD = "pcall(function() Game.GetDynamicEntitySystem():DeleteTagged('" + NIGHT_TAG + "') end)"
# fallback 2 (manual procedure): hold keyboard V ~1 s (vehicle selector), Enter; scan codes
SCAN_V, SCAN_F = 0x2F, 0x21
SCAN_L = 0x26   # UTPP shoulder swap, default keyboard binding (r6\input\utpp_shoulder.xml)
SCAN_DOWN = 0xE050   # Down arrow (extended scan code; gt_win32 sets KEYEVENTF_EXTENDEDKEY for > 0xFF)
POS_FUNC_DEF = (
    "NIGHTPOS=function() local p=Game.GetPlayer() local a=p:GetWorldPosition() "
    "local f=Game.GetCameraSystem():GetActiveCameraForward() local s='nil' local src='none' local v=nil "
    "pcall(function() local t=Game.GetDynamicEntitySystem():GetTagged('" + NIGHT_TAG + "') "
    "if t and t[1] then v=t[1] src='tag' end end) "
    "if not v then pcall(function() local d=GetAllBlackboardDefs().VehicleSummonData "
    "v=Game.FindEntityByID(Game.GetBlackboardSystem():Get(d):GetEntityID(d.SummonedVehicleEntityID)) "
    "if v then src='summon' end end) end "
    "if v then local b=v:GetWorldPosition() local r=v:GetWorldRight() "
    "s=string.format('%.2f,%.2f,%.2f,%.3f,%.3f',b.x,b.y,b.z,r.x,r.y) end "
    "Game.UTPPLogLine(string.format('[night] POS p=%.2f,%.2f,%.2f cf=%.3f,%.3f v=%s src=%s',"
    "a.x,a.y,a.z,f.x,f.y,s,src)) end")
POS_CALL = 'NIGHTPOS()'
# 0.5.2 utpp_core probe (Source\plugin\UTPP\cet\utpp_core\init.lua M.NightPos): 42 characters instead of ~760
NIGHTPOS_CMD = 'GetMod("utpp_core").NightPos("' + NIGHT_TAG + '")'


def nightauto_cmd(period: float = 0.5, on: bool = True, tag: str = NIGHT_TAG) -> str:
    """utpp_core 0.5.4: POS line every `period` s from onUpdate (echo '[night] auto on period=...'). `tag` = the tagged
    Codeware entity reported in `v=` (NIGHT_TAG = the vehicle, ENEMY_TAG = the fist fight target)."""
    return f'GetMod("utpp_core").NightAuto({"true" if on else "false"}, {period:.2f}, "{tag}")'


def mark_cmds(tag: str) -> list[str]:
    """Marker: print() for the CET log (may not reach a file) + UTPPLogLine (ms timestamp in the UTPP log)."""
    return [f'print("UTPP_MARK {tag}")', f'Game.UTPPLogLine("[night] UTPP_MARK {tag}")']


def arm_cmds(db: str, extra: list[str] | None = None) -> list[str]:
    # db 'none': keep the databases the mod loads by itself (the player's default setup)
    load = [] if db in ('', 'none') else [f'Game.UTPPLoadDatabase("{db}")']
    return ['Game.UTPPSetMMEnabled(true)', 'Game.UTPPSetMMTrace(true)'] + load + list(extra or [])


# ---------------------------------------------------------------------------------------------- UTPP log events

GAMEPLAY_PATTERNS = ('player animated object/tracks changed, re-arming', 'TPP <- user', 'MMGATE')
MOUNT_RIDE = 'keepTpp mount -> ride'  # '[utpp_vehiclecam] t=.. keepTpp mount -> ride'
# utpp_perspective lines that only appear with a weapon in the hand (UTPP weapon policy decisions)
DRAW_PATTERNS = ('FPP <- weapon', 'weapon drawn')
HOLSTER_PATTERNS = ()  # no holster-only perspective log line exists; holster is confirmed by NightPos w=0 only
EXIT_PULL_END = 'exit pull: end'
POS_RE = re.compile(r'\[night\] POS p=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+) cf=(-?[\d.]+),(-?[\d.]+) v=(\S+)'
                    r'(?: src=(\w+))?(?: w=([01-]))?(?: m=([01-]))?')
CAMVEH_RE = re.compile(r'cam veh=\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)')
LOGTS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')


def gameplay_seen(text: str) -> str | None:
    """First gameplay indicator found in new UTPP log text (None = still loading/menu)."""
    for p in GAMEPLAY_PATTERNS:
        if p in text:
            return p
    return None


def speed_calibrated(text: str, db: str) -> bool:
    return f'speed calibration [{db}]' in text


def find_line_ts(text: str, needle: str) -> datetime | None:
    """Timestamp of the first UTPP log line containing `needle`."""
    for line in text.splitlines():
        if needle in line:
            m = LOGTS_RE.match(line)
            if m:
                return datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
    return None


MOUNT_START = 'keepTpp off -> mount'  # '[utpp_vehiclecam] t=.. keepTpp off -> mount (utppCamera kept active, mount start)'


def mount_seen(text: str) -> bool:
    """Mounted: the mount start ('keepTpp off -> mount'), 'keepTpp mount -> ride' or a utpp_vehiclecam 'handover'
    line. The start line counts: the ride line comes only when V is seated, 4-6 s after X (2026-11-12 11:14: X at
    :40, mount start :42.7, ride :46.9 - the 3 s wait reported 'no mount line' for an X that had worked)."""
    if MOUNT_RIDE in text or MOUNT_START in text:
        return True
    return any('[utpp_vehiclecam]' in l and 'handover' in l for l in text.splitlines())


def mount_ts(text: str) -> datetime | None:
    """Time of the first mount line (start, ride or handover)."""
    for line in text.splitlines():
        if MOUNT_RIDE in line or MOUNT_START in line or ('[utpp_vehiclecam]' in line and 'handover' in line):
            m = LOGTS_RE.match(line)
            if m:
                return datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
    return None


def weapon_state(pos: dict | None, log_text: str, expect_drawn: bool) -> tuple[bool | None, str]:
    """(confirmed, evidence) for a draw (expect_drawn) or holster step.

    Primary: NightPos w= flag (item in AttachmentSlots.WeaponRight). Fallback: utpp_perspective lines in the step's
    log window (draw: weapon policy decision lines; holster: no log evidence). None = unknown.
    """
    if pos and pos.get('weapon') is not None:
        return pos['weapon'] == expect_drawn, f'NightPos w={int(pos["weapon"])}'
    pats = DRAW_PATTERNS if expect_drawn else HOLSTER_PATTERNS
    for line in log_text.splitlines():
        if '[utpp_perspective]' in line and any(p in line for p in pats):
            return True, 'log: ' + line.split('[utpp_perspective]', 1)[1].strip()[:100]
    return None, 'no NightPos weapon flag and no utpp_perspective evidence'


MELEE_W_MIN = 0.9      # MM weight that must hold while the melee upper-body mask is active (lower body from MM)
MELEE_BLEND_S = 0.3    # grace after a mask "on" (mask blend ~0.15 s + one log/tick period) before w is judged
MELEE_OFF_WINDOW_S = 3.0
_MM_W_RE = re.compile(r'(?<![A-Za-z])w=(-?\d+\.\d+)')          # MM weight "w=1.00" (NightPos w=0/1 is the weapon flag)
_MASK_EVT_RE = re.compile(r'MMMASK (on|off) reasons=0x([0-9A-Fa-f]+)')
_MASK_TICK_RE = re.compile(r'\bmask=-?[\d.]+/0x([0-9A-Fa-f]+)')   # MMTRACE tick: mask=<blend>/0x<reasons>
_GATE_MELEE_RE = re.compile(r'\bmelee=0x([0-9A-Fa-f]+)')
_GATE_HOLD_RE = re.compile(r'\bmeleeHold=(\d+)')
_GATE_MASK_RE = re.compile(r'\bmask=0x([0-9A-Fa-f]+)')


def melee_state(pos_lines, log_text: str, active: bool = False, w_min: float = MELEE_W_MIN,
                blend_s: float = MELEE_BLEND_S) -> dict:
    """Melee upper-body mask (UTPP 0.6.2) in a UTPP log window. Pure.

    Melee is active from `MMMASK on reasons=0x<..bit 1..>` / `MMGATE ... melee=0x<nonzero>` (or meleeHold=1, or
    mask=0x<bit 1>) / MMTRACE tick `mask=<b>/0x<bit 1>` until `MMMASK off` / a line without any melee bit. `active`
    = melee already active when the window starts (carried from the previous window). The MM weight comes from the
    `w=` of MMGATE / MMMASK / MMTRACE tick lines (NOT from NightPos: its `w=` is the weapon flag); a sample taken
    while active and later than `blend_s` after the mask went on counts, w < w_min is a violation. `pos_lines` =
    NightPos lines of the window (only used for the count and `m=1` = mounted, where a vehicle mask legitimately
    has w=0). Returns {'on': mask went on in the window, 'off': it went off, 'active_now', 'episodes', 'min_w'
    (None = no sample), 'samples', 'bad': [(ts, w)], 'pos_n', 'mounted', 'first_on'}."""
    rows = []
    for i, line in enumerate(log_text.splitlines()):
        if 'MMMASK' not in line and 'MMGATE' not in line and 'MMTRACE tick' not in line:
            continue
        m = LOGTS_RE.match(line)
        ts = datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f') if m else None
        rows.append((ts, i, line))
    if rows and all(r[0] is not None for r in rows):
        rows.sort(key=lambda r: (r[0], r[1]))          # the log is written by several threads
    out = {'on': False, 'off': False, 'episodes': 0, 'min_w': None, 'samples': 0, 'bad': [], 'first_on': None}
    on_ts = None
    for ts, _, line in rows:
        was, new, is_off_evt = active, active, False
        if 'MMMASK' in line:
            m = _MASK_EVT_RE.search(line)
            if not m:
                continue
            new = bool(int(m.group(2), 16) & 1)
            is_off_evt = m.group(1) == 'off'
        elif 'MMGATE' in line:
            mb, mh, mk = _GATE_MELEE_RE.search(line), _GATE_HOLD_RE.search(line), _GATE_MASK_RE.search(line)
            if not (mb or mh or mk):
                continue
            new = bool(mb and int(mb.group(1), 16)) or bool(mh and int(mh.group(1))) or bool(mk and int(mk.group(1), 16) & 1)
        else:
            mk = _MASK_TICK_RE.search(line)
            if mk:
                new = bool(int(mk.group(1), 16) & 1)
        if new and not was:
            out['on'] = True
            out['episodes'] += 1
            on_ts = ts
            out['first_on'] = out['first_on'] or (ts.strftime('%H:%M:%S.%f')[:-3] if ts else None)
        elif was and not new:
            out['off'] = True
        active = new
        wm = _MM_W_RE.search(line)
        if wm and active and not is_off_evt:
            if on_ts is not None and ts is not None and (ts - on_ts).total_seconds() < blend_s:
                continue
            w = float(wm.group(1))
            out['samples'] += 1
            out['min_w'] = w if out['min_w'] is None else min(out['min_w'], w)
            if w < w_min:
                out['bad'].append((ts.strftime('%H:%M:%S.%f')[:-3] if ts else None, w))
    pos = [l for l in (pos_lines or []) if '[night] POS' in l]
    out.update(active_now=active, pos_n=len(pos), mounted=any(re.search(r'\bm=1\b', l) for l in pos))
    return out


# ---------------------------------------------------------------------------------------------- aim probe (utpp_probe AimProbe)
AIMPROBE_MIN_SAMPLES = 4


def aiming_cmd(mode: str) -> str:
    """Console: utpp_core Set("perspective.aiming", <tpp|fpp>) (noSave); echo '[night] AIMPROBE aiming=<mode> ok=..'."""
    return ('local ok,e=pcall(function() return GetMod("utpp_core").Set("perspective.aiming","' + mode + '",{noSave=true}) end) '
            "Game.UTPPLogLine('[night] AIMPROBE aiming=" + mode + " ok='..tostring(ok)..' '..tostring(e))")


def aimprobe_cmd(on: bool) -> str:
    """Console: utpp_probe AimProbe(true|false); echo '[night] AIMPROBE on|off ok=..'."""
    return ('local ok,e=pcall(function() GetMod("utpp_probe").AimProbe(' + ('true' if on else 'false') + ') end) '
            "Game.UTPPLogLine('[night] AIMPROBE " + ('on' if on else 'off') + " ok='..tostring(ok)..' '..tostring(e))")


_AIMPROBE_LINE_RE = re.compile(r'\[utpp_probe\]\s+aimprobe\s+(sample|shot|summary)\b(.*)')
_KV_RE = re.compile(r'(\w+)=(\([^)]*\)|\S+)')
_MM_MASK_ON_RE = re.compile(r'MMMASK on reasons=0x([0-9A-Fa-f]+)')


def _aim_num(s):
    """'1.25' -> 1.25; nan / inf / garbage / missing -> None (strict-JSON friendly)."""
    try:
        v = float(s)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _aim_vec(s):
    """'(1,2,3)' -> (1.0, 2.0, 3.0); 'none' / missing / unparsable / non-finite component -> None."""
    if not s or not s.startswith('('):
        return None
    parts = [_aim_num(x) for x in s.strip('()').split(',')]
    return tuple(parts) if parts and all(p is not None for p in parts) else None


def _aim_int(s):
    return int(s) if s and s.isdigit() else None


def aimprobe_state(log_text: str) -> dict:
    """utpp_probe AimProbe lines in a UTPP log window. Pure.

    `[utpp_probe] aimprobe sample ...` -> counted; `... shot n=<i> origin=(x,y,z) dir=(x,y,z) dCam= dFpp= dMuzzle=
    angCam= angFpp= hit=(x,y,z)|none src=<name>` -> {n, origin, dir, dCam, dFpp, dMuzzle, angCam, angFpp, hit, src}
    (missing / nan / inf values = None); `... summary samples= shots= originNearest=cam|fpp|muzzle|unknown meanDCam=
    meanDFpp= meanDMuzzle= meanAngCam= meanAngFpp=` -> the last one. tppDuringAim = `MMMASK on reasons=0x<bit 2>` or
    `MMGATE ... upperBody=6 ... tpp=1` seen (the aim ran in TPP). Returns {'samples', 'shots': [..], 'summary':
    dict|None, 'tppDuringAim'}."""
    out = {'samples': 0, 'shots': [], 'summary': None, 'tppDuringAim': False}
    for line in (log_text or '').splitlines():
        m = _AIMPROBE_LINE_RE.search(line)
        if m:
            kind, kv = m.group(1), dict(_KV_RE.findall(m.group(2)))
            if kind == 'sample':
                out['samples'] += 1
            elif kind == 'shot':
                out['shots'].append({
                    'n': _aim_int(kv.get('n')), 'origin': _aim_vec(kv.get('origin')), 'dir': _aim_vec(kv.get('dir')),
                    'dCam': _aim_num(kv.get('dCam')), 'dFpp': _aim_num(kv.get('dFpp')),
                    'dMuzzle': _aim_num(kv.get('dMuzzle')), 'angCam': _aim_num(kv.get('angCam')),
                    'angFpp': _aim_num(kv.get('angFpp')), 'hit': _aim_vec(kv.get('hit')), 'src': kv.get('src')})
            else:
                out['summary'] = {
                    'samples': _aim_int(kv.get('samples')), 'shots': _aim_int(kv.get('shots')),
                    'originNearest': kv.get('originNearest'),
                    **{k: _aim_num(kv.get(k)) for k in ('meanDCam', 'meanDFpp', 'meanDMuzzle', 'meanAngCam',
                                                        'meanAngFpp')}}
            continue
        mm = _MM_MASK_ON_RE.search(line)
        if mm and int(mm.group(1), 16) & 2:
            out['tppDuringAim'] = True
        elif 'MMGATE' in line and re.search(r'\bupperBody=6\b', line) and re.search(r'\btpp=1\b', line):
            out['tppDuringAim'] = True
    return out


def aimprobe_fails(st: dict) -> list[str]:
    """Reasons the AimProbe window is not a PASS (empty = PASS): >= 4 samples, >= 1 shot, a summary line."""
    fails = []
    if st['samples'] < AIMPROBE_MIN_SAMPLES:
        fails.append('no samples' if not st['samples'] else
                     f'only {st["samples"]} aimprobe samples (< {AIMPROBE_MIN_SAMPLES})')
    if not st['shots']:
        fails.append('no shots (no "aimprobe shot" line)')
    if st['summary'] is None:
        fails.append('no summary (no "aimprobe summary" line)')
    return fails


def movement_seen(lines: list, t0: datetime, t1: datetime, min_speed: float = 0.5) -> bool | None:
    """Did MMTRACE show |v| >= min_speed in [t0, t1]? None = no trace lines at all (trace off / game paused)."""
    sel = [l for l in lines if l.is_trace and t0 <= l.ts <= t1]
    if not sel:
        return None
    return any((l.num('|v|') or 0.0) >= min_speed for l in sel)


def commands_movement(sc, threshold: float = 0.3) -> bool:
    """Does a scenario push the left stick to at least `threshold` in a step?"""
    return any(mx is not None and mx >= threshold for _, mx in sc.step_stick_stats().values())


def parse_pos(text: str) -> dict | None:
    """Last '[night] POS' line -> {'p': (x,y,z), 'cf': (x,y), 'v': (x,y,z) | None, 'vr': (x,y) | None}."""
    found = None
    for m in POS_RE.finditer(text):
        found = m
    if not found:
        return None
    g = found.groups()
    flag = {'1': True, '0': False}
    out = {'p': tuple(float(v) for v in g[0:3]), 'cf': (float(g[3]), float(g[4])), 'v': None, 'vr': None,
           'src': g[6] or None, 'weapon': flag.get(g[7]), 'mounted': flag.get(g[8])}
    if g[5] != 'nil':
        vals = [float(v) for v in g[5].split(',')]
        if len(vals) >= 5:
            out['v'], out['vr'] = tuple(vals[0:3]), (vals[3], vals[4])
    return out


def door_point(pos: dict, side_offset: float = 1.3) -> tuple[float, float] | None:
    """Driver door point = vehicle centre - right * offset (left-hand drive); None without a vehicle."""
    if not pos or not pos.get('v'):
        return None
    vx, vy = pos['v'][0], pos['v'][1]
    rx, ry = pos.get('vr') or (0.0, 0.0)
    n = math.hypot(rx, ry)
    if n < 1e-6:
        return vx, vy
    return vx - rx / n * side_offset, vy - ry / n * side_offset


def approach_stick(player_xy, cam_fwd_xy, target_xy, mag: float = 0.6) -> tuple[float, float, float]:
    """Camera-relative left-stick (x right, y forward) toward target, plus the horizontal distance.

    World: z up; camera right = (f.y, -f.x) for forward f (forward (0,1) -> right (1,0)).
    """
    dx, dy = target_xy[0] - player_xy[0], target_xy[1] - player_xy[1]
    dist = math.hypot(dx, dy)
    fx, fy = cam_fwd_xy
    fn = math.hypot(fx, fy)
    if dist < 1e-6 or fn < 1e-6:
        return 0.0, 0.0, dist
    fx, fy = fx / fn, fy / fn
    rx, ry = fy, -fx
    sx, sy = (dx * rx + dy * ry) / dist, (dx * fx + dy * fy) / dist
    return round(sx * mag, 4), round(sy * mag, 4), dist


# ---------------------------------------------------------------------------------------------- ffmpeg

GAME_TITLE = 'Cyberpunk 2077 (C) 2020 by CD Projekt RED'
START_RE = re.compile(r'Input #0, (\w+),.*?\n\s*Duration: [^,]+, start: (-?\d+\.\d+)', re.S)
SIZE_RE = re.compile(r'Stream #0:0[^\n]*?Video: [^\n]*?, (\d{2,5})x(\d{2,5})')
TIME_RE = re.compile(r'time=(\d+):(\d\d):(\d\d(?:\.\d+)?)')
YAVG_RE = re.compile(r'signalstats\.YAVG=([\d.]+)')
FRAG_FLAGS = '+frag_keyframe+empty_moov+default_base_moof'   # a killed recording stays decodable
CAPTURE_MODES = ('desktop', 'ddagrab')                        # gdigrab title= records black frames (D3D12 flip model)


def even_rect(rect) -> tuple[int, int, int, int]:
    """(left, top, right, bottom) -> (x, y, w, h) with even w/h (yuv420p)."""
    left, top, right, bottom = (int(v) for v in rect)
    w, h = right - left, bottom - top
    return left, top, w - (w % 2), h - (h % 2)


def clamp_rect(rect, screen):
    """Intersect the client rect with its monitor (a window hanging over the edge would make gdigrab fail)."""
    if rect is None or screen is None:
        return rect
    l, t, r, b = rect
    return max(l, screen[0]), max(t, screen[1]), min(r, screen[2]), min(b, screen[3])


def crop_needed(rect, screen) -> bool:
    """Crop unless the game client rect covers the whole monitor (screen = monitor rect, physical px)."""
    if rect is None:
        return False
    return screen is None or tuple(int(v) for v in rect) != tuple(int(v) for v in screen)


def encoder_args(encoder: str, fps: int = 30) -> list[str]:
    gop = ['-g', str(fps * 2)]
    if encoder == 'h264_nvenc':
        return ['-c:v', 'h264_nvenc', '-preset', 'p4', '-rc', 'vbr', '-cq', '23', '-b:v', '0', '-pix_fmt', 'yuv420p'] + gop
    if encoder == 'libx264':
        return ['-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '23', '-pix_fmt', 'yuv420p'] + gop
    raise ValueError(f'unknown encoder {encoder!r}')


def build_ffmpeg_cmd(ffmpeg: str, out: str, mode: str = 'desktop', rect=None, screen=None, fps: int = 30,
                     encoder: str = 'h264_nvenc') -> list[str]:
    """Recording command (fragmented mp4; stop gracefully with 'q' on stdin).

    desktop: gdigrab of the desktop, always cropped to the game client rect (physical px; also right for multi-monitor
    desktops when the window covers its monitor);
    ddagrab: Desktop Duplication of monitor 0, downloaded to the CPU and cropped with crop= relative to `screen` unless
    the window covers the monitor.
    """
    cmd = [ffmpeg, '-hide_banner', '-nostats', '-loglevel', 'info', '-y']
    vf = []
    rect = clamp_rect(rect, screen)
    if mode == 'desktop':
        cmd += ['-f', 'gdigrab', '-framerate', str(fps), '-draw_mouse', '0']
        if rect is not None:
            x, y, w, h = even_rect(rect)
            cmd += ['-offset_x', str(x), '-offset_y', str(y), '-video_size', f'{w}x{h}']
        cmd += ['-i', 'desktop']
    elif mode == 'ddagrab':
        cmd += ['-f', 'lavfi', '-i', f'ddagrab=output_idx=0:framerate={fps}:draw_mouse=0']
        vf = ['hwdownload', 'format=bgra']
        if crop_needed(rect, screen):
            x, y, w, h = even_rect(rect)
            ox, oy = (int(screen[0]), int(screen[1])) if screen else (0, 0)
            vf.append(f'crop={w}:{h}:{x - ox}:{y - oy}')
    else:
        raise ValueError(f'unknown capture mode {mode!r} (title= is not supported: black frames)')
    vf.append('scale=trunc(iw/2)*2:trunc(ih/2)*2')
    cmd += ['-vf', ','.join(vf)] + encoder_args(encoder, fps) + ['-r', str(fps), '-movflags', FRAG_FLAGS, out]
    return cmd


def parse_frame_size(stderr: str) -> tuple[int, int] | None:
    m = SIZE_RE.search(stderr)
    return (int(m.group(1)), int(m.group(2))) if m else None


def parse_decoded_seconds(stderr: str) -> float | None:
    """Last 'time=' of a decode pass (true length of a killed fragmented recording)."""
    t = TIME_RE.findall(stderr)
    if not t:
        return None
    h, m, sec = t[-1]
    return int(h) * 3600 + int(m) * 60 + float(sec)


def recording_check(frame_size, expected_wh, yavgs: list, min_luma: float = 2.0) -> list[str]:
    """Problems of a finished recording (empty = OK): black frames, wrong frame size."""
    probs = []
    if not yavgs:
        probs.append('brightness not measurable')
    elif sum(yavgs) / len(yavgs) < min_luma:
        probs.append(f'BLACK recording: mean luma {sum(yavgs) / len(yavgs):.2f} < {min_luma}')
    if expected_wh and frame_size and tuple(frame_size) != tuple(expected_wh):
        probs.append(f'frame size {frame_size[0]}x{frame_size[1]} != game client {expected_wh[0]}x{expected_wh[1]}')
    if frame_size is None:
        probs.append('frame size not readable')
    return probs


def parse_input_start(stderr: str) -> tuple[str, float] | None:
    """('gdigrab', start_seconds) from ffmpeg's input banner. gdigrab stamps frames with av_gettime() (Unix epoch
    microseconds), so start is the wall clock of the first captured frame."""
    m = START_RE.search(stderr)
    return (m.group(1), float(m.group(2))) if m else None


def recording_clock(spawn_wall: datetime, stop_wall: datetime, duration: float, input_start: tuple | None = None,
                    marker_send: datetime | None = None, marker_log: datetime | None = None,
                    same_clock_tolerance: float = 0.25) -> dict:
    """Video start wall time and video_sheets arguments.

    video start: gdigrab epoch start (if it is a plausible epoch within 10 s after spawn) else the spawn time.
    cross-check: stop_wall - duration (graceful 'q' stops within ~1 frame; difference = start/stop latency).
    marker: log_minus_wall = UTPP log timestamp of the marker - wall time Enter was sent. Within tolerance the log
    and wall clocks agree (difference = input->log latency) and clock_offset = 0; otherwise the clocks differ and
    clock_offset = log_minus_wall.
    """
    method, start = 'spawn_time', spawn_wall
    if input_start and input_start[1] > 1e9:
        cand = datetime.fromtimestamp(input_start[1])
        if timedelta(seconds=-1) <= cand - spawn_wall <= timedelta(seconds=10):
            method, start = f'{input_start[0]}_epoch_start', cand
    from_end = stop_wall - timedelta(seconds=duration)
    log_minus_wall = (marker_log - marker_send).total_seconds() if marker_send and marker_log else None
    offset = 0.0
    if log_minus_wall is not None and abs(log_minus_wall) > same_clock_tolerance:
        offset = log_minus_wall
    cross = (from_end - start).total_seconds()
    frame = 1.0 / 30
    unc = frame + (abs(log_minus_wall) if log_minus_wall is not None and offset == 0.0 else 0.25)
    if method == 'spawn_time':
        unc += max(abs(cross), 0.5)
    return {'video_start': start, 'video_end': start + timedelta(seconds=duration), 'start_method': method,
            'start_from_stop': from_end, 'start_vs_stop_delta_s': round(cross, 3), 'duration_s': duration,
            'log_minus_wall_s': None if log_minus_wall is None else round(log_minus_wall, 3),
            'clock_offset_s': round(offset, 3), 'uncertainty_s': round(unc, 3)}


def sheet_windows(video_start: datetime, duration: float, scenarios: list[dict], pad: float = 3.0) -> list[dict]:
    """0.1 s windows around each scenario start and end (video seconds, clipped to the recording)."""
    out = []
    for sc in scenarios:
        for edge in ('start', 'end'):
            t = sc.get(edge)
            if not t:
                continue
            vt = (t - video_start).total_seconds()
            a, b = max(0.0, vt - pad), min(duration - 0.05, vt + pad)
            if b > a:
                out.append({'name': f'{sc["name"]}_{edge}', 'start': round(a, 2), 'end': round(b, 2)})
    return out


# ---------------------------------------------------------------------------------------------- quit decision

def quit_action(alive: bool, elapsed: float, cet_sent: bool, taskkill_sent: bool, soft_timeout: float = 60.0,
                kill_grace: float = 20.0) -> str:
    """Next step of ensure_closed: 'done' | 'wait' | 'taskkill' | 'taskkill_force'."""
    if not alive:
        return 'done'
    if cet_sent and elapsed < soft_timeout:
        return 'wait'
    if not taskkill_sent:
        return 'taskkill'
    if elapsed < (soft_timeout if cet_sent else 0.0) + kill_grace:
        return 'wait'
    return 'taskkill_force'


# ---------------------------------------------------------------------------------------------- night metrics

def _in(l, t0, t1) -> bool:
    return (t0 is None or l.ts >= t0) and (t1 is None or l.ts <= t1)


def vehiclecam_episodes(lines: list, gap_s: float = 2.0) -> list[dict]:
    """'cam veh=(x,y,z)' samples split into episodes by gaps > gap_s; max sample-to-sample jump per episode."""
    eps: list[dict] = []
    prev = None
    for l in lines:
        if l.kind != 'CET':
            continue
        m = CAMVEH_RE.search(l.raw)
        if not m:
            continue
        v = tuple(float(x) for x in m.groups())
        if prev is None or (l.ts - prev[0]).total_seconds() > gap_s:
            eps.append({'start': l.ts, 'end': l.ts, 'samples': 1, 'max_jump_m': 0.0, 'max_jump_at': None})
        else:
            ep = eps[-1]
            ep['end'] = l.ts
            ep['samples'] += 1
            j = math.dist(v, prev[1])
            if j > ep['max_jump_m']:
                ep['max_jump_m'], ep['max_jump_at'] = round(j, 3), l.ts
        prev = (l.ts, v)
    return eps


def night_metrics(lines: list, t0: datetime | None = None, t1: datetime | None = None,
                  cost_limit: float = 60.0, sample_lines: int = 5) -> dict:
    """Extra metrics for summary.json on parsed UTPP log lines (gt_log.parse_*), optionally windowed."""
    sel = [l for l in lines if _in(l, t0, t1)]
    trans = [l for l in sel if l.kind == 'TRANS']
    high = [l for l in trans if (l.num('cost') or 0.0) > cost_limit]
    w1 = [l.num('|v|') for l in sel if l.is_trace and l.num('w') is not None and l.num('w') >= 0.999
          and l.num('|v|') is not None]
    gates = [l for l in sel if l.kind == 'GATE']
    if sel:
        a = t0 or sel[0].ts
        b = t1 or sel[-1].ts
        minutes = max((b - a).total_seconds() / 60.0, 1e-6)
    else:
        minutes = 0.0
    failed = [l for l in sel if l.kind == 'CET' and '[utpp_body]' in l.raw and 'FAILED' in l.raw]
    policy = [l for l in sel if l.kind == 'CET' and 'policy difference' in l.raw]
    eps = vehiclecam_episodes(sel)

    def short(l):
        return l.compact()
    return {
        'window': [t0.isoformat(sep=' ') if t0 else None, t1.isoformat(sep=' ') if t1 else None],
        'trans_total': len(trans),
        f'trans_cost_gt_{int(cost_limit)}': len(high),
        'trans_cost_max': round(max((l.num('cost') or 0.0) for l in trans), 3) if trans else None,
        'trans_cost_gt_samples': [short(l) for l in high[:sample_lines]],
        'max_v_w1': round(max(w1), 3) if w1 else None,
        'w1_samples': len(w1),
        'gate_lines': len(gates),
        'gate_flips_per_min': round(len(gates) / minutes, 2) if minutes else None,
        'vehiclecam_episodes': [{'start': e['start'].isoformat(sep=' '), 'end': e['end'].isoformat(sep=' '),
                                 'samples': e['samples'], 'max_jump_m': e['max_jump_m'],
                                 'max_jump_at': e['max_jump_at'].isoformat(sep=' ') if e['max_jump_at'] else None}
                                for e in eps],
        'vehiclecam_max_jump_m': max((e['max_jump_m'] for e in eps), default=None),
        'utpp_body_failed': len(failed),
        'utpp_body_failed_samples': [short(l) for l in failed[:sample_lines]],
        'perspective_policy_difference': len(policy),
        'perspective_policy_samples': [short(l) for l in policy[:sample_lines]],
    }


# ---------------------------------------------------------------------------------------------- round 3 helpers

SCAN_C = 0x2E   # ToggleCrouch_Button IK_C (inputUserMappings.xml: <button id="IK_C" overridableUI="crouchToggle"/>)
METRIC_FAIL_RE = re.compile(r'^[A-Za-z_@.|]+=[-\d.eE+]+ [<>] [-\d.eE+]+$')


def dead_reckon(xy, cam_fwd_xy, stick, seconds: float, speed: float) -> tuple[float, float]:
    """Estimated position after holding the camera-relative stick (x right, y forward) for `seconds`."""
    fx, fy = cam_fwd_xy
    n = math.hypot(fx, fy)
    if n < 1e-6:
        return tuple(xy)
    fx, fy = fx / n, fy / n
    rx, ry = fy, -fx
    sx, sy = stick
    m = math.hypot(sx, sy)
    if m < 1e-6:
        return tuple(xy)
    wx, wy = (sx * rx + sy * fx) / m, (sx * ry + sy * fy) / m
    d = speed * seconds
    return xy[0] + wx * d, xy[1] + wy * d


def rotate_stick(sx: float, sy: float, deg: float) -> tuple[float, float]:
    """Stick vector rotated clockwise by deg (stick convention: 0 = forward, + = right)."""
    a = math.radians(deg)
    return round(sx * math.cos(a) + sy * math.sin(a), 4), round(-sx * math.sin(a) + sy * math.cos(a), 4)


def stick_windows(sc, min_stick: float = 0.7, min_dur: float = 1.5, dt: float = 1.0 / 30) -> list[tuple]:
    """(t0, t1) scenario-relative windows where |left stick| >= min_stick for at least min_dur seconds."""
    out, start, last = [], None, None
    for _, fr in sc.sample(dt):
        on = math.hypot(*fr.ls) >= min_stick - 1e-9
        if on and start is None:
            start = fr.t
        if not on and start is not None:
            if last - start >= min_dur:
                out.append((start, last))
            start = None
        last = fr.t
    if start is not None and last - start >= min_dur:
        out.append((start, last))
    return out


def wheel_suspect(sc, run_steps: list, lines: list, min_stick: float = 0.7, min_dur: float = 1.5,
                  max_speed: float = 0.3, accel: float = 0.5) -> bool | None:
    """Weapon wheel / menu suspected: a >= min_dur window of |stick| >= min_stick in which every MMTRACE sample
    (after `accel` s) shows |v| < max_speed. None = nothing commanded or no samples (cannot tell)."""
    starts = [s['start'] for s in run_steps if s.get('start')]
    if not starts:
        return None
    t_start = starts[0]
    seen = False
    for a, b in stick_windows(sc, min_stick, min_dur):
        w0, w1 = t_start + timedelta(seconds=a + accel), t_start + timedelta(seconds=b)
        sel = [l for l in lines if l.is_trace and w0 <= l.ts <= w1 and l.num('|v|') is not None]
        if not sel:
            continue
        seen = True
        if all(l.num('|v|') < max_speed for l in sel):
            return True
    return False if seen else None


CROUCH_PAD_BUTTONS = ('R3', 'RS')   # InputSchemesMode=Alternative crouch toggle = R3 tap (verified 2026-10-23)


def crouch_toggle_steps(sc, min_hold: float = 0.3) -> list[int]:
    """Indices of pad crouch-toggle steps: metrics off, only crouch-toggle button events: R3/RS (Alternative scheme
    tap, current scenarios) or the legacy default-scheme B held >= min_hold."""
    idx = []
    for i, st in enumerate(sc.steps):
        if st.buttons and not st.metrics and all(b.button in CROUCH_PAD_BUTTONS or (b.button == 'B' and b.hold >= min_hold)
                                                 for b in st.buttons):
            idx.append(i)
    return idx


def stance_of(line) -> str | None:
    """'crouched' | 'standing' | None from one MMTRACE line (loco=2 crouch, loco=1 stand; clip name as backup)."""
    if not line.is_trace:
        return None
    loco = line.num('loco')
    clip = (line.fields.get('clip') or '').lower()
    if loco == 2 or (loco in (None, 0) and 'crouch' in clip):
        return 'crouched'
    if loco == 1 or (loco == 0 and clip and 'crouch' not in clip):
        return 'standing'
    return None


def stance_seen(lines: list, want: str) -> bool:
    return any(stance_of(l) == want for l in lines)


def is_metric_fail(text: str) -> bool:
    """'footlock_corr_max_cm=17.6 > 15.0' = gameplay metric; anything else (custom threshold message, macro or
    verification text) = test mechanics."""
    return bool(METRIC_FAIL_RE.match(text.strip()))


def split_failures(harness_steps: list, scenario_records: list) -> tuple[list, list]:
    """(mechanics, metrics) failure lists for the night summary."""
    mech, metr = [], []
    for s in harness_steps or []:
        for f in s.get('fails') or []:
            (metr if is_metric_fail(f) else mech).append(f'{s["scenario"]}#{s["run"]}.{s["step"]}: {f}')
    for r in scenario_records or []:
        for f in r.get('fails') or []:
            mech.append(f'{r["name"]}#{r["run"]}: {f}')
    return mech, metr


# ---------------------------------------------------------------------------------------------- round 4: steering

# Default speeds for dead reckoning (m/s; cycle 3: a backwards walk averaged ~0.3 m/s including pauses, so DR is only
# used for <= 2 s and every real POS replaces it). Measured speeds update these per class (night.py).
SPEEDS = {'forward': 3.8, 'sprint': 8.0, 'side': 1.5, 'back': 1.5}
# Right-stick yaw rate at full deflection (camera_whip_180 cycle 3: cam 113 -> 42 deg in a 0.6 s pulse ~ 120 deg/s);
# night.py adapts it from the cf feedback of each pulse.
CAMERA_RATE_DEG_S = 120.0


def bearing_deg(player_xy, cam_fwd_xy, target_xy) -> float:
    """Signed angle of the target relative to the camera forward (deg, + = right/clockwise, -180..180)."""
    dx, dy = target_xy[0] - player_xy[0], target_xy[1] - player_xy[1]
    fx, fy = cam_fwd_xy
    rx, ry = fy, -fx
    return math.degrees(math.atan2(dx * rx + dy * ry, dx * fx + dy * fy))


def turned_deg(cf_old, cf_new) -> float:
    """Signed camera yaw change from cf_old to cf_new (deg, + = turned right/clockwise)."""
    return bearing_deg((0.0, 0.0), cf_old, cf_new)


def stick_class(sx: float, sy: float, sprint: bool = False) -> str:
    """'forward' (|angle| <= 35), 'back' (>= 145), else 'side'; 'sprint' when forward and sprint toggled."""
    a = abs(math.degrees(math.atan2(sx, sy)))
    if a <= 35.0:
        return 'sprint' if sprint else 'forward'
    if a >= 145.0:
        return 'back'
    return 'side'


def speed_for(sx: float, sy: float, sprint: bool = False, speeds: dict | None = None) -> float:
    sp = speeds or SPEEDS
    return sp[stick_class(sx, sy, sprint)] * min(1.0, math.hypot(sx, sy) / 0.9)


def approach_timebox(dist: float, minimum: float = 25.0) -> float:
    return max(minimum, dist / 1.5 + 15.0)


def camera_pulse(bearing: float, rate_deg_s: float = CAMERA_RATE_DEG_S, sign: float = 1.0,
                 min_s: float = 0.06, max_s: float = 0.6) -> tuple[float, float]:
    """(right stick x, seconds) turning the camera towards a target `bearing` deg off; sign = -1 if the game turns
    the other way. Aims at ~80 % of the error to avoid overshoot."""
    dur = min(max_s, max(min_s, abs(bearing) * 0.8 / max(rate_deg_s, 1.0)))
    return (1.0 if bearing > 0 else -1.0) * sign, round(dur, 3)


CAM_PITCH_RE = re.compile(r'\[utpp_camera\] camTrace .*? pitch=(-?[\d.]+) ')


def latest_cam_pitch(text: str) -> float | None:
    """Camera pitch (deg, negative = looking down) of the newest utpp_camera trace line, None without one."""
    m = None
    for m in CAM_PITCH_RE.finditer(text):
        pass
    return float(m.group(1)) if m else None


def pitch_pulse(pitch: float, rate_deg_s: float = 60.0, sign: float = 1.0, stick: float = 0.5,
                min_s: float = 0.08, max_s: float = 1.2) -> tuple[float, float]:
    """(right stick y, seconds) bringing a camera `pitch` deg off level back towards 0 at half stick; rate is the
    pitch speed at full stick, sign = -1 if stick up looks down. Aims at ~80 % of the error."""
    dur = min(max_s, max(min_s, abs(pitch) * 0.8 / max(rate_deg_s * stick, 1.0)))
    return (stick if pitch < 0 else -stick) * sign, round(dur, 3)


# ---------------------------------------------------------------------------------------------- round 5: vehicle exit

RIDE_EXIT = 'keepTpp ride -> exit'                       # [utpp_vehiclecam] exit started (cycle 4: never with B)
EXIT_DONE = ('exit pull: end', 'keepTpp exit -> off')    # exit finished
PHASE_RE = re.compile(r'\[utpp_vehiclecam\](?: t=[\d.]+)? keepTpp (\w+) -> (\w+)')
MAX_VEHICLE_DIST = 60.0  # cycle 4: a POS 2629 m away (garage/summon default) is not a target


def vehiclecam_phase(text: str) -> str | None:
    """Last utpp_vehiclecam keepTpp phase in the text ('off' | 'mount' | 'ride' | 'exit'), None if no line."""
    last = None
    for m in PHASE_RE.finditer(text):
        last = m.group(2)
    return last


def mounted_state(pos: dict | None, log_text: str) -> bool | None:
    """V in a vehicle? NightPos m= first, then the last vehiclecam phase (mount/ride = mounted)."""
    if pos and pos.get('mounted') is not None:
        return pos['mounted']
    ph = vehiclecam_phase(log_text)
    if ph is None:
        return None
    return ph in ('mount', 'ride')


def exit_done(text: str) -> bool:
    return any(p in text for p in EXIT_DONE)


def vehicle_in_range(pos: dict | None, max_dist: float = MAX_VEHICLE_DIST) -> bool:
    if not pos or not pos.get('v'):
        return False
    return math.hypot(pos['v'][0] - pos['p'][0], pos['v'][1] - pos['p'][1]) <= max_dist


# ---------------------------------------------------------------------------------------------- round 8: overlay state

# CET overlay = toolbar + dark Console panel. Regions as fractions of the game client area (x0, x1, y0, y1):
# the panel box lies inside the Console window in every observed frame (cycle 5 frames, 1920x1080 window); the
# reference band is lower scenery. Measured (320x180 frames): open panel mean 40-48, dark fraction 0.48-0.61;
# closed panel mean 146, dark 0.00; bottom 95-119 in both.
OVERLAY_PANEL = (0.12, 0.45, 0.10, 0.35)
OVERLAY_REF = (0.10, 0.90, 0.70, 0.95)
DARK_LUMA = 40


def region_stats(gray: bytes, w: int, h: int, box) -> tuple[float, float]:
    """(mean luma, fraction of pixels < DARK_LUMA) of a fractional box of an 8-bit gray image (row-major)."""
    x0, x1, y0, y1 = int(box[0] * w), max(int(box[1] * w), int(box[0] * w) + 1), int(box[2] * h), \
        max(int(box[3] * h), int(box[2] * h) + 1)
    n = dark = total = 0
    for y in range(y0, y1):
        row = gray[y * w + x0:y * w + x1]
        total += sum(row)
        dark += sum(1 for v in row if v < DARK_LUMA)
        n += len(row)
    return (total / n, dark / n) if n else (0.0, 0.0)


def classify_overlay(gray: bytes, w: int, h: int) -> tuple[str | None, dict]:
    """'open' | 'closed' | None (cannot tell, e.g. a dark night scene) + the measured stats."""
    pm, pd = region_stats(gray, w, h, OVERLAY_PANEL)
    rm, _ = region_stats(gray, w, h, OVERLAY_REF)
    st = {'panel_mean': round(pm, 1), 'panel_dark': round(pd, 2), 'ref_mean': round(rm, 1)}
    if pm < 70 and pd >= 0.35 and rm > 50:
        return 'open', st
    if pm > 85 and pd < 0.10:
        return 'closed', st
    return None, st


def overlay_toggle_effect(before: dict, after: dict, expected: str, min_change: float = 15.0) -> str:
    """After one toggle: 'fixed' (the panel region changed in the expected direction), 'wrong' (changed the other
    way: the classifier was wrong, undo), 'none' (no visible change: the key was probably lost, retry)."""
    d = after['panel_mean'] - before['panel_mean']
    if abs(d) < min_change:
        return 'none'
    if (expected == 'closed' and d > 0) or (expected == 'open' and d < 0):
        return 'fixed'
    return 'wrong'


def latest_pos(text: str, now: datetime, max_age: float = 1.5) -> dict | None:
    """Newest '[night] POS p=' line of the log text if it is at most max_age s old (NightAuto lines)."""
    last = None
    for line in text.splitlines():
        if '[night] POS p=' in line:
            last = line
    if last is None:
        return None
    m = LOGTS_RE.match(last)
    if not m:
        return None
    ts = datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
    if (now - ts).total_seconds() > max_age:
        return None
    pos = parse_pos(last)
    if pos is not None:
        pos['ts'] = ts
        n = re.search(r' n=(\d+)', last)
        pos['n'] = int(n.group(1)) if n else None
    return pos


# ---------------------------------------------------------------------------------------------- round 10: car frame

# Car frame: lon along the car forward, lat positive to the car's LEFT (driver side: V sits on the left).
# POS gives the car's world RIGHT r=(rx,ry); forward f=(-ry, rx) (right = (fy, -fx)), left = -r.
DOOR_LON, DOOR_LAT = 0.35, 1.15     # coupe driver door: slightly ahead of the centre, 1.15 m left of it
END_LON, SIDE_LAT, CLEAR_LAT = 3.2, 2.2, 1.4


def car_axes(vr) -> tuple[tuple[float, float], tuple[float, float]]:
    rx, ry = vr
    n = math.hypot(rx, ry) or 1.0
    rx, ry = rx / n, ry / n
    return (-ry, rx), (-rx, -ry)          # forward, left


def car_frame(xy, v_xy, vr) -> tuple[float, float]:
    """World point -> (lon, lat) in the car frame (lat > 0 = left/driver side)."""
    f, l = car_axes(vr)
    dx, dy = xy[0] - v_xy[0], xy[1] - v_xy[1]
    return dx * f[0] + dy * f[1], dx * l[0] + dy * l[1]


def from_car_frame(lon: float, lat: float, v_xy, vr) -> tuple[float, float]:
    f, l = car_axes(vr)
    return v_xy[0] + f[0] * lon + l[0] * lat, v_xy[1] + f[1] * lon + l[1] * lat


def door_target(pos: dict, lat: float = DOOR_LAT, lon: float = DOOR_LON) -> tuple[float, float] | None:
    if not pos or not pos.get('v') or not pos.get('vr'):
        return None
    return from_car_frame(lon, lat, pos['v'][:2], pos['vr'])


def plan_path(lon: float, lat: float, door=(DOOR_LON, DOOR_LAT), end: float = END_LON, side: float = SIDE_LAT,
              clear: float = CLEAR_LAT, skip: float = 0.7, blocked_end: float = 0.0) -> list[tuple[float, float]]:
    """Car-frame waypoints from V (lon, lat) to the door, never cutting through the body.

    - wrong side (lat <= 0.3) or at an end (|lon| > 2.0 and |lat| < 1.6): around the nearer end -
      (end, own side) -> (end, door side `side`) -> along the door side -> door;
    - door side but hugging the body (lat < clear) away from the door (|lon - door lon| > 1.2): out to `side`, then
      along the side to within 1.2 m of the door lon, then the door;
    - otherwise straight to the door. Waypoints within `skip` m of V are dropped.
    `blocked_end` (-1 / +1, 0 = none): the car end where V stood still (night.approach stuck) - go round the other end
    (2026-11-12 04:16: V pushed 25 s at the rear corner and the re-plans chose the same end again).
    """
    dl, dlat = door
    wps: list = []
    wrong_side = lat <= 0.3
    at_end = abs(lon) > 2.0 and abs(lat) < 1.6
    if wrong_side or at_end:
        e = -end if lon <= dl else end
        flipped = bool(blocked_end) and math.copysign(1.0, e) == math.copysign(1.0, blocked_end)
        if flipped:
            e = -e
        # going to the far end along the door side must stay outside the body (lat 0.3-1.6 is inside it)
        own = max(lat, side) if flipped and at_end and not wrong_side else lat if at_end and not wrong_side else (math.copysign(max(abs(lat), side), lat) if abs(lat) > 0.3
                                                      else 0.0)
        wps += [(e, own), (e, side)]
        approach_lon = dl - 1.2 if e < 0 else dl + 1.2
        wps.append((approach_lon, side))
    elif lat < clear and abs(lon - dl) > 1.2:
        approach_lon = dl - 1.2 if lon < dl else dl + 1.2
        wps += [(lon, side), (approach_lon, side)]
    wps.append((dl, dlat))
    while len(wps) > 1 and math.hypot(wps[0][0] - lon, wps[0][1] - lat) < skip:
        wps.pop(0)
    return wps




# Camera aim before X: a point on the car this far ahead of V along the car and this far left of its centre line, so
# the camera looks along the car and the screen centre lies on the door instead of on V (2026-11-12 10:29:54).
X_AIM_AHEAD_M = 2.0
X_AIM_LAT_M = 0.6
# Where the enter prompt is along the car depends on the car: Female V's coupe mounts at lon 0.1-0.5, the Quadra of the
# Male V save showed the prompt at lon -0.79 lat 1.85 (2026-11-12 10:29:54) and none at lon 0.1-0.35 pressed against
# the body (lat 1.53-1.59). After a failed X the harness therefore walks along the car to these car-frame lon values,
# in this order, standing at least X_SEEK_LAT_M from the centre line (about 0.3 m off the body).
X_SEEK_LONS = (-0.8, -1.5, 0.3)
# Where to stand for the first X, per V body (the two test saves have different cars). Car frame (lon, lat); the
# approach stops within 0.8 m of it. woman: coupe, pad-X mounts logged at lon 0.12-0.52 lat 0.96-1.30. man: Quadra,
# X worked at lon -0.17 lat 1.56 (2026-11-12 11:14:40) and the prompt was seen at lon -0.79 lat 1.85; it never
# worked at lon 0.10-0.36. A spot where X worked is remembered in MOUNT_SPOTS_FILE and used on the next run.
DOOR_BY_BODY = {'woman': (0.35, 1.15), 'man': (-0.3, 1.6)}


def body_in_log(text: str) -> str | None:
    """'woman' / 'man' from the last 'body=' of the UTPP log (motion status / driver lines)."""
    m = re.findall(r'\bbody=(woman|man)\b', text or '')
    return m[-1] if m else None


def load_mount_spots(path: str) -> dict:
    try:
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        return {k: (float(v[0]), float(v[1])) for k, v in d.items() if k in DOOR_BY_BODY and len(v) == 2}
    except (OSError, ValueError, TypeError, IndexError):
        return {}


def save_mount_spot(path: str, body: str, lon: float, lat: float) -> None:
    d = {k: list(v) for k, v in load_mount_spots(path).items()}
    d[body] = [round(lon, 2), round(lat, 2)]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + '.tmp', 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=1)
    os.replace(path + '.tmp', path)
X_SEEK_LAT_M = 1.85
X_SEEK_TOL_M = 0.25




def parked_far(st: dict, hold: float, max_dist: float) -> bool:
    """arrival_state result of a car that really drove in (> 1 m, >= 2 driving segments), has stood still >= hold s
    but parked beyond the arrival distance (state 'waiting') and within max_dist m."""
    return (st.get('state') == 'waiting' and (st.get('segments') or 0) >= 2 and (st.get('moved') or 0) > 1.0
            and (st.get('still') or 0) >= hold and st.get('dist') is not None and st['dist'] <= max_dist)


def at_door(lon: float, lat: float, door=(DOOR_LON, DOOR_LAT), radius: float = 0.8,
            lat_range=(0.9, 1.7)) -> bool:
    return math.hypot(lon - door[0], lat - door[1]) <= radius and lat_range[0] <= lat <= lat_range[1]


# ---------------------------------------------------------------------------------------------- round 11: car speed

def stationary_duration(samples: list, thr: float = 0.2) -> float:
    """samples [(t_s, (x, y))] ascending: seconds the vehicle has been slower than thr m/s up to the last sample."""
    if len(samples) < 2:
        return 0.0
    start = samples[-1][0]
    for (t0, p0), (t1, p1) in reversed(list(zip(samples, samples[1:]))):
        dt = t1 - t0
        if dt <= 0:
            continue
        if math.dist(p0, p1) / dt >= thr:
            break
        start = t0
    return samples[-1][0] - start


# ---------------------------------------------------------------------------------------------- round 12: arrival

def arrival_state(samples: list, player_xy, max_dist: float = 15.0, stop_speed: float = 0.2, hold: float = 3.0,
                  min_moved: float = 1.0, min_drive_segments: int = 2, stale_after: float = 10.0) -> dict:
    """State of a vehicle that should drive in (V-hold / summon). samples [(t_s, (x, y))] since first seen.

    'arrived': it moved > min_moved m with >= min_drive_segments driving segments (0.5-40 m/s; a single jump of a
    re-assigned entity id does not count), has been still (< stop_speed) for >= hold s and is <= max_dist m from V;
    'stale': no movement > min_moved for >= stale_after s (cycle 12: a phantom SummonedVehicleEntityID position
    in an empty desert); 'driving'; 'waiting'; 'none' (no sample).
    """
    if not samples:
        return {'state': 'none', 'moved': 0.0, 'dist': None, 'still': 0.0, 'segments': 0}
    first = samples[0][1]
    moved = max(math.dist(first, p) for _, p in samples)
    segs = 0
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt > 0 and 0.5 <= math.dist(p0, p1) / dt <= 40.0:
            segs += 1
    still = stationary_duration(samples, stop_speed)
    d = math.dist(samples[-1][1], player_xy) if player_xy else None
    span = samples[-1][0] - samples[0][0]
    out = {'moved': round(moved, 2), 'dist': None if d is None else round(d, 1), 'still': round(still, 1),
           'segments': segs}
    if moved > min_moved and segs >= min_drive_segments and still >= hold and d is not None and d <= max_dist:
        out['state'] = 'arrived'
    elif moved <= min_moved and span >= stale_after:
        out['state'] = 'stale'
    elif segs and still < hold:
        out['state'] = 'driving'
    else:
        out['state'] = 'waiting'
    return out


# ---------------------------------------------------------------------------------------------- route look-ahead / blocked walk

# 2026-11-12: V stood in a bush for three steps (20261111-173219) and walked into an obstacle (20261112-040756); the
# 0.6 s probe only sees the first 0.3 m. A route longer than ROUTE_LONG_M is probed over the main leg first
# (route_estimate / lookahead_plan); a finished walk is checked from the NightAuto POS lines (blocked_report).
ROUTE_LONG_M = 6.0           # net displacement or longest straight leg above this = long route (long look-ahead);
                             # shorter routes keep the 0.6 s probe (weapon_walk 8.4 m is long, melee_punch_walk 5.3 m is not)
LOOKAHEAD_MAX_S = 4.0        # the outbound look-ahead walk never lasts longer
LOOKAHEAD_TOTAL_S = 9.0      # outbound + walk back (a walk back with the reversed stick is slower than the way out)
LOOKAHEAD_MIN_S = 0.6
LOOKAHEAD_STICK = 0.8        # stick magnitude of the look-ahead walk (and of the walk back)
LOOKAHEAD_STALL_S = 1.0      # look-ahead: a stall of this length while the stick is pushed = blocked
LOOKAHEAD_BACK_TOL_M = 1.0   # residual after the walk back above this -> one correction pass
LEG_TURN_DEG = 30.0          # a heading change above this starts a new straight leg
STALL_STICK_MIN = 0.2        # stick pushed (same as gt_log.PUSHED_STICK)
STALL_SPEED_MAX = 0.3        # m/s between two POS samples (same as run.BLOCKED_SPEED_MAX)
BLOCK_MIN_S = 1.5            # scenario: pushed and slower than STALL_SPEED_MAX for this long = blocked
STALL_ACCEL_S = 0.5          # the first seconds after a push starts are acceleration, not a stall
_L3_MASK = 0x0040


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def route_estimate(sc, speeds: dict | None = None, cam_rate: float = CAMERA_RATE_DEG_S, cam_sign: float = 1.0,
                   dt: float = 1.0 / 30, min_stick: float = STALL_STICK_MIN) -> dict:
    """Estimated route of a scenario from its stick tracks, in the camera frame at the scenario start (x right,
    y forward, m). The left stick is camera-relative, so the right stick (camera yaw rate cam_rate deg/s per unit,
    sign cam_sign) rotates it; the speed per sample is speed_for(stick, sprint, speeds) and an L3 press toggles
    sprint (released stick = sprint off). A leg is a run of samples whose heading stays within LEG_TURN_DEG of the
    leg so far. Returns {'net': (x, y), 'net_m', 'path_m', 'leg_m', 'leg_s', 'leg_heading_deg' (+ = right of the
    camera forward), 'leg_start': (x, y)}; zeros when the stick never moves V."""
    yaw, sprint, prev_l3 = 0.0, False, False
    x = y = path = 0.0
    legs: list = []
    cur = None
    for _, fr in sc.sample(dt):
        l3 = bool(fr.buttons & _L3_MASK)
        if l3 and not prev_l3:
            sprint = not sprint
        prev_l3 = l3
        yaw += fr.rs[0] * cam_rate * cam_sign * dt
        mag = math.hypot(*fr.ls)
        if mag < min_stick:
            sprint, cur = False, None
            continue
        sx, sy = rotate_stick(fr.ls[0], fr.ls[1], yaw)
        n = math.hypot(sx, sy) or 1.0
        ux, uy = sx / n, sy / n
        step = speed_for(fr.ls[0], fr.ls[1], sprint, speeds) * dt
        heading = math.degrees(math.atan2(ux, uy))
        if cur is not None and abs(_wrap_deg(heading - math.degrees(math.atan2(cur['vx'], cur['vy'])))) > LEG_TURN_DEG:
            cur = None
        if cur is None:
            cur = {'vx': 0.0, 'vy': 0.0, 'len': 0.0, 'dur': 0.0, 'start': (x, y)}
            legs.append(cur)
        cur['vx'] += ux * step
        cur['vy'] += uy * step
        cur['len'] += step
        cur['dur'] += dt
        x += ux * step
        y += uy * step
        path += step
    out = {'net': (round(x, 2), round(y, 2)), 'net_m': round(math.hypot(x, y), 2), 'path_m': round(path, 2),
           'leg_m': 0.0, 'leg_s': 0.0, 'leg_heading_deg': 0.0, 'leg_start': (0.0, 0.0)}
    if legs:
        leg = max(legs, key=lambda l: l['len'])
        out.update(leg_m=round(leg['len'], 2), leg_s=round(leg['dur'], 2),
                   leg_heading_deg=round(math.degrees(math.atan2(leg['vx'], leg['vy'])), 1),
                   leg_start=(round(leg['start'][0], 2), round(leg['start'][1], 2)))
    return out


def lookahead_plan(est: dict, speeds: dict | None = None, long_m: float = ROUTE_LONG_M,
                   cap_s: float = LOOKAHEAD_MAX_S, stick: float = LOOKAHEAD_STICK) -> dict | None:
    """None for a short route (the short probe is enough). Else the look-ahead walk along the longest leg:
    {'heading_deg', 'stick': (x, y), 'duration' (s, <= cap_s and out + back <= LOOKAHEAD_TOTAL_S), 'reach_m'}; the
    stick is camera-relative, so the camera is not turned. Sprint legs are probed at run speed (no L3 toggle: the look-ahead changes no state)."""
    if max(est['net_m'], est['leg_m']) <= long_m:
        return None
    h = math.radians(est['leg_heading_deg'])
    sx, sy = round(math.sin(h) * stick, 4), round(math.cos(h) * stick, 4)
    spd = max(speed_for(sx, sy, False, speeds), 0.5)
    back = max(speed_for(-sx, -sy, False, speeds), 0.5)
    dur = round(min(cap_s, LOOKAHEAD_TOTAL_S / (1.0 + spd / back), max(LOOKAHEAD_MIN_S, est['leg_m'] / spd)), 2)
    return {'heading_deg': est['leg_heading_deg'], 'stick': (sx, sy), 'duration': dur,
            'reach_m': round(min(est['leg_m'], spd * dur), 1)}


def pos_samples(text: str) -> list:
    """[(datetime, (x, y))] of every '[night] POS p=' line (NightAuto) in the log text, in file order."""
    out = []
    for line in text.splitlines():
        if '[night] POS p=' not in line:
            continue
        m, pm = LOGTS_RE.match(line), POS_RE.search(line)
        if m and pm:
            out.append((datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f'),
                        (float(pm.group(1)), float(pm.group(2)))))
    return out


def pushed_intervals(sc, steps: list, min_stick: float = STALL_STICK_MIN, dt: float = 1.0 / 30,
                     max_gap_s: float = 0.25) -> list:
    """[(start, end)] wall-clock windows in which the left stick of the played scenario was >= min_stick. Each step
    maps through its own recorded 'start' (steps[i], aligned with sc.steps; the key-crouch path inserts real-time
    toggle steps), so a gap between steps splits a window; steps that never started are skipped."""
    bounds = [b[1] for b in sc.step_bounds()]
    out: list = []
    cur = None
    for i, fr in sc.sample(dt):
        st = steps[i] if i < len(steps) else None
        if not st or not st.get('start'):
            cur = None
            continue
        t = st['start'] + timedelta(seconds=fr.t - bounds[i])
        if math.hypot(*fr.ls) >= min_stick - 1e-9:
            if cur is not None and (t - cur[1]).total_seconds() <= max_gap_s:
                cur[1] = t
            else:
                cur = [t, t]
                out.append(cur)
        else:
            cur = None
    return [(a, b) for a, b in out]


def blocked_report(samples: list, intervals: list, min_s: float = BLOCK_MIN_S, speed_max: float = STALL_SPEED_MAX,
                   accel_s: float = STALL_ACCEL_S, tol_s: float = 0.1) -> dict:
    """Did V stall while the stick was pushed? Between two POS samples the speed is |dp| / dt; a pair counts only
    when both samples lie inside one pushed window, accel_s after its start. {'blocked': stall >= min_s, 'stall_s'
    (longest run of slow pairs), 'at' (sample time where the longest stall ended), 'samples'}."""
    best, best_at, start = 0.0, None, None
    for (ta, pa), (tb, pb) in zip(samples, samples[1:]):
        span = (tb - ta).total_seconds()
        pushed = span > 0 and any(a + timedelta(seconds=accel_s - tol_s) <= ta and tb <= b + timedelta(seconds=tol_s)
                                  for a, b in intervals)
        if pushed and math.dist(pa, pb) / span < speed_max:
            start = start or ta
            stall = (tb - start).total_seconds()
            if stall > best:
                best, best_at = stall, tb
        else:
            start = None
    return {'blocked': best >= min_s, 'stall_s': round(best, 2), 'at': best_at, 'samples': len(samples)}

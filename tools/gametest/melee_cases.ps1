<#
.SYNOPSIS
Melee (fists) test cases for the UTPP melee full body + melee aim tilt (2026-11-09).
.DESCRIPTION
Sibling of bike_cases.ps1. Launches the game (unless -NoLaunch), leaves the vehicle when the save loads with V mounted (bike_exit_only +
bike_walk_away of scenarios_bike_cases.json; checked by a '[melee] mounted=' line), switches the MM trace on, runs the fists scenarios
of scenarios.json in recorded groups and saves the melee log lines (MMMELEEFULL, MMGUARD, MMWEAPON, MELEEAIM, MMMASK, '[melee]',
UTPPSetMelee*) to <Out>\melee-lines.txt. Recordings go straight to Media\mp4\YYYY-MM-DD_gametest_<run> (standing project rule); logs stay in <Out>.
Nothing here is a pass/fail verdict of the animation: the summary counts full-body switches, the why= reasons and the aim tilt lines.
Needs ViGEmBus, the game in the FOREGROUND (run.py aborts when focus is lost), CET and the UTPP log. -DryRun forwards --dry-run to run.py.

CET console commands are VERIFIED (2026-11-09; run melee-cases-20261109-161452 lost 4 of ~38 typed commands without notice): every command
ends with Game.UTPPLogLine('[cetv:<unique id>]...'), run.py waits for that marker in the UTPP log, types the command once more when it is
missing (input field cleared first) and aborts with exit 2 when it is still missing. A preparation step that is not confirmed (fists draw,
weapon draw + INHAND, gun prime + INHAND, guard settings) stops its group: the group is listed under "INVALIDATED" in the summary
(melee-cases-summary.txt) and the script exits 1. WEAPON_RM reports the real inventory count before/after the removal.

Groups (-Groups; default 'all' = the first five in this order; guard only when named):
  basic   melee_draw, melee_punch, melee_block                              (fists raised by the RT tap, punches, block)
  aim     melee_punch_look_up, melee_punch_look_down, melee_block_look_down  (aim tilt: MELEEAIM pitch > 0 / < 0)
  walk    melee_punch_then_walk, melee_holster                              (full body -> upper-body mask on walking, fists lowered)
  weapons per weapon of -Weapons: melee_punch, melee_block                  (the weapon is given + drawn by script with a DrawItemRequest,
          so the loadout slots stay as they are; an '[melee] INHAND <tag> <friendlyName>' line proves what is in the hand; afterwards it
          is holstered and removed again when the script gave it; hammer = control: w_melee_hammer has no UTPP set, full body stays off)
  gun     weapon_draw, weapon_aim, weapon_holster                          (sanity: a gun after melee looks unchanged; the ranged weapon
          is drawn and holstered by script first, so the Y tap of weapon_draw takes the gun and not the last melee weapon)
  guard   melee_draw, melee_punch, melee_punch_look_up, melee_punch_then_walk, melee_holster   (2026-11-09 round 3, NOT part of 'all',
          not run yet: melee.guard + melee.guardAimTilt switched on for the group only ({noSave=true}; -GuardLegs adds melee.guardFullBody),
          the previous values restored afterwards; expect 'MMGUARD on why=on' after the draw, no MMGUARD off between the punches, the
          lean through the guard (MELEEAIM), 'MMGUARD off why=state' or why=gate after the holster; walking keeps the upper body only)

Run (the game window must be in the foreground):
  powershell -File melee_cases.ps1 -DryRun
  powershell -File melee_cases.ps1                            launches the game, all groups, closes the game at the end (-KeepGame keeps it open)
  powershell -File melee_cases.ps1 -Groups aim -NoLaunch      game already in the world, V on foot
  powershell -File melee_cases.ps1 -Groups weapons -Weapons katana,hammer
#>
[CmdletBinding()]
param(
    [string[]]$Groups = @('all'),
    [string[]]$Weapons = @('all'),
    [switch]$DryRun,
    [switch]$NoLaunch,
    [switch]$NoRecord,
    [switch]$KeepGame,
    # guard group only: also switch on melee.guardFullBody (stage 2, the legs in the guard)
    [switch]$GuardLegs,
    [string]$Out = '',
    [string]$Log = '',
    [string]$Python = 'python',
    [string]$Game = '',
    # minimum pause before any call that types console commands (seconds since the previous run.py call ended; see car_cases.ps1)
    [double]$CetGapS = 6
)

$ErrorActionPreference = 'Stop'
$null = Get-Command $Python -ErrorAction Stop
$gt = $PSScriptRoot
. (Join-Path $gt '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$scenFile = Join-Path $gt 'scenarios.json'
$bikeScenFile = Join-Path $gt 'scenarios_bike_cases.json'
$navPy = Join-Path $gt 'bike_cases_nav.py'
$root = $null
for ($p = $gt; $p; $p = Split-Path -Parent $p) { if (Test-Path (Join-Path $p 'Tools\pylibs')) { $root = $p; break } }
if (-not $root) { throw 'project root (folder with Tools\pylibs) not found above ' + $gt }
$ffmpeg = Join-Path $root 'Tools\pylibs\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe'
if (-not $Out) { $Out = Join-Path $root ('Research\logs\gametest\melee-cases-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
$MediaOut = Join-Path $root ('Media\mp4\' + (Get-Date -Format 'yyyy-MM-dd') + '_gametest_' + (Split-Path $Out -Leaf))
if (-not $DryRun) { New-Item -ItemType Directory -Force $Out | Out-Null }

$groupTable = [ordered]@{
    basic = @('melee_draw', 'melee_punch', 'melee_block')
    aim   = @('melee_punch_look_up', 'melee_punch_look_down', 'melee_block_look_down')
    walk  = @('melee_punch_then_walk', 'melee_holster')
    weapons = @('melee_punch', 'melee_block')
    gun   = @('weapon_draw', 'weapon_aim', 'weapon_holster')
    guard = @('melee_draw', 'melee_punch', 'melee_punch_look_up', 'melee_punch_then_walk', 'melee_holster')
}
# weapon tag -> TweakDB record (friendlyName = the anim wrapper variable of its melee set, see the export report 11.5)
$weaponTable = [ordered]@{
    katana  = 'Items.Preset_Katana_Default'          # w_melee_katana
    crowbar = 'Items.Preset_Crowbar_Default'         # w_melee_one_hand_blunt (Wea_OneHandedClub)
    machete = 'Items.Preset_Machete_Default'         # w_melee_one_hand_blunt (Wea_LongBlade)
    bat     = 'Items.Preset_Baseball_Bat_Default'    # w_two_hand_blunt
    knife   = 'Items.Preset_Knife_Default'           # w_melee_knife
    hammer  = 'Items.Preset_Hammer_Default'          # w_melee_hammer: no UTPP set -> control, full body must stay off
}
# what the INHAND line must show per weapon (rec:FriendlyName() of the drawn item; machete/crowbar share one set)
$weaponFriendly = [ordered]@{
    katana = 'w_melee_katana'; crowbar = 'w_melee_one_hand_blunt'; machete = 'w_melee_one_hand_blunt'
    bat = 'w_two_hand_blunt'; knife = 'w_melee_knife'; hammer = 'w_melee_hammer'
}
$lineRx = 'MMMELEEFULL|MMGUARD|MMWEAPON|MELEEAIM|MMMASK|\[melee\]|UTPPSetMelee|meleefull'
$script:clip = 0
$script:lastRunEnd = $null
$script:cetRetried = 0          # commands that needed a second typing (run.py prints '(typed 2x)')
$script:invalid = [ordered]@{}  # group (or 'weapons/<tag>') -> why its result is not valid
$script:soft = @()              # lost status lines / failed cleanups: warnings that do not invalidate a group

function Read-Shared([string]$path) {
    $s = [IO.File]::Open($path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]"ReadWrite, Delete")
    $r = New-Object IO.StreamReader($s)
    try { return $r.ReadToEnd() } finally { $r.Dispose() }
}
function Invoke-NavJson([string[]]$NavArgs) {
    try { return ((& $Python -I $navPy @NavArgs | Select-Object -Last 1) | ConvertFrom-Json) } catch { Write-Warning "log helper failed: $_"; return $null }
}
function Get-NewestLog {
    if ($Log) { return $Log }
    $r = Invoke-NavJson @('newest', '--dir', (Join-Path $Game 'red4ext\logs'))
    if ($r -and $r.path) { return [string]$r.path }
    return $null
}
function Get-LogMark {
    $lg = Get-NewestLog
    if (-not $lg) { return '' }
    $r = Invoke-NavJson @('logmark', '--log', $lg)
    if ($r -and $r.token) { return [string]$r.token }
    return ''
}
function Find-Log([string]$Mark, [string]$Rx) {
    $lg = Get-NewestLog
    if (-not $lg -or -not $Mark) { return $false }
    $r = Invoke-NavJson @('found', '--log', $lg, '--since', $Mark, '--rx', $Rx)
    return [bool]($r -and $r.found)
}
# A preparation step is only valid when its own log line shows up (the typed command ran to its end AND reported success).
function Assert-Log([string]$Mark, [string]$Rx, [string]$Why) {
    if ($DryRun) { return }
    if (-not (Find-Log $Mark $Rx)) { throw $Why }
}
function Assert-NoLog([string]$Mark, [string]$Rx, [string]$Why) {
    if ($DryRun) { return }
    if (Find-Log $Mark $Rx) { throw $Why }
}
function Set-Invalid([string]$Key, [string]$Why) {
    $s = ($Why -replace '\s+', ' ').Trim()
    if ($s.Length -gt 300) { $s = $s.Substring(0, 300) + '...' }
    $script:invalid[$Key] = $s
    $script:fail++
    Write-Warning "INVALID $Key : $s"
}

function Start-Game {
    $g = Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue
    if ($g) {
        [void]$g.CloseMainWindow()
        $deadline = (Get-Date).AddSeconds(45)
        while ((Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) -and (Get-Date) -lt $deadline) { Start-Sleep -Seconds 2 }
        if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { Stop-Process -Name Cyberpunk2077 -Force; Start-Sleep -Seconds 4 }
    }
    if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'game still running' }
    $before = Get-Date
    $exe = Join-Path $Game 'bin\x64\Cyberpunk2077.exe'
    Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) -ArgumentList '-modded'
    $dir = Join-Path $Game 'red4ext\logs'
    $deadline = (Get-Date).AddSeconds(240)
    $inWorld = $false
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 5
        $u = Get-ChildItem $dir -Filter 'utpp-*.log' | Where-Object { $_.LastWriteTime -gt $before -and $_.Name -notmatch '\.\d+\.log$' } |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($u -and (Read-Shared $u.FullName) -match 'early pose arm: ready=1 rig=\d+ bones=6\d\d') { $inWorld = $true; break }
    }
    if (-not $inWorld) { throw 'world not reached' }
    Start-Sleep -Seconds 8
}

function Get-GameRect {
    $code = 'import sys, ctypes; ctypes.windll.shcore.SetProcessDpiAwareness(2); sys.path.insert(0, sys.argv[1]); import gt_win32 as w; a=w.Win32Api(); h=a.find_main_window(a.list_pids(w.GAME_EXE)); r=a.client_rect_screen(h); print(r[0], r[1], r[2]-r[0], r[3]-r[1])'
    $o = (& $Python -I -c $code $gt) | Select-Object -Last 1
    if ($LASTEXITCODE -ne 0 -or $o -notmatch '^-?\d+ -?\d+ \d+ \d+$') { throw "game window rect not available: $o" }
    return ($o -split ' ')
}

function Get-ScenarioSeconds([string[]]$Names, [string]$File) {
    $d = Get-Content $File -Raw | ConvertFrom-Json
    $sum = 0.0
    foreach ($n in $Names) {
        if (-not $d.scenarios.$n) { throw "scenario $n not in $File" }
        foreach ($st in $d.scenarios.$n.steps) { $sum += [double]$st.duration }
    }
    return $sum + 2.0 * [Math]::Max(0, $Names.Count - 1) + 6.0
}

# One run.py call: -Scenarios pad steps (the pad stays plugged), -Cet console commands typed first (';;' separated); no scenario = a
# CET-only call. run.py exit 1 (no PASS verdict: these scenarios score nothing) is only reported; exit 2 (abort / no log) stops.
function Invoke-Run {
    param([string]$Tag, [string[]]$Scenarios = @(), [string]$Cet = '', [switch]$Record, [string]$ScenarioFile = '', [switch]$NoWake)
    $scn = if ($ScenarioFile) { $ScenarioFile } else { $scenFile }
    $ra = @('run.py', '--out', (Join-Path $Out $Tag), '--keep-pad', '--no-dismiss')
    if ($Scenarios.Count) { $ra += @('--scenario-file', $scn, '--scenarios', ($Scenarios -join ',')) } else { $ra += @('--wake', 'none') }
    if ($NoWake -and $Scenarios.Count) { $ra += @('--wake', 'none') }
    if ($Cet) { $ra += @('--cet', $Cet) }
    if ($DryRun) { $ra += '--dry-run' }
    if ($Log) { $ra += @('--log', $Log) }
    $ErrorActionPreference = 'Continue'
    $ff = $null
    $secs = 0
    if ($Cet -and -not $DryRun -and $script:lastRunEnd) {
        $wait = $CetGapS - ((Get-Date) - $script:lastRunEnd).TotalSeconds
        if ($wait -gt 0) { Start-Sleep -Milliseconds ([int]($wait * 1000)) }
    }
    Push-Location $gt
    try {
        if ($Record -and -not $DryRun -and -not $NoRecord -and $Scenarios.Count) {
            $secs = Get-ScenarioSeconds $Scenarios $scn
            $x, $y, $w, $h = Get-GameRect
            $script:clip++
            if (-not (Test-Path $MediaOut)) { New-Item -ItemType Directory -Path $MediaOut -Force | Out-Null }
            $mp4 = Join-Path $MediaOut ('{0}-{1}.mp4' -f $Tag, $script:clip)
            $fa = "-y -loglevel error -f gdigrab -framerate 30 -offset_x $x -offset_y $y -video_size ${w}x${h} -i desktop -t $([int][Math]::Ceiling($secs)) -vf scale=1280:-2 -c:v libx264 -profile:v main -pix_fmt yuv420p -movflags +faststart -preset veryfast -crf 20 `"$mp4`""
            $ff = Start-Process -FilePath $ffmpeg -ArgumentList $fa -PassThru -WindowStyle Hidden
            Start-Sleep -Seconds 1
        }
        $o = @(& $Python @ra 2>&1 | ForEach-Object { "$_" })
        $code = $LASTEXITCODE
        if ($DryRun) { Write-Host "--- $Tag"; $o | Where-Object { $_ -match '^(CET commands|estimated input time|\S+ +[0-9.]+ s |melee_\w+: )' } | Out-Host }
        else { $o | Select-Object -Last 3 | Out-Host }
        $script:cetRetried += @($o | Where-Object { $_ -match '\(typed [2-9]x\)' }).Count
        if ($code -eq 1) { Write-Warning "run.py $Tag exit 1 (no PASS verdict; these scenarios have no scored metrics)" }
        elseif ($code -ne 0) {
            $o | Select-Object -Last 6 | Out-Host
            # the reason is the '[gametest] ABORT: ...' line (stderr is not buffered like stdout, so it is NOT necessarily the last line of
            # $o); only without it fall back to a '... NOT confirmed' echo line
            $why = @($o | Where-Object { $_ -match '\[gametest\] ABORT' }) | Select-Object -First 1
            if (-not $why) { $why = @($o | Where-Object { $_ -match 'NOT confirmed' }) | Select-Object -First 1 }
            throw ("run.py failed ($code) for $Tag" + $(if ($why) { ': ' + $why } else { '' }))
        }
    }
    finally {
        Pop-Location
        if ($ff) { [void]$ff.WaitForExit(([int]$secs + 20) * 1000) }
        $script:lastRunEnd = Get-Date
    }
}
# Every command carries '@@ID@@' inside its final Game.UTPPLogLine('[cetv:@@ID@@]...'): a fresh id per call, so run.py can verify
# (marker in the UTPP log), retype once and abort (exit 2 -> throw here) when the command did not arrive complete.
function Expand-CetId([string]$Cmd) { $Cmd.Replace('@@ID@@', [guid]::NewGuid().ToString('N').Substring(0, 10)) }
function Invoke-Cet([string]$Tag, [string[]]$Cmds) {
    $exp = @($Cmds | ForEach-Object { Expand-CetId $_ })
    Invoke-Run -Tag $Tag -Cet ($exp -join ';;')
}
# a lost status line does not change what the group measured: warning + summary note, no invalidation
function Status-Line([string]$Tag) {
    try { Invoke-Cet "status-$Tag" @("pcall(function() Game.UTPPLogLine('[cetv:@@ID@@][melee] status $Tag '..tostring(Game.UTPPMeleeStatus())) end)") }
    catch { $script:soft += "status line '$Tag' lost: $($_.Exception.Message)"; Write-Warning "status line '$Tag' lost: $_" }
}
# Fists by script (as gt_night.py FISTS_CMD / FISTS_OFF_CMD): the test save starts with a pistol in the hand, so an RT tap draws
# the pistol (and fires) and LT aims instead of blocking (run melee-cases-20261109-120859: melee=0 in every MMMASK line).
# QuickSlotsManager.RequestEquipFists, falling back to EquipmentSystemWeaponManipulationRequest RequestFists; off = UnequipWeapon.
$fistsCmd = "local ok,e=pcall(function() local p=Game.GetPlayer() local how='quickslots' local done=false pcall(function() p:GetQuickSlotsManager():RequestEquipFists() done=true end) if not done then how='eqsystem' local r=EquipmentSystemWeaponManipulationRequest.new() r.requestType=Enum.new('EquipmentManipulationAction','RequestFists') r.owner=p Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) end return how end) Game.UTPPLogLine('[cetv:@@ID@@][melee] FISTS ok='..tostring(ok)..' '..tostring(e))"
function Get-HolsterCmd([string]$Tag) {
    "local ok,e=pcall(function() local p=Game.GetPlayer() local r=EquipmentSystemWeaponManipulationRequest.new() r.requestType=Enum.new('EquipmentManipulationAction','UnequipWeapon') r.owner=p Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'eqsystem' end) Game.UTPPLogLine('[cetv:@@ID@@][melee] HOLSTER $Tag ok='..tostring(ok)..' '..tostring(e))"
}
# gun group: draw the last used / first available ranged weapon, then holster it, so the Y tap of weapon_draw draws the gun (run
# melee-cases-20261109-122402: after the fists the Y tap drew the fists again and LT blocked instead of aiming)
$gunPrimeCmd = "local ok,e=pcall(function() local p=Game.GetPlayer() local r=EquipmentSystemWeaponManipulationRequest.new() r.requestType=Enum.new('EquipmentManipulationAction','RequestLastUsedOrFirstAvailableRangedWeapon') r.owner=p Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'ranged' end) Game.UTPPLogLine('[cetv:@@ID@@][melee] GUN_PRIME ok='..tostring(ok)..' '..tostring(e))"
# weapons group: give the preset only when V has no item with that ID, draw it with a DrawItemRequest (as the heavy weapons / TakeAndEquip
# items of player.script: no loadout slot changes); the per-tag Lua global UTPP_GIVEN_<tag> remembers what the script gave, for the removal
function Get-WeaponDrawCmd([string]$Tag, [string]$Rec) {
    "local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() local id=ItemID.FromTDBID(TweakDBID.new('$Rec')) local how='had' if not ts:HasItem(p,id) then ts:GiveItem(p,id,1) UTPP_GIVEN_$Tag=true how='given' end local r=DrawItemRequest.new() r.owner=p r.itemID=id Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return how end) Game.UTPPLogLine('[cetv:@@ID@@][melee] WEAPON $Tag ok='..tostring(ok)..' '..tostring(e))"
}
function Get-InHandCmd([string]$Tag) {
    "local ok,e=pcall(function() local p=Game.GetPlayer() local w=Game.GetTransactionSystem():GetItemInSlot(p,TweakDBID.new('AttachmentSlots.WeaponRight')) if not w then return 'none' end local rec=TweakDBInterface.GetItemRecord(w:GetItemID().id) if not rec then return '?' end return tostring(rec:FriendlyName()) end) Game.UTPPLogLine('[cetv:@@ID@@][melee] INHAND $Tag ok='..tostring(ok)..' '..tostring(e)..' | '..tostring(Game.UTPPMeleeStatus()))"
}
# Removal with a real result (2026-11-09: the old command answered 'removed' without looking; the machete/hammer came back into the hand
# 30-40 s later). The item V holds has its own ItemID (seed), the ID built from the TweakDBID may not match it, so the real IDs are
# taken from the inventory list (GetItemList + ItemID.GetTDBID); the count before/after is read from the list (fallback: GetItemQuantity)
# and the answer is 'removed n->0' only when nothing of that record is left, else 'NOT removed n->m' (with RemoveItemByTDBID tried second).
# 'kept (V had it)' = the script did not give it, nothing was removed. If the list API fails the count is the GetItemQuantity one.
function Get-WeaponRemoveCmd([string]$Tag, [string]$Rec) {
    "local ok,e=pcall(function() if not UTPP_GIVEN_$Tag then return 'kept (V had it)' end local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() local tdb=TweakDBID.new('$Rec') local function ids() local out={} pcall(function() local a,b=ts:GetItemList(p) local l=type(b)=='table' and b or a for _,it in ipairs(l) do if ItemID.GetTDBID(it:GetID())==tdb then out[#out+1]=it:GetID() end end end) return out end local function left() local l=ids() if #l>0 then return #l end return ts:GetItemQuantity(p,ItemID.FromTDBID(tdb)) end local n0=left() if n0==0 then return 'UNKNOWN item not found before the removal (count 0)' end local r1='-' for _,id in ipairs(ids()) do r1=tostring(ts:RemoveItem(p,id,1)) end local n1=left() local r2='-' if n1>0 then r2=tostring(pcall(function() return ts:RemoveItemByTDBID(p,tdb,1) end)) n1=left() end if n1==0 then UTPP_GIVEN_$Tag=nil return 'removed '..n0..'->0 r='..r1 end return 'NOT removed '..n0..'->'..n1..' r='..r1..'/'..r2 end) Game.UTPPLogLine('[cetv:@@ID@@][melee] WEAPON_RM $Tag ok='..tostring(ok)..' '..tostring(e))"
}
# guard group (2026-11-09 round 3): the guard settings for this group only, runtime values ({noSave=true}); the values before the group
# are kept in the Lua global UTPP_GUARD_PREV and restored afterwards (the echo shows the effective values: 'nil' = an old CET mod
# without the keys). The Mod Settings sync still pushes changed values (1 s interval), so the restore is followed by a pause.
$guardKeys = if ($GuardLegs) { "'melee.guard','melee.guardAimTilt','melee.guardFullBody'" } else { "'melee.guard','melee.guardAimTilt'" }
$guardOnCmd = "local ok,e=pcall(function() local c=GetMod('utpp_core') if not UTPP_GUARD_PREV then UTPP_GUARD_PREV={} for _,k in ipairs({'melee.guard','melee.guardAimTilt','melee.guardFullBody'}) do UTPP_GUARD_PREV[k]=c.Get(k)==true end end for _,k in ipairs({$guardKeys}) do c.Set(k,true,{noSave=true}) end return 'guard='..tostring(c.Get('melee.guard'))..' tilt='..tostring(c.Get('melee.guardAimTilt'))..' legs='..tostring(c.Get('melee.guardFullBody')) end) Game.UTPPLogLine('[cetv:@@ID@@][melee] GUARD_SET ok='..tostring(ok)..' '..tostring(e)..' | '..tostring(Game.UTPPMeleeStatus()))"
$guardRestoreCmd = "local ok,e=pcall(function() local c=GetMod('utpp_core') local p=UTPP_GUARD_PREV if not p then return 'nothing to restore' end for k,v in pairs(p) do c.Set(k,v,{noSave=true}) end UTPP_GUARD_PREV=nil return 'guard='..tostring(c.Get('melee.guard'))..' tilt='..tostring(c.Get('melee.guardAimTilt'))..' legs='..tostring(c.Get('melee.guardFullBody')) end) Game.UTPPLogLine('[cetv:@@ID@@][melee] GUARD_RESTORE ok='..tostring(ok)..' '..tostring(e))"
$meleeGroups = @('basic', 'aim', 'walk', 'guard')

$run = @()
foreach ($g in @($Groups | ForEach-Object { $_ -split ',' } | Where-Object { $_ })) {   # -File passes 'a,b' as one string
    if ($g -eq 'all') { $run += @($groupTable.Keys | Where-Object { $_ -ne 'guard' }) }   # guard: experimental settings, only when named
    elseif ($groupTable.Contains($g)) { $run += $g }
    else { throw "unknown group '$g' (known: $($groupTable.Keys -join ', '), all)" }
}
$weaponRun = @()
foreach ($w in @($Weapons | ForEach-Object { $_ -split ',' } | Where-Object { $_ })) {
    if ($w -eq 'all') { $weaponRun += @($weaponTable.Keys) }
    elseif ($weaponTable.Contains($w)) { $weaponRun += $w }
    else { throw "unknown weapon '$w' (known: $($weaponTable.Keys -join ', '), all)" }
}
Write-Host ('groups: ' + ($run -join ', ') + $(if ($run -contains 'weapons') { '  weapons: ' + ($weaponRun -join ', ') } else { '' }) + "   out: $Out")

$fail = 0
$startMark = ''
$setupWhy = ''
try {
    # inside the try: a failed launch ('world not reached') still reaches the finally below, which closes the game
    try {
        if (-not $DryRun -and -not $NoLaunch) { Start-Game }
        if (-not $DryRun) {
            $startMark = Get-LogMark
            Invoke-Cet 'start' @('Game.UTPPSetMMTrace(true)',
                "pcall(function() local p=Game.GetPlayer() local v=Game.GetMountedVehicle(p) Game.UTPPLogLine('[cetv:@@ID@@][melee] start mounted='..tostring(v~=nil)) end)")
            if (Find-Log $startMark '\[melee\] start mounted=true') {
                Write-Host 'save loaded mounted: leaving the vehicle'
                Invoke-Run -Tag 'setup-exit' -Scenarios @('bike_exit_only') -ScenarioFile $bikeScenFile
                Invoke-Run -Tag 'setup-walk' -Scenarios @('bike_walk_away') -ScenarioFile $bikeScenFile -NoWake
                Start-Sleep -Seconds 3
            }
            Status-Line 'before'
        }
    }
    catch { $setupWhy = "setup failed: $($_.Exception.Message)"; Write-Warning $setupWhy }
    foreach ($g in $run) {
        if ($setupWhy) { Set-Invalid $g $setupWhy; continue }
        Write-Host "=== group $g"
        try {
            if ($meleeGroups -contains $g) {
                # the fists must be confirmed (typed command arrived complete AND ok=true) before the group may run
                $m = if ($DryRun) { '' } else { Get-LogMark }
                Invoke-Cet "fists-$g" @($fistsCmd)
                if (-not $DryRun) { Start-Sleep -Seconds 2 }
                Assert-Log $m '\[melee\] FISTS ok=true' "fists request for $g reported ok=false (no '[melee] FISTS ok=true' line)"
            }
            elseif ($g -eq 'weapons') {
                foreach ($w in $weaponRun) {
                    Write-Host "--- weapon $w ($($weaponTable[$w]))"
                    try {
                        $m = if ($DryRun) { '' } else { Get-LogMark }
                        Invoke-Cet "weapon-$w" @((Get-WeaponDrawCmd $w $weaponTable[$w]))
                        if (-not $DryRun) { Start-Sleep -Seconds 3 }
                        Assert-Log $m "\[melee\] WEAPON $w ok=true" "draw request for $w reported ok=false (preset not in this game version?)"
                        $m = if ($DryRun) { '' } else { Get-LogMark }
                        Invoke-Cet "inhand-$w" @((Get-InHandCmd $w))
                        Assert-Log $m "\[melee\] INHAND $w ok=true $($weaponFriendly[$w]) " "INHAND $w is not $($weaponFriendly[$w]): another weapon (or nothing) is in the hand"
                        Invoke-Run -Tag "weapons-$w" -Scenarios $groupTable['weapons'] -Record -NoWake
                    }
                    catch { Set-Invalid "weapons/$w" $_.Exception.Message }
                    finally {
                        # always holster and take back what the script gave, also after a failed step
                        try {
                            Invoke-Cet "holster-$w" @((Get-HolsterCmd $w))
                            if (-not $DryRun) { Start-Sleep -Seconds 2 }
                            $m = if ($DryRun) { '' } else { Get-LogMark }
                            Invoke-Cet "remove-$w" @((Get-WeaponRemoveCmd $w $weaponTable[$w]))
                            if (-not $DryRun -and -not (Find-Log $m "\[melee\] WEAPON_RM $w ok=true (removed|kept)")) {
                                $script:soft += "weapon $w NOT removed from the inventory (see the WEAPON_RM line)"
                                Write-Warning "weapon $w was NOT removed from the inventory (WEAPON_RM line shows NOT removed / UNKNOWN / ok=false)"
                            }
                        }
                        catch { $script:soft += "cleanup of weapon $w failed: $($_.Exception.Message)"; Write-Warning "cleanup of $w failed: $_" }
                    }
                }
                if (-not $DryRun) { Status-Line $g }
                continue
            }
            elseif ($g -eq 'gun') {
                # the gun must really be in the hand (not the last melee weapon) before the group may run
                $m = if ($DryRun) { '' } else { Get-LogMark }
                Invoke-Cet 'gun-prime' @($gunPrimeCmd)
                if (-not $DryRun) { Start-Sleep -Seconds 3 }
                Assert-Log $m '\[melee\] GUN_PRIME ok=true' 'gun prime request reported ok=false (no ranged weapon drawn)'
                $m = if ($DryRun) { '' } else { Get-LogMark }
                Invoke-Cet 'gun-inhand' @((Get-InHandCmd 'gun'))
                Assert-Log $m '\[melee\] INHAND gun ok=true ' 'INHAND gun line missing or ok=false'
                Assert-NoLog $m '\[melee\] INHAND gun ok=true (none|\?|w_melee\w*|w_two_hand_blunt|w_fists\w*) ' 'after gun prime the hand holds no ranged weapon (none or a melee weapon)'
                Invoke-Cet 'gun-holster' @((Get-HolsterCmd 'gun'))
                if (-not $DryRun) { Start-Sleep -Seconds 2 }
            }
            if ($g -eq 'guard') {
                try {
                    $m = if ($DryRun) { '' } else { Get-LogMark }
                    Invoke-Cet 'guard-on' @($guardOnCmd)
                    $setRx = '\[melee\] GUARD_SET ok=true guard=true tilt=true' + $(if ($GuardLegs) { ' legs=true' } else { '' })
                    Assert-Log $m $setRx 'guard settings were not switched on (GUARD_SET line missing, ok=false or guard/tilt not true)'
                    Invoke-Run -Tag $g -Scenarios $groupTable[$g] -Record -NoWake
                    if (-not $DryRun) { Status-Line $g }
                }
                finally {
                    # always put the guard settings back, also after a failed step, and let the Mod Settings sync push them
                    try {
                        $m = if ($DryRun) { '' } else { Get-LogMark }
                        Invoke-Cet 'guard-restore' @($guardRestoreCmd)
                        if (-not $DryRun) { Start-Sleep -Seconds 3 }
                        Assert-Log $m '\[melee\] GUARD_RESTORE ok=true' 'GUARD_RESTORE reported ok=false'
                    }
                    catch { $script:soft += "guard settings restore failed: $($_.Exception.Message)"; Write-Warning "guard settings restore failed: $_" }
                }
                continue
            }
            Invoke-Run -Tag $g -Scenarios $groupTable[$g] -Record -NoWake
            if (-not $DryRun) { Status-Line $g }
        }
        catch { Set-Invalid $g $_.Exception.Message }
    }
}
finally {
    if (-not $DryRun) {
        Push-Location $gt
        try { & $Python run.py --padd-stop 2>&1 | Select-Object -Last 1 } catch { Write-Warning "padd stop failed: $_" } finally { Pop-Location }
        $lg = Get-NewestLog
        if ($lg -and $startMark) {
            $dst = Join-Path $Out 'melee-lines.txt'
            $r = Invoke-NavJson @('lines', '--log', $lg, '--since', $startMark, '--marker', '[melee] start ', '--rx', $lineRx, '--out', $dst)
            if ($r -and $r.ok) { Write-Host "lines -> $dst ($($r.lines) lines)" } else { Write-Warning 'no melee lines file (start marker not found)' }
        }
        if (-not $KeepGame) {
            Stop-Process -Name Cyberpunk2077 -Force -ErrorAction SilentlyContinue
            for ($i = 0; $i -lt 30 -and (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 1 }
            Write-Host "game closed: $(-not [bool](Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue))"
        }
    }
}
$summary = @("melee cases $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  groups=$($run -join ',')  invalidated: $($script:invalid.Count)  CET commands typed twice: $($script:cetRetried)")
if ($script:invalid.Count) {
    $summary += 'INVALIDATED (a preparation step was not confirmed; the results of these groups/weapons are NOT valid, repeat them):'
    foreach ($k in $script:invalid.Keys) { $summary += "  - $k : $($script:invalid[$k])" }
}
foreach ($s in $script:soft) { $summary += "NOTE: $s" }
$linesFile = Join-Path $Out 'melee-lines.txt'
if (-not $DryRun -and (Test-Path $linesFile)) {
    $t = Get-Content $linesFile
    $summary += ('MMMELEEFULL on: {0}  off: {1}  MELEEAIM lines: {2}  why= {3}' -f
        @($t | Where-Object { $_ -cmatch 'UTPP MMMELEEFULL on' }).Count, @($t | Where-Object { $_ -cmatch 'UTPP MMMELEEFULL off' }).Count,
        @($t | Where-Object { $_ -cmatch 'UTPP MELEEAIM ' }).Count,
        ((@($t | ForEach-Object { if ($_ -cmatch 'UTPP MMMELEEFULL \w+ why=(\w+)') { $Matches[1] } }) | Group-Object | ForEach-Object { "$($_.Name)=$($_.Count)" }) -join ','))
    $summary += ('MMGUARD on: {0}  off: {1}  why= {2}' -f
        @($t | Where-Object { $_ -cmatch 'UTPP MMGUARD on' }).Count, @($t | Where-Object { $_ -cmatch 'UTPP MMGUARD off' }).Count,
        ((@($t | ForEach-Object { if ($_ -cmatch 'UTPP MMGUARD \w+ why=(\w+)') { $Matches[1] } }) | Group-Object | ForEach-Object { "$($_.Name)=$($_.Count)" }) -join ','))
    $summary += ('FISTS echoes: ' + ((@($t | Where-Object { $_ -cmatch '\[melee\] FISTS' }) | ForEach-Object { ($_ -replace '^.*\[melee\] ', '') }) -join ' | '))
    $summary += @($t | Where-Object { $_ -cmatch '\[melee\] (WEAPON|INHAND|HOLSTER|WEAPON_RM|GUN_PRIME|GUARD_SET|GUARD_RESTORE) ' } | ForEach-Object {
            $s = ($_ -replace '^.*\[melee\] ', ''); if ($s.Length -gt 220) { $s.Substring(0, 220) } else { $s } })
}
$summary | ForEach-Object { Write-Host $_ }
if (-not $DryRun) { $summary | Set-Content -Path (Join-Path $Out 'melee-cases-summary.txt') -Encoding UTF8 }
$validGroups = @($run | Where-Object { $g = $_; -not @($script:invalid.Keys | Where-Object { $_ -eq $g -or $_ -like "$g/*" }) })
Write-Host "done: $($validGroups.Count) of $($run.Count) groups valid; out: $Out"
if ($fail) { exit 1 }

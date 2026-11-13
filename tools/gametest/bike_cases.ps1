<#
.SYNOPSIS
Motorcycle test cases for the UTPP vehicle work: mount from the left / right, dismount to the left / right, a full cycle.
.DESCRIPTION
Sibling of car_cases.ps1 (the car cases are untouched). Drives run.py with scenarios_bike_cases.json: launches the game (unless -NoLaunch),
leaves the vehicle the save loads mounted on and walks away (only after a launch made by this script; -ForceSetup runs it with -NoLaunch),
spawns -Vehicle (Codeware dynamic entity, tag UTPPCar, same as the car cases) next to V, switches on GetMod('utpp_vehiclecam').SetCamTrace(true)
and GetMod('utpp_core').NightAuto(true,0.25,'UTPPCar') ('[night] POS' every 0.25 s), runs the pad steps of each case, records the game window
with ffmpeg (one mp4 per mount attempt, ride and dismount) and saves the log lines of the case to <Out>\<case>-lines.txt.
Nothing here is a pass/fail verdict of the animation: the summary only says whether V mounted / dismounted and on which side.
Needs ViGEmBus, the game in the FOREGROUND (run.py aborts when focus is lost), CET and the UTPP log. -DryRun forwards --dry-run to run.py
(no input, no launch, no recording) and prints the plan and the CET commands.

Why car_cases.ps1 -Vehicle <bike> ends in NOT MOUNTED (2026-11-07): the car geometry. The bike
has no preset in car_cases_nav.py, so the generic CAR window applied (V may stand 1.5 .. 2.7 m to the side) and V pressed X at lat 1.49 .. 1.59 m of the
bike's centre line (the '[night] POS' lines). The bike 'Mount' prompt is gated by a seat sphere of r 1.0 .. 1.3 m around V (InteractRangeCore.hpp;
UTPP InteractRange mode 1 = sphere only, no look-at); beyond it there is no prompt and X does nothing (the video shows no prompt). The UTPP approach
planner skips bikes (the engine slides V to the clip start itself), so nothing walks V closer. bike_cases_nav.py gives bike presets: V stands 0.3 ..
0.9 m beside the bike (target 0.85); the seat offset from the bike origin is not extracted, so a mount ladder shifts V along the bike
(lon 0, +0.45, -0.45, +0.9 m) until a mount line appears (the first success is printed as the calibration for the record).

Cases (-Cases; default 'all' = all five in this order):
  mount_left      V stands at the bike's LEFT (bike spawned 0.9 m to V's right), X, 6.5 s seated, dismount (side not forced).
  mount_right     V at the bike's RIGHT (bike at V's left), X, seated, dismount (side not forced).
  dismount_left   mount from the left, (ride), dismount forced to the left.
  dismount_right  mount from the left, (ride), dismount forced to the right.
  cycle           mount L -> short ride + stop -> dismount R -> mount R -> dismount L (the bike is respawned on V's side when the
                  measured dismount side differs from the side the next mount needs: V never walks through the bike).
Dismount side: it is NOT a script choice. vehicleTransition.script:919 calls the native vehicle.CanUnmount(true, player) (no stick / direction
argument); for bikes the closest and the opposite exit are both accepted, so the engine picks the free side. Therefore a forced side works by
BLOCKING the other side: a parked car (Vehicle.v_standard2_archer_hella_player, tag UTPPBikeBlock) is spawned 1.6 m beside the bike on the
opposite side just before the B hold and removed right after (-NoBlocker disables it; UNVERIFIED in game). -ExitStick additionally holds the left
stick toward the wanted side during the B hold (also unverified; the stick may only steer the parked bike). The measured side (sign of lat in the
bike frame from the POS line after the exit) is logged as '[bike] EXIT want=.. actual=.. SIDE OK|SIDE MISMATCH' and printed in the summary.
Before every B hold the native answers CanUnmount(default / Left / Right) are logged as '[bike] CanUnmount ...' (diagnostic, may print err).

Mount guard: after X the log is searched for the ride lines ('ride N: utpp_vehicle_tpp=1', 'vehtrans mountdiag', 'workspot animVariables ... kind=bike');
four attempts at the ladder positions, then '[bike] NOT MOUNTED case=..' and the rest of the case is skipped (no drive / B hold on foot: a B hold on
foot is a dodge). After the first failed X the interaction state is logged ('[bike] ir ...' = UTPPInteractRangeStatus, '[bike] probe ...' = utpp_loothead Probe).
Dismount guard: no '... exit: unmounted' / 'keepTpp exit -> off' / 'ExitWithTeleport' line after the B hold = '[bike] NOT DISMOUNTED', the case stops.

Vehicles (-Vehicle; a TweakDB record or a shorthand):
  sportbike / kusanagi  Vehicle.v_sportbike1_yaiba_kusanagi_player   (Yaiba Kusanagi; record UNVERIFIED, the project data also names
                                                                   v_sportbike1_yaiba_kusanagi_player_02 - the spawn checks TweakDB:GetRecord and logs RECORD MISSING)
  sportbike2 / arch     Vehicle.v_sportbike2_arch_player             (Arch; spawned OK in the 2026-11-07 run)
  sportbike3 / apollo   Vehicle.v_sportbike3_brennan_apollo_player   (Brennan Apollo; record UNVERIFIED, same check)

Run commands (from the repo's tools\gametest folder or by full path; the game window must be in the foreground):
  powershell -File bike_cases.ps1 -DryRun                                  plan of everything, no input
  powershell -File bike_cases.ps1 -Cases cycle -Vehicle sportbike2         one case, launches the game first
  powershell -File bike_cases.ps1 -Cases mount_left,mount_right -NoLaunch  game already in the world, V on foot (no setup)
  powershell -File bike_cases.ps1 -Vehicle apollo -Ride                    all cases, with a short ride before every dismount
Output: <Out> (default Research\logs\gametest\bike-cases-<stamp>): run.py output per step, <tag>-<n>.mp4 (in Media\mp4\YYYY-MM-DD_gametest_<run>), <case>-lines.txt, bike-cases-summary.txt.
#>
[CmdletBinding()]
param(
    [string[]]$Cases = @('all'),
    [switch]$DryRun,
    [switch]$NoLaunch,
    [switch]$ForceSetup,
    [switch]$NoRecord,
    [switch]$CloseGame,
    [string]$Out = '',
    [string]$Log = '',
    [string]$Python = 'python',
    [string]$Game = '',
    # TweakDB record or shorthand (sportbike|kusanagi, sportbike2|arch, sportbike3|apollo)
    [string]$Vehicle = 'sportbike2',
    # lateral distance V -> bike centre line at the spawn (V's capsule 0.3 m + the bike half width ~0.4 m: do not go below 0.8)
    [double]$SpawnSide = 0.9,
    # short ride (drive 2 s, brake) between the mount and the dismount in every case (the cycle case always rides)
    [switch]$Ride,
    # no blocker car on the opposite side: the engine picks the dismount side
    [switch]$NoBlocker,
    # also hold the left stick toward the wanted side during the B hold (unverified lever)
    [switch]$ExitStick,
    # mount attempts at the lon ladder positions (max 4)
    [int]$NavTries = 4,
    [switch]$NoHome,
    # minimum pause before any call that types console commands (seconds since the previous run.py call ended; see car_cases.ps1)
    [double]$CetGapS = 6
)

$ErrorActionPreference = 'Stop'
$null = Get-Command $Python -ErrorAction Stop
$gt = $PSScriptRoot
. (Join-Path $gt '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$bikeScenFile = Join-Path $gt 'scenarios_bike_cases.json'
$navPy = Join-Path $gt 'bike_cases_nav.py'
$root = $null
for ($p = $gt; $p; $p = Split-Path -Parent $p) { if (Test-Path (Join-Path $p 'Tools\pylibs')) { $root = $p; break } }
if (-not $root) { throw 'project root (folder with Tools\pylibs) not found above ' + $gt }
$ffmpeg = Join-Path $root 'Tools\pylibs\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe'
if (-not $Out) { $Out = Join-Path $root ('Research\logs\gametest\bike-cases-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
# Recordings go straight to Media\mp4\YYYY-MM-DD_gametest_<run> (standing project rule); logs stay in $Out.
$MediaOut = Join-Path $root ('Media\mp4\' + (Get-Date -Format 'yyyy-MM-dd') + '_gametest_' + (Split-Path $Out -Leaf))
New-Item -ItemType Directory -Force $Out | Out-Null
$inv = [Globalization.CultureInfo]::InvariantCulture
function N([double]$v) { $v.ToString('0.###', $inv) }

# ---- vehicle ------------------------------------------------------------------------------------------------------------------
$bikeAlias = @{
    'sportbike' = 'Vehicle.v_sportbike1_yaiba_kusanagi_player'; 'sportbike1' = 'Vehicle.v_sportbike1_yaiba_kusanagi_player'; 'kusanagi' = 'Vehicle.v_sportbike1_yaiba_kusanagi_player'
    'sportbike2' = 'Vehicle.v_sportbike2_arch_player'; 'arch' = 'Vehicle.v_sportbike2_arch_player'
    'sportbike3' = 'Vehicle.v_sportbike3_brennan_apollo_player'; 'apollo' = 'Vehicle.v_sportbike3_brennan_apollo_player'
}
if ($bikeAlias.ContainsKey($Vehicle.ToLowerInvariant())) { $Vehicle = $bikeAlias[$Vehicle.ToLowerInvariant()] }
if ($Vehicle -notmatch '^Vehicle\.[A-Za-z0-9_]+$') { throw "bad vehicle record '$Vehicle' (expected Vehicle.<name> or a shorthand: $($bikeAlias.Keys -join ', '))" }
$blockerRecord = 'Vehicle.v_standard2_archer_hella_player'
$preset = (& $Python -I $navPy preset $Vehicle | Select-Object -Last 1) | ConvertFrom-Json
$verified = if ($Vehicle -eq 'Vehicle.v_sportbike2_arch_player') { 'record verified (spawned in the 2026-11-07 run)' } else { 'record UNVERIFIED (checked by TweakDB:GetRecord at the spawn)' }

$lineRx = '\[night\] POS|NOT MOUNTED|NOT DISMOUNTED|\[car\]|\[bike\]|\[utpp_vehiclecam\]|\[utpp_vehicle_exit\]|\[utpp_vehicle_anim\]|\[utpp_camera\]|vehtrans|JUMPOUT|EXITDIAG|gamelook|keepTpp|always third person|vehicle hold|\[utpp_vehicle_door\]'
$script:clip = 0
$script:results = New-Object System.Collections.ArrayList
$script:notMounted = 0
$script:notDismounted = 0
$script:sideMismatch = 0
$script:spawnSkipped = 0
$script:spawnOk = @{}
$script:spawnMarks = @{}
$script:lastRunEnd = $null

function Read-Shared([string]$path) {
    $s = [IO.File]::Open($path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]"ReadWrite, Delete")
    $r = New-Object IO.StreamReader($s)
    try { return $r.ReadToEnd() } finally { $r.Dispose() }
}

# Log access goes through the car_cases_nav.py helpers (rotation-safe marks; the UTPP log rotates at 10 MB), reached via bike_cases_nav.py.
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
    if (-not $lg -or -not $Mark) { return $null }
    $r = Invoke-NavJson @('found', '--log', $lg, '--since', $Mark, '--rx', $Rx)
    return [bool]($r -and $r.found)
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

# One run.py call (see car_cases.ps1): -Scenarios pad steps (the pad stays plugged), -Cet console commands typed first (';;' separated);
# no scenario = a CET-only call. -NoWake: no right-stick wake nudge (it moves the camera ~6 deg; only the first call needs it).
function Invoke-Run {
    param([string]$Tag, [string[]]$Scenarios = @(), [string]$Cet = '', [switch]$Record, [string]$ScenFile = '', [switch]$NoWake)
    $scn = if ($ScenFile) { $ScenFile } else { $bikeScenFile }
    $ra = @('run.py', '--out', (Join-Path $Out $Tag), '--keep-pad', '--no-dismiss')
    if ($Scenarios.Count) { $ra += @('--scenario-file', $scn, '--scenarios', ($Scenarios -join ',')) } else { $ra += @('--wake', 'none') }
    if ($NoWake -and $Scenarios.Count) { $ra += @('--wake', 'none') }
    if ($Cet) { $ra += @('--cet', $Cet) }
    if ($DryRun) { $ra += '--dry-run' }
    if ($Log) { $ra += @('--log', $Log) }
    $ErrorActionPreference = 'Continue'
    $ff = $null
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
        if ($DryRun) { Write-Host "--- $Tag"; $o | Where-Object { $_ -match '^(CET commands|estimated input time|\S+ +[0-9.]+ s |bike_\w+: )' } | Out-Host }
        else { $o | Select-Object -Last 3 | Out-Host }
        if ($code -ne 0) { $o | Select-Object -Last 6 | Out-Host; throw "run.py failed ($code) for $Tag" }
    }
    finally {
        Pop-Location
        if ($ff) { [void]$ff.WaitForExit(([int]$secs + 20) * 1000) }
        $script:lastRunEnd = Get-Date
    }
}
function Invoke-Cet([string]$Tag, [string[]]$Cmds) { Invoke-Run -Tag $Tag -Cet ($Cmds -join ';;') }

# ---- spawn --------------------------------------------------------------------------------------------------------------------
# V stands at the bike's -VSide- (left: the bike is 0.9 m to V's right). Replaces the previous UTPPCar and UTPPBikeBlock.
# A respawn inside a case logs '[car] RESPAWN case=.. n=..' (not '[car] SPAWN') so the lines extraction keeps one segment per case.
function Test-Spawned([string]$Prefix, [string]$Mark) {
    $f = Find-Log $Mark ([regex]::Escape("$Prefix ok=true"))
    if ($null -eq $f) { return $true }   # no log / mark: assumed spawned
    return $f
}
function Spawn-Bike([string]$Case, [string]$VSide, [switch]$Respawn) {
    $r = if ($VSide -eq 'left') { $SpawnSide } else { -$SpawnSide }
    $sf = [double]$preset.fwd; $sa = [double]$preset.away
    $n = 1 + [int]$script:respawns[$Case]
    $prefix = if ($Respawn) { "[car] RESPAWN case=$Case n=$n" } else { "[car] SPAWN case=$Case" }
    $lua = "local ok,e=pcall(function() local okr,rec=pcall(function() return TweakDB:GetRecord('$Vehicle') end) if okr and rec==nil then error('record missing: $Vehicle') end local des=Game.GetDynamicEntitySystem() des:DeleteTagged('UTPPCar') des:DeleteTagged('UTPPBikeBlock') local p=Game.GetPlayer() local a=p:GetWorldPosition() local r=p:GetWorldRight() local f=p:GetWorldForward() local s=DynamicEntitySpec.new() s.recordID=TweakDBID.new('$Vehicle') local px=a.x+r.x*($(N $r))+f.x*0 local py=a.y+r.y*($(N $r))+f.y*0 local dx=px-a.x local dy=py-a.y local dl=math.max(0.01,math.sqrt(dx*dx+dy*dy)) px=px+f.x*($(N $sf))+dx/dl*($(N $sa)) py=py+f.y*($(N $sf))+dy/dl*($(N $sa)) s.position=Vector4.new(px,py,a.z+0.3,1) s.orientation=EulerAngles.new(0,0,f:ToRotation().yaw):ToQuat() s.alwaysSpawned=true s.persistState=false s.persistSpawn=false s.tags={'UTPPCar'} return des:CreateEntity(s) end) Game.UTPPLogLine('$prefix ok='..tostring(ok)..' '..tostring(e)) GetMod('utpp_vehiclecam').SetCamTrace(true) Game.UTPPSetVehicleVaultLog(true) pcall(function() GetMod('utpp_core').NightAuto(true,0.25,'UTPPCar') end)"
    $script:spawnOk[$Case] = $true
    if (-not $Respawn -and -not $NoHome) {
        $homeCmd = "Game.UTPPLogLine('[car] CMD home case=$Case') local p=Game.GetPlayer() if UTPPCarHome==nil then local w=p:GetWorldPosition() UTPPCarHome={x=w.x,y=w.y,z=w.z,yaw=p:GetWorldYaw()} Game.UTPPLogLine('[car] HOME set case=$Case') else Game.GetTeleportationFacility():Teleport(p,Vector4.new(UTPPCarHome.x,UTPPCarHome.y,UTPPCarHome.z,1),EulerAngles.new(0,0,UTPPCarHome.yaw)) Game.UTPPLogLine('[car] HOME return case=$Case') end"
        try { Invoke-Cet "$Case-home" @($homeCmd) } catch { Write-Warning "case ${Case}: HOME teleport not confirmed: $_" }
        if (-not $DryRun) { Start-Sleep -Seconds 2 }
    }
    for ($try = 1; $try -le 2; $try++) {
        $mark = if ($DryRun) { '' } else { Get-LogMark }
        if (-not $Respawn -and -not $DryRun -and $try -eq 1) { $script:spawnMarks[$Case] = $mark }   # case start: nav / lines only look after it
        $tg = if ($Respawn) { "$Case-respawn$n" } else { "$Case-spawn" }
        try { Invoke-Run -Tag "$tg$(if ($try -gt 1) { $try })" -Scenarios @('bike_idle') -Cet "Game.UTPPLogLine('[car] CMD spawn case=$Case try=$try side=$VSide') $lua" }
        catch { Write-Warning "case ${Case}: spawn command not confirmed (try $try): $_" }
        if ($DryRun) { return }
        Start-Sleep -Seconds ([double]$preset.settle)
        if (Test-Spawned $prefix $mark) { if ($Respawn) { $script:respawns[$Case] = $n }; return }
        Write-Warning "case ${Case}: no '$prefix ok=true' line after the spawn command (try $try; a '[car] ... ok=false record missing' line = the TweakDB record does not exist)"
    }
    $script:spawnOk[$Case] = $false
    $script:spawnSkipped++
    Write-Warning "case ${Case}: the test bike did not spawn - the case is SKIPPED and counted as a failure"
}
$script:respawns = @{}

function Save-Lines([string]$Case) {
    if ($DryRun) { return }
    $lg = Get-NewestLog
    if (-not $lg) { Write-Warning 'no UTPP log found'; return }
    $since = $script:spawnMarks[$Case]
    if (-not $since) { Write-Warning "no case start mark for $Case (the spawn did not run) - no lines file"; return }
    $dst = Join-Path $Out "$Case-lines.txt"
    $r = Invoke-NavJson @('lines', '--log', $lg, '--since', $since, '--marker', "[car] SPAWN case=$Case ", '--rx', $lineRx, '--out', $dst)
    if ($r -and $r.ok) { Write-Host "lines -> $dst ($($r.lines) lines)" }
    else { Write-Warning "no spawn marker for $Case in the log session of $lg after the case start - no lines file" }
}

# ---- position feedback ---------------------------------------------------------------------------------------------------------
function Get-Nav([string]$Case, [string]$Side, [string]$Mode, [double]$Fraction, [double]$LonBias) {
    $lg = Get-NewestLog
    if (-not $lg) { return $null }
    $na = @('-I', $navPy, 'nav', '--log', $lg, '--marker', "[car] SPAWN case=$Case ", '--vehicle', $Vehicle, '--side', $(if ($Side -eq 'left') { 'driver' } else { 'passenger' }), '--mode', $Mode, '--fraction', (N $Fraction), '--lon-bias', (N $LonBias), '--out-scen', (Join-Path $Out '_nav.json'))
    $since = $script:spawnMarks[$Case]
    if ($since) { $na += @('--since', $since) }
    try { return ((& $Python @na 2>&1 | Select-Object -Last 1) | ConvertFrom-Json) } catch { Write-Warning "nav helper failed: $_"; return $null }
}

# Which side of the bike V stands on now (state ok|mounted|nopos|nocar); $null without a log.
function Get-BikeSide([string]$Case) {
    $lg = Get-NewestLog
    if (-not $lg) { return $null }
    $na = @('-I', $navPy, 'side', '--log', $lg, '--marker', "[car] SPAWN case=$Case ")
    $since = $script:spawnMarks[$Case]
    if ($since) { $na += @('--since', $since) }
    try { return ((& $Python @na 2>&1 | Select-Object -Last 1) | ConvertFrom-Json) } catch { Write-Warning "side helper failed: $_"; return $null }
}

# V on the wrong side of the bike for the next mount: respawn the bike on V's side (walking would cross the bike).
function Ensure-Side([string]$Case, [string]$Want) {
    if ($DryRun) { Write-Host "  side check [$Case]: skipped in a dry run (respawns the bike when V stands on the wrong side)"; return }
    $s = Get-BikeSide $Case
    if ($s -and $s.state -eq 'ok' -and $s.side -ne $Want) {
        Write-Host ("  side [{0}]: V stands at the bike's {1} (lat {2}), the next mount needs the {3}: respawn the bike on V's {3} side" -f $Case, $s.side, $s.lat, $Want)
        Spawn-Bike $Case $Want -Respawn
    }
    elseif (-not $s -or $s.state -ne 'ok') { Write-Warning "side [$Case]: not measurable ($(if ($s) { $s.state + ' ' + $s.why } else { 'no log' }))" }
}

function Approach-Seat([string]$Case, [string]$Tag, [string]$Side, [double]$LonBias) {
    if ($DryRun) { Write-Host "  nav [$Case/$Side]: position feedback skipped in a dry run (reads '[night] POS' after '[car] SPAWN case=$Case ', walks to lon=$($preset.lon_target)+$(N $LonBias) lat=$(N ($preset.door_x + 0.45)) at the bike's $Side; window lat $(N ($preset.door_x - 0.1))..$(N $preset.lat_max))"; return }
    for ($i = 1; $i -le 4; $i++) {
        $fr = if ($i -le 2) { 0.85 } else { 0.5 }
        $r = Get-Nav $Case $Side 'check' $fr $LonBias
        if (-not $r) { Write-Warning "nav [$Case]: no answer (no log) - X pressed without feedback"; return }
        Write-Host ("  nav [{0}/{1}] #{2}: state={3} lon={4} lat={5} (target lon {6} + bias {7})  {8}" -f $Case, $Side, $i, $r.state, (N $r.lon), (N $r.lat), (N $r.lon_target), (N $LonBias), $r.why)
        if ($r.state -eq 'ok') { return }
        if ($r.state -ne 'move') { Write-Warning "nav [$Case]: $($r.state) $($r.why) - X pressed without feedback"; return }
        Invoke-Run -Tag "$Tag-nav$i" -Scenarios @('car_nav') -ScenFile $r.scenario -NoWake
    }
    Write-Warning "nav [$Case]: still outside the window after 4 walks - X pressed anyway"
}

# ---- mount / dismount ---------------------------------------------------------------------------------------------------------
$mountRx = 'ride \d+: utpp_vehicle_tpp=1|vehtrans mountdiag: ride=|keepTpp mount -> ride|workspot animVariables .* kind=bike'
$exitRx = '\[utpp_vehiclecam\].* exit: unmounted|keepTpp exit -> off|\[utpp_vehicle_exit\] ExitWithTeleport'

function Test-Mounted([string]$Mark) {
    if ($DryRun) { return $true }
    for ($i = 0; $i -lt 3; $i++) {
        $f = Find-Log $Mark $mountRx
        if ($null -eq $f) { Write-Warning 'no UTPP log mark: mount not verifiable (assumed mounted)'; return $true }
        if ($f) { return $true }
        Start-Sleep -Milliseconds 1500
    }
    return $false
}
function Test-Dismounted([string]$Mark) {
    if ($DryRun) { return $true }
    for ($i = 0; $i -lt 3; $i++) {
        $f = Find-Log $Mark $exitRx
        if ($null -eq $f) { Write-Warning 'no UTPP log mark: dismount not verifiable (assumed done)'; return $true }
        if ($f) { return $true }
        Start-Sleep -Milliseconds 1500
    }
    return $false
}

$irDiag = "pcall(function() Game.UTPPLogLine('[bike] ir '..tostring(Game.UTPPInteractRangeStatus()):gsub('%s+',' ')) end) pcall(function() Game.UTPPLogLine('[bike] probe '..tostring(GetMod('utpp_loothead').Probe(4.0)):gsub('%s+',' ')) end)"

# Mount from the bike's -Side-: the lon ladder (hypothesis: the seat offset is unknown), X each time, mount guard. $true = mounted.
# The result goes to $script:mountLon (the lon bias that mounted) - NOT the output stream.
function Mount-Bike([string]$Case, [string]$Tag, [string]$Side) {
    $script:mountLon = $null
    if ($script:spawnOk.ContainsKey($Case) -and -not $script:spawnOk[$Case]) { Write-Warning "case ${Case}: SKIPPED - the test bike did not spawn"; return $false }
    Ensure-Side $Case $Side   # V on the wrong side (e.g. after an exit to the other side): respawn the bike, never walk through it
    $ladder = @(0.0, 0.45, -0.45, 0.9) | Select-Object -First ([Math]::Max(1, [Math]::Min(4, $NavTries)))
    if ($DryRun) { Write-Host "  mount ladder [$Case/$Side]: lon bias $(($ladder | ForEach-Object { N $_ }) -join ', ') m, stops at the first mount line" }
    $k = 0
    foreach ($bias in $ladder) {
        $k++
        Approach-Seat $Case "$Tag-a$k" $Side $bias
        $off = if ($DryRun) { '' } else { Get-LogMark }
        Invoke-Run -Tag "$Tag-mount$k" -Scenarios @('bike_mount') -NoWake -Record
        if (Test-Mounted $off) { $script:mountLon = $bias; if (-not $DryRun) { Write-Host "  MOUNT OK [$Case/$Side] attempt $k (lon bias $(N $bias) m)" }; return $true }
        Write-Warning "case ${Case}: no mount line after X (attempt $k, lon bias $(N $bias) m)"
        if ($k -eq 1) { try { Invoke-Cet "$Tag-irdiag" @($irDiag) } catch { Write-Warning "interaction diagnostic failed: $_" } }
    }
    Write-Warning "case ${Case}: NOT MOUNTED after $k attempts - the rest of the case is skipped (no drive / B hold on foot)"
    try { Invoke-Cet "$Tag-notmounted" @("Game.UTPPLogLine('[bike] NOT MOUNTED case=$Case side=$Side attempts=$k vehicle=$Vehicle')") } catch { Write-Warning "log line failed: $_" }
    $script:notMounted++
    return $false
}

# Dismount: -Want left|right = forced (blocker car on the other side, optional stick), any = engine's choice. Measures the actual side.
# Result: $script:exitSide (left|right|unknown) and $true / $false (dismounted) - the bool is the return value, nothing else is emitted.
function Dismount-Bike([string]$Case, [string]$Tag, [string]$Want) {
    $script:exitSide = 'unknown'
    $block = ($Want -ne 'any') -and -not $NoBlocker
    if ($block) {
        $sg = if ($Want -eq 'right') { -1 } else { 1 }   # lateral sign along the bike's right vector: right exit -> block the LEFT (-1.6 m)
        $bl = "local ok,e=pcall(function() local des=Game.GetDynamicEntitySystem() des:DeleteTagged('UTPPBikeBlock') local b=des:GetTagged('UTPPCar')[1] local bp=b:GetWorldPosition() local br=b:GetWorldRight() local s=DynamicEntitySpec.new() s.recordID=TweakDBID.new('$blockerRecord') s.position=Vector4.new(bp.x+br.x*($sg*1.6),bp.y+br.y*($sg*1.6),bp.z+0.3,1) s.orientation=b:GetWorldOrientation() s.alwaysSpawned=true s.persistState=false s.persistSpawn=false s.tags={'UTPPBikeBlock'} return des:CreateEntity(s) end) Game.UTPPLogLine('[bike] BLOCKER case=$Case wantExit=$Want blocksSide='..'$(if ($Want -eq 'right') { 'left' } else { 'right' })'..' ok='..tostring(ok)..' '..tostring(e))"
        try { Invoke-Cet "$Tag-blocker" @($bl) } catch { Write-Warning "blocker spawn not confirmed: $_" }
        if (-not $DryRun) { Start-Sleep -Seconds 3 }
    }
    $cu = "local function cu(d) local ok,r=pcall(function() local v=Game.GetDynamicEntitySystem():GetTagged('UTPPCar')[1] if d then return v:CanUnmount(true,Game.GetPlayer(),Enum.new('vehicleExitDirection',d)) end return v:CanUnmount(true,Game.GetPlayer()) end) if not ok then return 'err' end return tostring(r.direction) end Game.UTPPLogLine('[bike] CanUnmount case=$Case want=$Want default='..cu(nil)..' left='..cu('Left')..' right='..cu('Right'))"
    try { Invoke-Cet "$Tag-canunmount" @($cu) } catch { Write-Warning "CanUnmount diagnostic failed: $_" }
    $scn = if ($ExitStick -and $Want -eq 'left') { 'bike_dismount_stick_l' } elseif ($ExitStick -and $Want -eq 'right') { 'bike_dismount_stick_r' } else { 'bike_dismount' }
    $off = if ($DryRun) { '' } else { Get-LogMark }
    Invoke-Run -Tag "$Tag-dismount" -Scenarios @($scn) -NoWake -Record
    $done = Test-Dismounted $off
    $actual = 'unknown'
    if ($done -and -not $DryRun) {
        $s = Get-BikeSide $Case
        if ($s -and $s.state -eq 'ok') { $actual = $s.side }
    }
    $verdict = if ($Want -eq 'any') { 'side not forced' } elseif ($actual -eq $Want) { 'SIDE OK' } elseif ($actual -eq 'unknown') { 'SIDE UNKNOWN' } else { 'SIDE MISMATCH' }
    if ($verdict -eq 'SIDE MISMATCH') { $script:sideMismatch++ }
    $clean = "pcall(function() Game.GetDynamicEntitySystem():DeleteTagged('UTPPBikeBlock') end) Game.UTPPLogLine('[bike] EXIT case=$Case want=$Want actual=$actual dismounted=$(([string]$done).ToLower()) $verdict')"
    try { Invoke-Cet "$Tag-exitresult" @($clean) } catch { Write-Warning "exit result line / blocker cleanup failed: $_" }
    if (-not $done) {
        Write-Warning "case ${Case}: NOT DISMOUNTED (no exit line after the B hold) - the rest of the case is skipped"
        $script:notDismounted++
    }
    $script:exitSide = $actual
    if (-not $DryRun) { Write-Host "  DISMOUNT [$Case] want=$Want actual=$actual dismounted=$done  $verdict" }
    return $done
}

function Add-Result([string]$Case, [string]$Text) { [void]$script:results.Add(("{0,-15} {1}" -f $Case, $Text)) }

function Do-Ride([string]$Case, [string]$Tag) { Invoke-Run -Tag "$Tag-ride" -Scenarios @('bike_ride') -NoWake -Record }

# ---- cases --------------------------------------------------------------------------------------------------------------------
function Case-Mount([string]$Side) {
    $c = "mount_$Side"
    Spawn-Bike $c $Side
    $m = Mount-Bike $c $c $Side
    if (-not $m) { Add-Result $c "mount $Side : NOT MOUNTED"; Save-Lines $c; return }
    $ml = $script:mountLon
    if ($Ride) { Do-Ride $c $c }
    $d = Dismount-Bike $c $c 'any'
    Add-Result $c ("mount $Side : mounted (lon bias {0}); dismount side not forced: actual {1}{2}" -f (N $ml), $script:exitSide, $(if (-not $d) { ' (NOT DISMOUNTED)' } else { '' }))
    Save-Lines $c
}

function Case-Dismount([string]$Want) {
    $c = "dismount_$Want"
    Spawn-Bike $c 'left'
    $m = Mount-Bike $c $c 'left'
    if (-not $m) { Add-Result $c 'setup mount left : NOT MOUNTED'; Save-Lines $c; return }
    $ml = $script:mountLon
    if ($Ride) { Do-Ride $c $c }
    $d = Dismount-Bike $c $c $Want
    Add-Result $c ("setup mount left (lon bias {0}); dismount wanted {1}: actual {2}{3}" -f (N $ml), $Want, $script:exitSide, $(if (-not $d) { ' (NOT DISMOUNTED)' } else { '' }))
    Save-Lines $c
}

function Case-Cycle {
    $c = 'cycle'
    Spawn-Bike $c 'left'
    $steps = @()
    $m = Mount-Bike $c "$c-1" 'left'
    if (-not $m) { Add-Result $c 'step 1 mount L : NOT MOUNTED'; Save-Lines $c; return }
    $steps += 'mount L ok'
    Do-Ride $c "$c-1"
    if (-not (Dismount-Bike $c "$c-2" 'right')) { Add-Result $c ($steps -join ' > ') ; Add-Result $c 'step 2 dismount R : NOT DISMOUNTED'; Save-Lines $c; return }
    $steps += "dismount R (actual $($script:exitSide))"
    $m = Mount-Bike $c "$c-3" 'right'
    if (-not $m) { Add-Result $c (($steps -join ' > ') + ' > mount R : NOT MOUNTED'); Save-Lines $c; return }
    $steps += 'mount R ok'
    if (-not (Dismount-Bike $c "$c-4" 'left')) { Add-Result $c (($steps -join ' > ') + ' > dismount L : NOT DISMOUNTED'); Save-Lines $c; return }
    $steps += "dismount L (actual $($script:exitSide))"
    Add-Result $c ($steps -join ' > ')
    Save-Lines $c
}

$table = [ordered]@{
    mount_left = { Case-Mount 'left' }
    mount_right = { Case-Mount 'right' }
    dismount_left = { Case-Dismount 'left' }
    dismount_right = { Case-Dismount 'right' }
    cycle = { Case-Cycle }
}
$run = @()
foreach ($c in @($Cases | ForEach-Object { $_ -split ',' } | Where-Object { $_ })) {   # -File passes 'a,b' as one string
    if ($c -eq 'all') { $run += @($table.Keys) }
    elseif ($table.Contains($c)) { $run += $c }
    else { throw "unknown case '$c' (known: $($table.Keys -join ', '), all)" }
}
Write-Host ('cases: ' + ($run -join ', ') + "   out: $Out")
Write-Host ("vehicle: $Vehicle  ($verified)  preset={0} ({1}{2})  spawn: V {3} m beside the bike  mount window: V lat {4}..{5} m beside the bike (target {6}), lon {7} +-0.25 + ladder 0/+0.45/-0.45/+0.9; blocker={8} exitStick={9} ride={10}" -f `
    $preset.key, $preset.label, $(if (-not $preset.known) { ', NOT a known preset' } else { '' }), (N $SpawnSide), (N ($preset.door_x - 0.1)), (N $preset.lat_max), (N ($preset.door_x + 0.45)), (N $preset.lon_target), $(-not $NoBlocker), [bool]$ExitStick, [bool]$Ride)

if (-not $DryRun -and -not $NoLaunch) { Start-Game }
$fail = 0
try {
    if (-not $DryRun -and (-not $NoLaunch -or $ForceSetup)) {
        # the save loads with V mounted on a bike: leave it and walk away (fresh load only)
        Invoke-Run -Tag 'setup-exit' -Scenarios @('bike_exit_only')
        Invoke-Run -Tag 'setup-walk' -Scenarios @('bike_walk_away')
        Start-Sleep -Seconds 3
    }
    foreach ($c in $run) {
        Write-Host "=== case $c"
        try { & $table[$c] }
        catch { $fail++; Write-Warning "case $c failed: $_" }
    }
}
finally {
    if (-not $DryRun) {
        try { Invoke-Cet 'cleanup' @("pcall(function() Game.GetDynamicEntitySystem():DeleteTagged('UTPPBikeBlock') end) pcall(function() GetMod('utpp_core').NightAuto(false) end) Game.UTPPLogLine('[bike] cleanup (blocker removed, NightAuto off)')") } catch { Write-Warning "cleanup failed (the game may be closed): $_" }
        Push-Location $gt
        try { & $Python run.py --padd-stop 2>&1 | Select-Object -Last 1 } catch { Write-Warning "padd stop failed: $_" } finally { Pop-Location }
        if ($CloseGame) { Stop-Process -Name Cyberpunk2077 -Force -ErrorAction SilentlyContinue }
    }
}
$summary = @("bike cases $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  vehicle=$Vehicle  blocker=$(-not $NoBlocker) exitStick=$([bool]$ExitStick) ride=$([bool]$Ride)") + @($script:results) +
    @("not mounted: $($script:notMounted)  not dismounted: $($script:notDismounted)  side mismatches: $($script:sideMismatch)  spawn skipped: $($script:spawnSkipped)  case errors: $fail")
$summary | ForEach-Object { Write-Host $_ }
if (-not $DryRun) { $summary | Set-Content -Path (Join-Path $Out 'bike-cases-summary.txt') -Encoding UTF8 }
Write-Host "done: $($run.Count - $fail) of $($run.Count) cases ran; out: $Out"
if ($fail -or $script:notMounted -or $script:notDismounted -or $script:sideMismatch -or $script:spawnSkipped) { exit 1 }

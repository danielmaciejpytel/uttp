<#
.SYNOPSIS
Car test cases for the UTPP vehicle work (entry spin, camera start, passenger, jump-out, weapon exit, first-person preference).
.DESCRIPTION
Drives run.py with scenarios_car_cases.json: launches the game (unless -NoLaunch), leaves the vehicle the save loads mounted on and
walks away (that setup runs only after a launch made by this script; -ForceSetup runs it with -NoLaunch, only right after a fresh
load: B on foot is a dodge), spawns -Vehicle (default Vehicle.v_standard2_archer_hella_player; Codeware dynamic entity, tag UTPPCar) next to V, turns on
GetMod('utpp_vehiclecam').SetCamTrace(true), runs the pad steps of each case, records the game window with ffmpeg and saves the
log lines of the case to <Out>\<case>-lines.txt. Nothing here is a pass/fail verdict: the lines below are what to look for.
Needs ViGEmBus, the game in the foreground (run.py aborts when focus is lost), CET and the UTPP log. -DryRun forwards --dry-run to
run.py (no input, no launch, no recording) and prints the plan and the CET commands.

Cases (-Cases; default 'all' = 1..7 in this order, the first-person preference case last):
  normal_cycle  (1) driver cycle from 3 approach angles: parallel (car at V's right; a second ride starts from the exit pose =
                the ride-3 geometry of the 2026-11-01 test), facing the door (car 2.6 m ahead, turned 90 deg), from behind
                (V starts 4 m behind the seat line, 1.9 m left of the car, and walks along the driver side; before
                2026-11-01 it started 1 m left with a 2.3 s walk and ended ~2 m short of the door on every car).
                The approach is a short open-loop walk (JSON) followed by position feedback (see 'Mount guard' below).
                The yaw sign of the door angle (+90) is verified: every door case of 2026-11-01 logged seat_front_left.
                Look at: 'vehtrans mount: ... rootPeakYaw=' (the unfixed hold-and-release signature is rootPeakYaw 63.4 with a
                non-monotonic yaw; after the yaw-blend fix expect a monotonic turn, 'yawAlign=1 alignAt=' when the turn passes
                30 deg), 'vehtrans seat: ... neckFire / neckStep', video of the entry turn. FAIL = visible turn about her own
                axis / pop at the entry on any of the 3 angles. Camera: '[utpp_vehiclecam]' lines and 'done (vehicle TPP'.
  fpp_persist   (2) game vehicle camera persistently FPP: ride, FPP request (vehicleRequestCameraPerspectiveEvent), exit, enter
                again. PASS (last entry): '[utpp_vehiclecam] ... obs state=0 mounted=true ... persistent=0', then 'request TPPFar
                (state=1 active=0 persistent=0' (rideBase), 'done (vehicle TPPFar)', 'handover ... keepTpp mount -> ride', no
                'failed ('. FAIL: the ride stays first person ('failed (' or 'state=1 ... active=0' after 'no request for this
                ride'). Restore: a TPPClose request is typed in the ride before the exit (also in the finally block).
  ride_toggle   (3) in-ride perspective toggle (DPAD_RIGHT) is not remembered after the exit. PASS only if ride 1 logs 'keepTpp
                ride -> game (vehicle camera FPP)' (the toggle really reached first person; without that line the case is
                inconclusive, the toggle order of the game is not verified) and ride 2 again ends in 'done (vehicle TPP...)'.
                FAIL: ride 2 starts in FPP.
  passenger     (4) passenger seat: the car is spawned at V's LEFT (same heading), so V stands at the passenger door; the seat
                switch to the driver seat is blocked with the debug blackboard flag Vehicle_BlockSwitchSeats (reset in the finally
                block; variant passenger_switch = same without the flag, regression of the seat switch). The pad takes the seat
                with X exactly like the driver (no CET mount call). Expected today: 'passenger-side entry (slot seat_front_right,
                driver seat free=true) - seat switch to the driver seat expected', 'seat final: passenger state 3 ... (no switch
                to the driver seat)', 'always third person on entry: passenger ride - request ...', 'keepTpp off -> game
                (passenger ...)', obs 'state=3 ... active' in a third-person perspective. FAIL vs the rule 'every entry starts
                in TPP': active=0 in state 3 after the passenger request window. Also read '[utpp_vehicle_anim] ride N: utpp_vehicle_tpp=1 (car mount ...)'. At the
                exit: '[utpp_camera] enter epoch=N ... Activate' without a '[utpp_vehiclecam] exit (' line (cut from first person).
  jumpout_stand (5a) QuickExit (B double tap) when stationary; jumpout_move (5b) at speed. Needs the perk
                Reflexes_Left_Milestone_1 (the log line '[car] coolExitPerk ok= true|false' is written before the case; -GrantPerk
                tries BuyNewPerk(..., true) in memory, unverified on 2.5; do not save the game afterwards) and open sky above the
                car. Without the perk the double tap is consumed silently: the case is conditional. PASS (moving): '[utpp_vehiclecam]
                exit while the vehicle moves N m/s (> 4.0): jump-out'; 'UTPP JUMPOUT vehicle bit released (flight after the
                unmount): vz>=3'; EXITDIAG exitsHanded=1 noCapture=0; status jumpOuts incremented ('[car] status' line after the
                run). FAIL: a flight on video with no JUMPOUT line (car impulse rise < 3 m/s: hanging legs until the apex);
                '[utpp_vehicle_exit] ExitWithTeleport ... bike=false' (the cool exit fell back after 1 s).
  weapon_exit   (6) exit with a weapon drawn (Y draws, 3 RB taps = hip fire without LB; a ranged weapon must be equipped in the
                save). Variants: weapon_exit_lb (LB held and released before B), weapon_preentry (weapon drawn before the entry).
                Expected: obs 'state=6' with active 4-6 (DriverCombat TPP), at the exit 'keepTpp ride -> exit', 'exit (exit start)',
                'MMWEAPON on=1 ... held=1' around the unmount, EXITDIAG once, 'MMMASK on reasons=0x4 exit=2.00'; '[car] status'
                line with Game.UTPPGetMotionStatus() (action=, hipFirePose=). Judge on video: hip-fire pose held through the stand-up
                (up to 2 s), clipping the door frame, abrupt release at the window end.
  onfoot_fpp    (7) on-foot first-person preference: GetMod('utpp_perspective').SetUserWantsTpp(false,'test'), a cycle, then
                the MANDATORY restore SetUserWantsTpp(true,'test') (finally block; the setting is persisted to the user's
                settings, check 'user preference TPP (test)' in the log). Expected: '[utpp_camera] vehicle hold refused' or
                '[utpp_vehiclecam] mount while the camera is not held-ready (camera not in third person ...)', 'always third
                person on entry: driver ride without the UTPP hold (ride from FPP) - armed', 'done (vehicle TPP...)', 'keepTpp
                off -> game (driver ride without the hold)', at the exit 'keepTpp game -> off (exit in first person (user prefers
                first person))'. FAIL: obs 'state=1 ... active=0' after 'no request for this ride'; 'head missing' with repairs > 0
                under the third-person vehicle camera; 'vehicle TPP animation' absent.
  hold_off      variant of 7: TPP on foot with the mount hold forced off (SetKeepTpp(false), restored with SetKeepTpp(true); it
                persists vehiclecam settings). PASS: 'gamelook: vehicle exit start -> look zeroing'; 'vehicle edge zeroing done'
                drift < 2 deg; EXITDIAG captures>0 and noCapture=0. FAIL: 'no look zeroing (utppTpp=0 rideCamera=tpp)', fillNoCapture=1.
  'variants' = passenger_switch, weapon_exit_lb, weapon_preentry, hold_off.
  jumpout_*     since r13 the build logs '[utpp_vehiclecam] ... jo diag t=.. car=(x,y,z) vcar=(..) onG=.. carV=..' (V in the car frame, ~20 Hz,
                from the jump-out until 1.5 s after the landing; onG is derived from V's vertical speed, not a game flag). The spawn command
                switches vehiclecam's camTrace on (no -CamTrace needed); the lines go into <case>-lines.txt, the case prints their count.
  carhit_queue_mm (and the other carhit_queue* variants; run it ALONE, without -CamTrace: multi-case runs lost the typed queue command)
                switches the body-trace capture on for the episode when it is not running yet (off again in the restore call), then
                runs bt_getup.py: Out\\<case>-getup\\ gets the trimmed bodytrace CSV, <case>-getup-window.log, getup_metric.json / .txt
                and the M1..M5 summary is printed (tool: -GetUpMetric, default tools\\ragdoll\\getup_metric.py of this checkout).

Position feedback and mount guard (car_cases_nav.py; offline tests: test_car_cases_nav.py):
  * The spawn command starts GetMod('utpp_core').NightAuto(true,0.25,'UTPPCar'): a '[night] POS' line every 0.25 s (V position,
    camera forward, car position + right). It is switched off again at the end. The lines files keep those lines, so the V pose
    at the X press can be read afterwards (car frame: lon along the car, lat > 0 = car left = driver side).
  * Every case that mounts runs split in parts (<scenario>__pre, __x = the enter step, __tail = drive/brake/exit): pre -> up to
    -NavTries (5) corrective walks until V is inside the working window of the car's preset (lon = seat y - 0.14 +- 0.25 m,
    lat between the door x and max(door x + 1.7, 2.7) m; Bratsk 2.65) -> X -> check the log for 'ride N: utpp_vehicle_tpp=1' / 'vehtrans mountdiag' ->
    if absent one retry (walk to the exact door point, or 0.35 m forward when already there, then X again) -> if still absent
    '[car] NOT MOUNTED case=<case>' is logged and the rest of the case (drive, brake, exit, B hold) is skipped, so no RT / LT /
    B input is ever sent on foot (it drew the pistol, aimed and dodged in the 2026-11-01 runs). The run.py wake nudge is only sent
    before the first part. -NoNav sends the old open-loop steps (no feedback); the mount guard stays on.
  * Log reading is rotation-safe (the UTPP log rotates at 10 MB: utpp-<start>.log -> .1.log -> .2.log): the current file and its
    rotated siblings are one stream (gt_log via car_cases_nav.py), positions are marks (file identity + offset), and every case
    looks only after the mark taken right before its spawn (no marker of an earlier run; no spawn marker = no lines file).
  * Per-car presets (Vehicle.* record, with or without _player; exact match): spawn correction that mounted on 2026-11-01 plus the
    rig seat / door numbers: archer_hella 0/0, villefort_columbus 0/0, chevalier_emperor 0/0, quadra_turbo(_r) 0.55/0,
    kaukaz_bratsk -3.49/0.56. v_sport2_porsche_911turbo has NO rig and was never run: generic geometry. An explicit -SpawnForward /
    -SpawnAway overrides the preset. An unknown record uses 0/0 and the Hella window (printed at the start).

Run commands (from the repo's tools\gametest folder or by full path):
  powershell -File car_cases.ps1 -DryRun                      plan of everything, no input
  powershell -File car_cases.ps1 -Cases normal_cycle          one case, launches the game first
  powershell -File car_cases.ps1 -Cases passenger,jumpout_move -NoLaunch -GrantPerk   game already in the world (no setup: V must already be on foot)
Output: <Out> (default Research\logs\gametest\car-cases-<stamp>): run.py output per step, <case>-<n>.mp4 (in Media\mp4\YYYY-MM-DD_gametest_<run>), <case>-lines.txt.
If a run is aborted between the FPP request and the restore of case 2, mount any car and press the camera toggle (DPAD_RIGHT) or
queue a TPP request again; case 7 restores in finally, but if the process is killed run SetUserWantsTpp(true,'test') by hand.
#>
[CmdletBinding()]
param(
    [string[]]$Cases = @('all'),
    [switch]$DryRun,
    [switch]$NoLaunch,
    [switch]$ForceSetup,
    [switch]$NoRecord,
    [switch]$GrantPerk,
    [switch]$CloseGame,
    [string]$Out = '',
    # ragdolleng: run only this variant tag (m0, m0off, m0ioff, m1off); one forced ragdoll per game launch keeps episodes independent
    [string]$RagdollVariant = '',
    [string]$Log = '',
    [string]$Python = 'python',
    [string]$Game = '',
    # TweakDB vehicle record of the test car (another car class, e.g. Vehicle.v_standard3_chevalier_emperor)
    [string]$Vehicle = 'Vehicle.v_standard2_archer_hella_player',
    # spawn correction for another body, in the CAR's frame: +SpawnForward moves the car forward, +SpawnAway moves it away from V.
    # The case geometry is tuned on the Archer Hella (seat_front_left y=0.14 m, door x=0.99 m in its rig); for another car use
    # SpawnForward = 0.14 - <its seat_front_left y> and SpawnAway = <its door x> - 0.99 (e.g. Kaukaz Bratsk -3.49 / 0.56).
    # Without the parameters the per-car preset of car_cases_nav.py applies (Hella 0/0); an explicit value overrides the preset.
    [double]$SpawnForward = 0,
    [double]$SpawnAway = 0,
    # corrective walks (position feedback) allowed before the X press; -NoNav = old open-loop approach only
    [int]$NavTries = 5,
    [switch]$NoNav,
    # every case starts at the same place: the first case of the game session stores V's position and heading (CET global
    # UTPPCarHome), later cases (also later -NoLaunch runs for other cars) teleport V back there before the spawn. Without it V
    # starts each case where the previous ride left her (2026-11-01: the Bratsk cases ran on a slope 10 m up the road).
    [switch]$NoHome,
    # minimum pause before any call that types console commands (seconds since the previous run.py call ended). 2026-11-02: 37 of 103
    # console calls were typed but never executed, almost all when they followed the previous call by 0.2-2.9 s; after
    # >= 5 s (pad runs) 43 of 44 ran. 6 s is a working assumption, not a measured threshold.
    [double]$CetGapS = 6,
    # camera.trace (utpp_camera 10 Hz camTrace lines) on for the run, off again at the end
    [switch]$CamTrace,
    # carhit_queue* cases: getup_metric.py of the get-up acceptance metric (default: tools\ragdoll\getup_metric.py of this checkout)
    [string]$GetUpMetric = ''
)

$ErrorActionPreference = 'Stop'
$null = Get-Command $Python -ErrorAction Stop
$gt = $PSScriptRoot
. (Join-Path $gt '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$scenFile = Join-Path $gt 'scenarios_car_cases.json'
$navPy = Join-Path $gt 'car_cases_nav.py'
# project root = the first parent that holds Tools\pylibs (works from Source\ and from Build\wt-*\)
$root = $null
for ($p = $gt; $p; $p = Split-Path -Parent $p) { if (Test-Path (Join-Path $p 'Tools\pylibs')) { $root = $p; break } }
if (-not $root) { throw 'project root (folder with Tools\pylibs) not found above ' + $gt }
$ffmpeg = Join-Path $root 'Tools\pylibs\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe'
if (-not $Out) { $Out = Join-Path $root ('Research\logs\gametest\car-cases-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
# Recordings go straight to Media\mp4\YYYY-MM-DD_gametest_<run> (standing project rule); logs stay in $Out.
$MediaOut = Join-Path $root ('Media\mp4\' + (Get-Date -Format 'yyyy-MM-dd') + '_gametest_' + (Split-Path $Out -Leaf))
New-Item -ItemType Directory -Force $Out | Out-Null
$inv = [Globalization.CultureInfo]::InvariantCulture
# every scenario split at its 'enter' step (<name>__pre / __x / __tail) so the mount can be checked before drive / brake / exit
$splitFile = Join-Path $Out '_scenarios_split.json'
$null = & $Python -I $navPy split $scenFile $splitFile
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $splitFile)) { throw "scenario split failed ($LASTEXITCODE)" }
$split = Get-Content $splitFile -Raw | ConvertFrom-Json
function Has-Scn([string]$n) { return [bool]$split.scenarios.PSObject.Properties[$n] }
$preset = (& $Python -I $navPy preset $Vehicle | Select-Object -Last 1) | ConvertFrom-Json
$sf = if ($PSBoundParameters.ContainsKey('SpawnForward')) { $SpawnForward } else { [double]$preset.fwd }
$sa = if ($PSBoundParameters.ContainsKey('SpawnAway')) { $SpawnAway } else { [double]$preset.away }
$settleS = if ($preset.settle) { [double]$preset.settle } else { 4.0 }
$lineRx = '\[night\] POS|NOT MOUNTED|\[car\]|\[utpp_vehiclecam\]|\[utpp_vehicle_exit\]|\[utpp_vehicle_anim\]|\[utpp_camera\]|vehtrans|JUMPOUT|EXITDIAG|MMWEAPON|MMMASK|gamelook|keepTpp|always third person|user preference|head missing|vehicle hold|\[utpp_vehicle_door\]'
$script:clip = 0
# carhit_queue* get-up metric (bt_getup.py): the log lines getup_metric.py reads ([utpp_ragdoll] episode, [night] POS, MOTION picks) and its path
$ragRx = '\[utpp_ragdoll\]|\[night\] POS|\bMotion\b|\[car\]'
$btPy = Join-Path $gt 'bt_getup.py'
$metricPy = if ($GetUpMetric) { $GetUpMetric } else { Join-Path $PSScriptRoot '..\ragdoll\getup_metric.py' }

function N([double]$v) { $v.ToString('0.###', $inv) }

function Read-Shared([string]$path) {
    $s = [IO.File]::Open($path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]"ReadWrite, Delete")
    $r = New-Object IO.StreamReader($s)
    try { return $r.ReadToEnd() } finally { $r.Dispose() }
}

# The UTPP log rotates at 10 MB (utpp-<start>.log -> .1.log -> .2.log, a new .log starts; 2026-11-01). Everything below that reads
# the log goes through car_cases_nav.py (gt_log): the CURRENT file of the newest session, a mark (opaque string: file identity +
# offset) taken before an action, and reads of the session as one stream (current + rotated siblings). Never a file size / offset.
function Invoke-NavJson([string[]]$NavArgs) {
    try { return ((& $Python -I $navPy @NavArgs | Select-Object -Last 1) | ConvertFrom-Json) } catch { Write-Warning "log helper failed: $_"; return $null }
}

function Get-NewestLog {
    if ($Log) { return $Log }
    $r = Invoke-NavJson @('newest', '--dir', (Join-Path $Game 'red4ext\logs'))
    if ($r -and $r.path) { return [string]$r.path }
    return $null
}

function Start-Game {
    # close the game gracefully (then forced), launch with -modded and wait for the world (the save loads automatically)
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
    # client rect of the game window in screen pixels: "x y w h"
    # per-monitor DPI aware first: otherwise a scaled desktop gives logical coordinates and gdigrab (physical pixels) records the wrong area (2026-11-05)
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
    return $sum + 2.0 * [Math]::Max(0, $Names.Count - 1) + 6.0   # gaps between scenarios + pad settle / log flush margin
}

# One run.py call. -Scenarios: pad steps (the pad stays plugged: --keep-pad); -Cet: console commands typed first (';;' separated);
# neither scenario nor -Wake: a CET-only call. -ScenFile: another scenario file (default = the split file of this run);
# -NoWake: no right-stick wake nudge (it moves the camera ~6 deg per call; only the first part of a case needs it).
function Invoke-Run {
    param([string]$Tag, [string[]]$Scenarios = @(), [string]$Cet = '', [switch]$Record, [string]$ScenFile = '', [switch]$NoWake)
    $scn = if ($ScenFile) { $ScenFile } else { $splitFile }
    $ra = @('run.py', '--out', (Join-Path $Out $Tag), '--keep-pad', '--no-dismiss')
    if ($Scenarios.Count) { $ra += @('--scenario-file', $scn, '--scenarios', ($Scenarios -join ',')) } else { $ra += @('--wake', 'none') }
    if ($NoWake -and $Scenarios.Count) { $ra += @('--wake', 'none') }
    if ($Cet) { $ra += @('--cet', $Cet) }
    if ($DryRun) { $ra += '--dry-run' }
    if ($Log) { $ra += @('--log', $Log) }
    $ErrorActionPreference = 'Continue'   # python warnings on stderr must not abort (exit code is checked below)
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
        if ($DryRun) { Write-Host "--- $Tag"; & $Python @ra 2>&1 | Where-Object { $_ -match '^(CET commands|estimated input time|\S+ +[0-9.]+ s |car_\w+: )' } | Out-Host }
        else { & $Python @ra 2>&1 | Select-Object -Last 3 | Out-Host }
        if ($LASTEXITCODE -ne 0) { throw "run.py failed ($LASTEXITCODE) for $Tag" }
    }
    finally {
        Pop-Location
        if ($ff) { [void]$ff.WaitForExit(([int]$secs + 20) * 1000) }
        $script:lastRunEnd = Get-Date
    }
}

function Invoke-Cet([string]$Tag, [string[]]$Cmds) { Invoke-Run -Tag $Tag -Cet ($Cmds -join ';;') }

# Lua helpers (single quotes only: the string goes through the command line and the CET console).
function Status-Line([string]$Fn) { "local _s=$Fn Game.UTPPLogLine('[car] status '..tostring(_s))" }

# Spawn the test car relative to V: R m to V's right, F m ahead, YAW deg added to V's heading. Replaces the previous UTPPCar.
function Spawn-Car([string]$Case, [double]$R, [double]$F, [double]$Yaw) {
    $lua = "local ok,e=pcall(function() local des=Game.GetDynamicEntitySystem() des:DeleteTagged('UTPPCar') local p=Game.GetPlayer() local a=p:GetWorldPosition() local r=p:GetWorldRight() local f=p:GetWorldForward() local s=DynamicEntitySpec.new() s.recordID=TweakDBID.new('$Vehicle') local yr=math.rad($(N $Yaw)) local cfx=f.x*math.cos(yr)-f.y*math.sin(yr) local cfy=f.x*math.sin(yr)+f.y*math.cos(yr) local px=a.x+r.x*($(N $R))+f.x*($(N $F)) local py=a.y+r.y*($(N $R))+f.y*($(N $F)) local dx=px-a.x local dy=py-a.y local dl=math.max(0.01,math.sqrt(dx*dx+dy*dy)) px=px+cfx*($(N $sf))+dx/dl*($(N $sa)) py=py+cfy*($(N $sf))+dy/dl*($(N $sa)) s.position=Vector4.new(px,py,a.z+0.3,1) s.orientation=EulerAngles.new(0,0,f:ToRotation().yaw+($(N $Yaw))):ToQuat() s.alwaysSpawned=true s.persistState=false s.persistSpawn=false s.tags={'UTPPCar'} return des:CreateEntity(s) end) Game.UTPPLogLine('[car] SPAWN case=$Case ok='..tostring(ok)..' '..tostring(e)) GetMod('utpp_vehiclecam').SetCamTrace(true) Game.UTPPSetVehicleVaultLog(true) pcall(function() GetMod('utpp_core').NightAuto(true,0.25,'UTPPCar') end)"
    # The spawn line must appear after the case mark: in the 2026-11-01 Bratsk run three spawn commands never reached CET (no
    # SPAWN line, no CET error) and the case walked on foot to a car that was not there. One retry, then the case is skipped.
    $script:spawnOk[$Case] = $true
    if (-not $NoHome) {
        try { Invoke-Cet "$Case-home" @("Game.UTPPLogLine('[car] CMD home case=$Case') local p=Game.GetPlayer() if UTPPCarHome==nil then local w=p:GetWorldPosition() UTPPCarHome={x=w.x,y=w.y,z=w.z,yaw=p:GetWorldYaw()} Game.UTPPLogLine('[car] HOME set case=$Case') else Game.GetTeleportationFacility():Teleport(p,Vector4.new(UTPPCarHome.x,UTPPCarHome.y,UTPPCarHome.z,1),EulerAngles.new(0,0,UTPPCarHome.yaw)) Game.UTPPLogLine('[car] HOME return case=$Case') end") }
        catch { Write-Warning "case ${Case}: HOME teleport not confirmed: $_" }
        if (-not $DryRun) { Start-Sleep -Seconds 2 }
    }
    for ($try = 1; $try -le 2; $try++) {
        if (-not $DryRun) { $script:spawnMarks[$Case] = Get-LogMark }   # case start: nav / lines look only after this point
        try { Invoke-Run -Tag "$Case-spawn$(if ($try -gt 1) { $try })" -Scenarios @('car_idle') -Cet "Game.UTPPLogLine('[car] CMD spawn case=$Case try=$try') $lua" }
        catch { Write-Warning "case ${Case}: spawn command not confirmed (try $try): $_" }
        if ($DryRun) { return }
        Start-Sleep -Seconds $settleS
        if (Test-Spawned $Case $script:spawnMarks[$Case]) { return }
        Write-Warning "case ${Case}: no '[car] SPAWN case=$Case ok=true' line after the spawn command (try $try)"
    }
    $script:spawnOk[$Case] = $false
    $script:spawnSkipped++
    Write-Warning "case ${Case}: the test car did not spawn - the case is SKIPPED and counted as a failure"
}

# true when the spawn line of this case (ok=true) is in the log after the mark; no mark (no log / helper failed) = assumed spawned
function Test-Spawned([string]$Case, [string]$Mark) {
    $lg = Get-NewestLog
    if (-not $lg -or -not $Mark) { return $true }
    $r = Invoke-NavJson @('found', '--log', $lg, '--since', $Mark, '--rx', [regex]::Escape("[car] SPAWN case=$Case ok=true"))
    return [bool]($r -and $r.found)
}
$script:spawnOk = @{}
$script:spawnSkipped = 0

# Mark at the end of the log session (rotation-safe position, see Get-NewestLog); '' = no log / helper failed.
function Get-LogMark {
    $lg = Get-NewestLog
    if (-not $lg) { return '' }
    $r = Invoke-NavJson @('logmark', '--log', $lg)
    if ($r -and $r.token) { return [string]$r.token }
    return ''
}

# case start marks: Spawn-Car takes one right before the spawn command; nav and the lines file only look after it, so a marker of
# an earlier run (same case name, other car; or the spawn that never logged) is never taken
$script:spawnMarks = @{}

function Save-Lines([string]$Case) {
    if ($DryRun) { return }
    $lg = Get-NewestLog
    if (-not $lg) { Write-Warning 'no UTPP log found'; return }
    $since = $script:spawnMarks[$Case]
    if (-not $since) { Write-Warning "no case start mark for $Case (the spawn did not run) - no lines file"; return }
    # door hold counters of the session (utpp_vehicle_door.reds): per ride expect 'ride-entry ... applied=true' + 'own close fired'
    # diagnostic only: an unconfirmed console command must not cost the case its lines file (2026-11-02 emperor / turbo passenger)
    try { Invoke-Cet "$Case-doorstatus" @("Game.UTPPLogLine('[utpp_vehicle_door] status '..tostring(Game.UTPPVehicleDoorStatus()))") }
    catch { Write-Warning "case ${Case}: door status not logged: $_" }
    $dst = Join-Path $Out "$Case-lines.txt"
    $r = Invoke-NavJson @('lines', '--log', $lg, '--since', $since, '--marker', "[car] SPAWN case=$Case ", '--rx', $lineRx, '--out', $dst)
    if ($r -and $r.ok) { Write-Host "lines -> $dst ($($r.lines) lines)" }
    else { Write-Warning "no spawn marker for $Case in the log session of $lg after the case start - no lines file" }
}

# ---- position feedback and mount guard (see the header) --------------------------------------------------------------------
# mounted = the vehicle animation / transition lines of a new ride after the mark (not the '=0 (unmount)' line)
function Test-Mounted([string]$Mark) {
    if ($DryRun) { return $true }
    $lg = Get-NewestLog
    if (-not $lg -or -not $Mark) { Write-Warning 'no UTPP log mark: mount not verifiable (assumed mounted)'; return $true }
    for ($i = 0; $i -lt 3; $i++) {
        $r = Invoke-NavJson @('found', '--log', $lg, '--since', $Mark, '--rx', 'ride \d+: utpp_vehicle_tpp=1|vehtrans mountdiag: ride=|keepTpp mount -> ride')
        if ($r -and $r.found) { return $true }
        Start-Sleep -Milliseconds 1500
    }
    return $false
}

# one helper call: mode check | retry | face. Returns the parsed JSON (state ok|move|face|mounted|nopos|nocar|far) or $null.
function Get-Nav([string]$Case, [string]$Side, [string]$Mode, [double]$Fraction = 0.85, [double]$LonBias = 0) {
    $lg = Get-NewestLog
    if (-not $lg) { return $null }
    $na = @('-I', $navPy, 'nav', '--log', $lg, '--marker', "[car] SPAWN case=$Case ", '--vehicle', $Vehicle, '--side', $Side, '--mode', $Mode, '--fraction', (N $Fraction), '--lon-bias', (N $LonBias), '--out-scen', (Join-Path $Out '_nav.json'))
    $since = $script:spawnMarks[$Case]
    if ($since) { $na += @('--since', $since) }
    try { return ((& $Python @na 2>&1 | Select-Object -Last 1) | ConvertFrom-Json) } catch { Write-Warning "nav helper failed: $_"; return $null }
}

# walk V to the door of the preset: corrective walks until the pose is inside the working window (or the tries are used up)
function Approach-Door([string]$Case, [string]$Tag, [string]$Side, [switch]$Face, [switch]$Record, [double]$LonBias = 0) {
    if ($DryRun) { Write-Host "  nav [$Case/$Side]: position feedback skipped in a dry run (reads '[night] POS' after '[car] SPAWN case=$Case ', up to $NavTries walks to lon=$($preset.lon_target) lat=+-$($preset.lat_target))"; return }
    if ($NoNav) { return }
    for ($i = 1; $i -le $NavTries; $i++) {
        $fr = if ($i -le 2) { 0.85 } elseif ($i -eq 3) { 0.6 } else { 0.4 }   # shorter walks after an overshoot (no ping-pong)
        $r = Get-Nav $Case $Side 'check' $fr $LonBias
        if (-not $r) { Write-Warning "nav [$Case]: no answer (no log) - X pressed without feedback"; return }
        Write-Host ("  nav [{0}/{1}] #{2}: state={3} lon={4} lat={5} (target lon {6})  {7}" -f $Case, $Side, $i, $r.state, (N $r.lon), (N $r.lat), (N $r.lon_target), $r.why)
        if ($r.state -eq 'ok') {
            if ($Face) {
                $f = Get-Nav $Case $Side 'face' 0.85 $LonBias
                if ($f -and $f.state -eq 'face') { Invoke-Run -Tag "$Tag-face" -Scenarios @('car_nav') -ScenFile $f.scenario -NoWake -Record:$Record }
            }
            return
        }
        if ($r.state -ne 'move') { Write-Warning "nav [$Case]: $($r.state) $($r.why) - X pressed without feedback"; return }
        Invoke-Run -Tag "$Tag-nav$i" -Scenarios @('car_nav') -ScenFile $r.scenario -NoWake -Record:$Record
    }
    Write-Warning "nav [$Case]: still outside the window after $NavTries walks - X pressed anyway"
}

# One mounting cycle of a scenario that has an 'enter' step: pre -> feedback walk -> X -> mount check (+1 retry) -> tail.
# -FromExit = the entry starts from the pose of the previous exit (no corrective walk; the mount guard and the retry stay).
# Result in $script:cycleOk (NOT the return value: the output of the called functions must never leak into it).
# $false = V is NOT in the car: the tail (drive / brake / exit / B hold) was not sent; the caller must stop its case.
function Run-Cycle {
    param([string]$Case, [string]$Scenario, [string]$Tag = '', [string]$Side = 'driver', [switch]$Face, [switch]$Record, [switch]$FromExit, [double]$LonBias = 0)
    if (-not $Tag) { $Tag = $Case }
    if ($script:spawnOk.ContainsKey($Case) -and -not $script:spawnOk[$Case]) { Write-Warning "case ${Case}: SKIPPED - the test car did not spawn"; $script:cycleOk = $false; return }
    if (-not (Has-Scn "${Scenario}__x")) { Invoke-Run -Tag $Tag -Scenarios @($Scenario) -Record:$Record; $script:cycleOk = $true; return }
    if (Has-Scn "${Scenario}__pre") { Invoke-Run -Tag "$Tag-pre" -Scenarios @("${Scenario}__pre") -Record:$Record }
    if (-not $FromExit) { Approach-Door -Case $Case -Tag $Tag -Side $Side -Face:$Face -Record:$Record -LonBias $LonBias }   # -FromExit: V stands where the exit left her (the geometry under test), no walk
    $off = Get-LogMark
    Invoke-Run -Tag "$Tag-x" -Scenarios @("${Scenario}__x") -NoWake -Record:$Record
    if (-not (Test-Mounted $off)) {
        Write-Warning "case $Case ($Scenario): no mount line after X - one retry (walk to the exact door point, X again)"
        $r = if ($NoNav) { $null } else { Get-Nav $Case $Side 'retry' 0.85 $LonBias }
        if ($r -and $r.state -eq 'move') {
            Write-Host ("  nav retry [{0}]: lon={1} lat={2} -> walk {3} s" -f $Case, (N $r.lon), (N $r.lat), (N $r.dur))
            Invoke-Run -Tag "$Tag-retrynav" -Scenarios @('car_nav') -ScenFile $r.scenario -NoWake -Record:$Record
        } elseif ($r) { Write-Host "  nav retry [$Case]: state=$($r.state) $($r.why)" }
        Invoke-Run -Tag "$Tag-x2" -Scenarios @("${Scenario}__x") -NoWake -Record:$Record
        if (-not (Test-Mounted $off)) {
            Write-Warning "case $Case ($Scenario): NOT MOUNTED after 2 X presses - the rest of the case is skipped (no drive / brake / exit on foot)"
            try { Invoke-Cet "$Tag-notmounted" @("Game.UTPPLogLine('[car] NOT MOUNTED case=$Case scenario=$Scenario after 2 X presses')") } catch { Write-Warning "log line failed: $_" }
            $script:notMounted++
            $script:cycleOk = $false
            return
        }
    }
    if (Has-Scn "${Scenario}__tail") { Invoke-Run -Tag "$Tag-tail" -Scenarios @("${Scenario}__tail") -NoWake -Record:$Record }
    $script:cycleOk = $true
}
$script:notMounted = 0
$script:cycleOk = $true

$perkProbe = "local ok,v=pcall(function() return PlayerDevelopmentSystem.GetData(Game.GetPlayer()):IsNewPerkBoughtAnyLevel(gamedataNewPerkType.Reflexes_Left_Milestone_1) end) Game.UTPPLogLine('[car] coolExitPerk ok='..tostring(ok)..' '..tostring(v))"
$perkGrant = "local ok,v=pcall(function() local d=PlayerDevelopmentSystem.GetData(Game.GetPlayer()) if d:IsNewPerkBoughtAnyLevel(gamedataNewPerkType.Reflexes_Left_Milestone_1) then return 'already' end return d:BuyNewPerk(gamedataNewPerkType.Reflexes_Left_Milestone_1,true) end) Game.UTPPLogLine('[car] coolExitPerk grant ok='..tostring(ok)..' '..tostring(v))"
$fppEvent = "local ev=vehicleRequestCameraPerspectiveEvent.new() ev.cameraPerspective=Enum.new('vehicleCameraPerspective','FPP') Game.GetPlayer():QueueEvent(ev) Game.UTPPLogLine('[car] FPP vehicle camera request queued')"
$tppEvent = "local ev=vehicleRequestCameraPerspectiveEvent.new() ev.cameraPerspective=Enum.new('vehicleCameraPerspective','TPPClose') Game.GetPlayer():QueueEvent(ev) Game.UTPPLogLine('[car] TPPClose vehicle camera request queued (restore)')"
$blockSeats = "local d=Game.GetAllBlackboardDefs().DebugData Game.GetBlackboardSystem():Get(d):SetBool(d.Vehicle_BlockSwitchSeats,true) Game.UTPPLogLine('[car] Vehicle_BlockSwitchSeats=true')"
$freeSeats = "local d=Game.GetAllBlackboardDefs().DebugData Game.GetBlackboardSystem():Get(d):SetBool(d.Vehicle_BlockSwitchSeats,false) Game.UTPPLogLine('[car] Vehicle_BlockSwitchSeats=false (restored)')"

function Case-NormalCycle {
    # behind: V starts 4 m behind the seat line (F 4.0 + the preset's forward correction = seat y - 4.14) and 1.9 m left of the car
    # (R 1.9: clear of the body; the old R 1.0 rubbed the side), walks along the driver side and the feedback finishes the walk.
    $angles = @(
        @{ n = 'parallel'; r = 2.6; f = 0.0; y = 0.0; s = @('car_cycle_parallel', 'car_cycle'); face = $false },   # second ride = ride-3 geometry
        @{ n = 'door'; r = 0.5; f = 2.6; y = 90.0; s = @('car_cycle_door'); face = $false; bias = 0.5 },   # bias: V's lon is seat + 0.36 here (mounted on all 5 cars)
        @{ n = 'behind'; r = 1.9; f = 4.0; y = 0.0; s = @('car_cycle_behind'); face = $true }
    )
    foreach ($a in $angles) {
        $c = "normal_cycle_$($a.n)"
        Spawn-Car $c $a.r $a.f $a.y
        foreach ($sc in $a.s) {
            $tg = if ($sc -eq $a.s[0]) { $c } else { "$c-ride2" }
            Run-Cycle -Case $c -Scenario $sc -Tag $tg -Face:$a.face -FromExit:($sc -ne $a.s[0]) -LonBias ([double]$a.bias) -Record
            if (-not $script:cycleOk) { break }
        }
        Save-Lines $c
    }
}

function Case-FppPersist {
    $c = 'fpp_persist'
    Spawn-Car $c 2.6 0 0
    try {
        Run-Cycle -Case $c -Scenario 'car_enter_parallel' -Tag "$c-1" -Record      # ride 1: enter, seated
        if (-not $script:cycleOk) { return }
        Invoke-Cet "$c-fpp" @($fppEvent)                                          # persistent FPP while mounted
        Start-Sleep -Seconds 2
        Invoke-Run -Tag "$c-2" -Scenarios @('car_exit')
        Run-Cycle -Case $c -Scenario 'car_ride' -Tag "$c-3" -FromExit -Record     # entry under test
        if (-not $script:cycleOk) { return }
        Invoke-Cet "$c-restore" @($tppEvent)                                      # restore inside the ride, before the exit
        Invoke-Run -Tag "$c-4" -Scenarios @('car_exit')
    }
    finally {
        try { Invoke-Cet "$c-restore2" @($tppEvent) } catch { Write-Warning "FPP restore failed: $_" }
        Save-Lines $c
    }
}

function Case-RideToggle {
    $c = 'ride_toggle'
    Spawn-Car $c 2.6 0 0
    # ride 1 toggles, ride 2 must start in TPP again
    Run-Cycle -Case $c -Scenario 'car_cycle_toggle' -Tag "$c-ride1" -Record
    if ($script:cycleOk) { Run-Cycle -Case $c -Scenario 'car_cycle' -Tag "$c-ride2" -FromExit -Record }
    Save-Lines $c
}

function Case-Passenger([string]$Variant) {
    $c = $Variant
    Spawn-Car $c -2.6 0 0
    try {
        if ($Variant -eq 'passenger') { Invoke-Cet "$c-block" @($blockSeats) }
        Run-Cycle -Case $c -Scenario 'car_passenger' -Side passenger -Record
    }
    finally {
        if ($Variant -eq 'passenger') { try { Invoke-Cet "$c-free" @($freeSeats) } catch { Write-Warning "seat flag reset failed: $_" } }
        Save-Lines $c
    }
}

function Case-JumpOut([string]$Variant) {
    $c = $Variant
    $scn = if ($Variant -eq 'jumpout_stand') { 'car_jump_stand' } else { 'car_jump_move' }
    Spawn-Car $c 2.6 0 0
    $pre = @($perkProbe)
    if ($GrantPerk) { $pre = @($perkGrant, $perkProbe) }
    Invoke-Cet "$c-perk" $pre
    Write-Host "$c needs the perk Reflexes_Left_Milestone_1 (see the [car] coolExitPerk line) and open sky above the car"
    Run-Cycle -Case $c -Scenario $scn -Record
    Invoke-Cet "$c-status" @((Status-Line 'Game.UTPPGetStatus()'))
    Save-Lines $c
    # r13 jump-out push diagnostic: '[utpp_vehiclecam] ... jo diag t=.. car=(x,y,z) vcar=(..) onG= carV=' (camTrace is switched on by the spawn command)
    if (-not $DryRun) {
        $lf = Join-Path $Out "$c-lines.txt"
        if (Test-Path $lf) {
            $n = @(Select-String -Path $lf -Pattern 'jo diag' -SimpleMatch).Count
            Write-Host "  jo diag lines in $c-lines.txt: $n"
            if ($n -eq 0) { Write-Warning "case ${c}: no 'jo diag' line (no jump-out happened, the perk is missing, camTrace off, or the build has no jo diag)" }
        }
    }
}

function Case-Weapon([string]$Variant) {
    $c = $Variant
    $scn = switch ($Variant) { 'weapon_exit' { 'car_weapon_exit' } 'weapon_exit_lb' { 'car_weapon_exit_lb' } default { 'car_weapon_preentry' } }
    Spawn-Car $c 2.6 0 0
    Run-Cycle -Case $c -Scenario $scn -Record
    Invoke-Cet "$c-status" @((Status-Line 'Game.UTPPGetMotionStatus()'))
    Save-Lines $c
}

function Case-Pref([string]$Variant) {
    $c = $Variant
    $set = if ($Variant -eq 'onfoot_fpp') { "GetMod('utpp_perspective').SetUserWantsTpp(false,'test')" } else { "GetMod('utpp_vehiclecam').SetKeepTpp(false)" }
    $rest = if ($Variant -eq 'onfoot_fpp') { "GetMod('utpp_perspective').SetUserWantsTpp(true,'test')" } else { "GetMod('utpp_vehiclecam').SetKeepTpp(true)" }
    try {
        Invoke-Cet "$c-set" @("local _r=$set Game.UTPPLogLine('[car] $Variant preference set')")
        Spawn-Car $c 2.6 0 0
        Run-Cycle -Case $c -Scenario 'car_cycle_parallel' -Record
    }
    finally {
        try { Invoke-Cet "$c-restore" @("local _r=$rest Game.UTPPLogLine('[car] $Variant preference restored')") }
        catch { Write-Warning "MANDATORY RESTORE FAILED for $Variant - run by hand in CET: $rest" }
        Save-Lines $c
    }
}

function Case-Shoulders {
    # on foot, no car: footage of the shoulder width (unarmed, armed, aim, hip fire, reload, sprint) plus the MM v2 status line
    Run-Plain -Tag 'shoulders' -Scenario 'onfoot_shoulders'
}

function Case-GetUp {
    foreach ($o in @('0', '1.2', '3', '6')) {
        Invoke-Run -Tag "getup-$o" -Scenarios @('getup_probe_hold') -Cet "Game.UTPPLogLine('[car] getup start '..tostring(Game.UTPPRagdollGetUpProbe($o, 5)))" -Record
        Invoke-Cet "getup-$o-status" @((Status-Line 'Game.UTPPRagdollGetUpProbeStatus()'))
    }
    Invoke-Cet 'getup-mmv2' @((Status-Line 'Game.UTPPGetMotionStatus()'))
}

function Case-RagdollEngine {
    # ragdoll R1: engine ragdoll on V (needs the asset package with the RagdollComponent; without it every force line is 'refused').
    # V stands still on foot in TPP; getup_probe_hold = camera orbit right / left + still (9 s) covers the reds 8 s watchdog.
    # Impulse 30 = the vanilla thrown-NPC value (carriedObject.script: direction * 30..100). Native lines: '[utpp_ragdoll] eng ...'.
    # Note: UTPPSetEnabled(false) does not stop mm.v1 / mm.motionDriver pose writes (see 'eng start ... before: utpp= mm1= mm2='); this case leaves
    # them as they are. An engine-only pose needs BOTH off (v2 off only hands over to v1): GetMod('utpp_motion').SetEnabled(false) +
    # Game.UTPPSetMMEnabled(false), restored with SetEnabled(true) + GetMod('utpp_core').ReapplyMM('test').
    Invoke-Cet 'ragdolleng-status' @((Status-Line 'Game.UTPPRagdollEngineStatus()'))
    # mmOff: mm.motionDriver (CET glue, applied on its next sync) and mm.v1 (native) off for the run, so the visible pose is the engine's;
    # restored directly (ReapplyMM would not re-enable: utpp_core still believes mm is on).
    $mmOff = "GetMod('utpp_motion').SetEnabled(false) Game.UTPPSetMMEnabled(false) "
    $mmOn = "GetMod('utpp_motion').SetEnabled(true) Game.UTPPSetMMEnabled(true) Game.UTPPLogLine('[car] ragdolleng mm restored')"
    foreach ($v in @(@{ tag = 'm0'; mode = 0; imp = 0; off = $false }, @{ tag = 'm0off'; mode = 0; imp = 0; off = $true },
                     @{ tag = 'm0ioff'; mode = 0; imp = 30; off = $true }, @{ tag = 'm1off'; mode = 1; imp = 0; off = $true })) {
        if ($RagdollVariant -and $v.tag -ne $RagdollVariant) { continue }
        $t = "ragdolleng-$($v.tag)"
        $pre = if ($v.off) { $mmOff } else { '' }
        try {
            Invoke-Run -Tag $t -Scenarios @('getup_probe_hold') -Cet "$pre Game.UTPPLogLine('[car] ragdolleng start $($v.tag) '..tostring(Game.UTPPRagdollEngineForce($($v.mode), $($v.imp))))" -Record
            Invoke-Cet "$t-status" @((Status-Line 'Game.UTPPRagdollEngineStatus()'))
        }
        finally {
            try { Invoke-Cet "$t-stop" @((Status-Line 'Game.UTPPRagdollEngineStop()')) }
            catch { Write-Warning "MANDATORY STOP FAILED for $t - run by hand in CET: Game.UTPPRagdollEngineStop() (the reds watchdog also stops after 8 s)" }
            if ($v.off) {
                try { Invoke-Cet "$t-mmon" @($mmOn) }
                catch { Write-Warning "MM RESTORE FAILED for $t - run by hand in CET: $mmOn" }
            }
        }
    }
}
# ---- get-up metric of the carhit_queue* cases (bt_getup.py) ------------------------------------------------------------------
# The continuous body-trace capture (Game.UTPPBodyPoseTraceContinuous, red4ext\logs\utpp-bodytrace-<time>.csv, ~250 MB / min) is switched
# on for the episode only when it is not running yet ("Turn all logs on" runs it for the whole session: then it stays as it is) and
# off again in the restore call. After the case: the CSV of this run is trimmed to the episode window (Out\<case>-getup\), the log
# window of the case is saved next to it and getup_metric.py prints / stores M1..M5 (getup_metric.json, getup_metric.txt).
# The decision "capture already running (Turn all logs on / earlier call)" is taken in the game: core.allLogs.on or PSM status 'active=1'
# (the continuous writer keeps the capture armed). Only a capture switched on here is switched off again (CET globals UTPPCarBtOwned /
# UTPPCarMmOwned, cleared by the off call). mm.trace is switched on (session only, noSave) when it is off: getup_metric.py M4 reads
# the 'UTPP MOTION ... db= clip=' lines, which exist only with mm.trace.
$btOnCmd = "local ok,e=pcall(function() local c=GetMod('utpp_core') local st=tostring(Game.UTPPBodyPoseTraceStatus()) local was=(c and c.allLogs and c.allLogs.on==true) or st:find('active=1',1,true)~=nil local r='already running (left on)' UTPPCarBtOwned=false if not was then UTPPCarBtOwned=true r=tostring(Game.UTPPBodyPoseTraceContinuous(true)) end UTPPCarMmOwned=false if c and c.Effective('mm.trace')~=true then UTPPCarMmOwned=true c.Set('mm.trace',true,{noSave=true}) r=r..' mmtrace=on' end return r end) Game.UTPPLogLine('[car] bodytrace on ok='..tostring(ok)..' '..tostring(e))"
$btOffCmd = "pcall(function() if UTPPCarBtOwned then Game.UTPPBodyPoseTraceContinuous(false) UTPPCarBtOwned=false end if UTPPCarMmOwned then GetMod('utpp_core').Set('mm.trace',false,{noSave=true}) UTPPCarMmOwned=false end end) Game.UTPPLogLine('[car] bodytrace off (only what the case switched on)')"

# snapshot of the body-trace CSVs (names, sizes, growing = capture already running) before the case; $null in a dry run / on failure
function Bt-Snapshot([string]$Case) {
    if ($DryRun) { return $null }
    $dir = Join-Path $Out "$Case-getup"
    New-Item -ItemType Directory -Force $dir | Out-Null
    try {
        $json = (& $Python -I $btPy snap --dir (Join-Path $Game 'red4ext\logs') --wait 2 | Select-Object -Last 1)
        if ($LASTEXITCODE -ne 0 -or -not $json) { throw "bt_getup snap exit $LASTEXITCODE" }
        Set-Content -Path (Join-Path $dir 'bt_snapshot.json') -Value $json -Encoding ASCII
        return ($json | ConvertFrom-Json)
    }
    catch { Write-Warning "body-trace snapshot failed ($_): the capture is switched on and off by this case"; return $null }
}

# after the case (capture switched off, case lines saved): trimmed CSV + log window -> getup_metric.py; prints the M1..M5 summary.
# A FAIL verdict is not an error of the harness (exit code 1 of the metric); only missing inputs warn.
function Collect-GetUp([string]$Case) {
    if ($DryRun) {
        Write-Host "  get-up metric [$Case]: skipped in a dry run (after the case: bt_getup.py collect -> $Out\$Case-getup\getup_metric.json, tool $metricPy)"
        return
    }
    $lg = Get-NewestLog
    $since = $script:spawnMarks[$Case]
    if (-not $lg -or -not $since) { Write-Warning "get-up metric [$Case]: no log / case start mark - skipped"; return }
    $dir = Join-Path $Out "$Case-getup"
    New-Item -ItemType Directory -Force $dir | Out-Null
    $on = Invoke-NavJson @('found', '--log', $lg, '--since', $since, '--rx', '\[car\] bodytrace on ok=true (true|already running)')
    if (-not ($on -and $on.found)) { Write-Warning "get-up metric [$Case]: no '[car] bodytrace on ok=true ...' line - the capture command did not run or was refused (typed but not executed?); the CSV may be missing" }
    Start-Sleep -Seconds 3   # the writer thread drains and closes the CSV after the off call
    $win = Join-Path $dir "$Case-getup-window.log"
    $w = Invoke-NavJson @('lines', '--log', $lg, '--since', $since, '--marker', "[car] SPAWN case=$Case ", '--rx', $ragRx, '--out', $win)
    if (-not ($w -and $w.ok)) { Write-Warning "get-up metric [$Case]: no log window (spawn marker not found after the case start mark) - skipped"; return }
    Write-Host "  get-up log window -> $win ($($w.lines) lines)"
    $ca = @('-I', $btPy, 'collect', '--dir', (Join-Path $Game 'red4ext\logs'), '--out', $dir, '--log-window', $win, '--metric', $metricPy)
    $snapFile = Join-Path $dir 'bt_snapshot.json'
    if (Test-Path $snapFile) { $ca += @('--snap', $snapFile) }
    foreach ($d in @(Get-ChildItem (Join-Path $Out $Case) -Filter 'carhit_still_r*.txt' -ErrorAction SilentlyContinue)) { $ca += @('--digest', $d.FullName) }
    $ErrorActionPreference = 'Continue'   # python warnings on stderr must not abort (the exit code is checked below)
    & $Python @ca 2>&1 | Out-Host
    # 2026-11-05 r13f/r13h: with the continuous capture left running the CSV rows of the episode may not be flushed yet (exit 2 in the run,
    # PASS on the same data after the game closed): up to two retries 15 s apart
    for ($rt = 1; $rt -le 2 -and $LASTEXITCODE -eq 2; $rt++) { Write-Host "  get-up metric [$Case]: input not ready (exit 2), retry $rt in 15 s"; Start-Sleep -Seconds 15; & $Python @ca 2>&1 | Out-Host }
    if ($LASTEXITCODE -ne 0) { Write-Warning "get-up metric [$Case]: bt_getup.py exit $LASTEXITCODE (no usable CSV / log, or metric input error) - see above" }
}

function Case-CarHit([string]$Variant) {
    # Car hit on V. carhit_queue: the engine's OnCarHitPlayer event queued on V from CET (car id of the spawned test car, the car
    # in front of V, impulse away from it), MM v1+v2 off so the visible pose is the game's. carhit_real: the test car 10 m ahead
    # facing V is woken and pushed toward V with VehicleObject.AddCollisionForce (whether that moves a parked car is under test).
    # Read '[utpp_ragdoll] carhit' / 'carhit+t' (wrap of OnCarHitPlayer in the ragdoll probe reds) and '[car] carhit' lines.
    $c = $Variant
    $god = "pcall(function() Game.GetGodModeSystem():AddGodMode(Game.GetPlayer():GetEntityID(), gameGodModeType.Immortal, 'utpp_carhit') end) "
    $ungod = "pcall(function() Game.GetGodModeSystem():RemoveGodMode(Game.GetPlayer():GetEntityID(), gameGodModeType.Immortal, 'utpp_carhit') end) "
    $mmOff = "GetMod('utpp_motion').SetEnabled(false) Game.UTPPSetMMEnabled(false) "
    $mmOn = "GetMod('utpp_motion').SetEnabled(true) Game.UTPPSetMMEnabled(true) Game.UTPPLogLine('[car] carhit mm restored')"
    $car = "local v=Game.GetDynamicEntitySystem():GetTagged('UTPPCar')[1] local p=Game.GetPlayer() "
    if ($Variant -like 'carhit_queue*') {
        # the event goes through the DelaySystem (6 s) so the CET console is closed when V is hit (2026-11-03 probe 1: a hit queued
        # straight from the console kept the console on screen for the whole knockdown); carhit_queue_mm keeps MM on
        $off = $Variant -ne 'carhit_queue_mm'
        # carhit_queue_nopush: MM off and ragdoll.getUpPush=false for this session (the knockdown recovery clip is not pushed at
        # the release; A/B of the crouch after the get-up), restored to true in the finally block
        $nopush = $Variant -eq 'carhit_queue_nopush'
        $cfg = if ($nopush) { "Game.UTPPLogLine('[car] carhit getUpPush=false ok='..tostring(Game.UTPPRagdollSetConfig(true, 1.0, 2.5, false))) " } else { '' }
        Spawn-Car $c 0 6 180
        # get-up metric: body-trace capture on for the episode (only when it is not running yet), see Bt-Snapshot / Collect-GetUp
        $null = Bt-Snapshot $c   # names / sizes before the case: only used to pick this run's CSV afterwards
        # typed in its own call before the recorded still run (2026-11-03 probe 2: typing the ~700 characters took ~11 s of a 16 s clip)
        # state probe on the '[night] POS' lines (session only): PSM Locomotion / LocomotionDetailed / UpperBody / HighLevel / Vision /
        # Takedown and the applied status effect records, to name the pose V is left in after the get-up
        $diag = @'
local c=GetMod('utpp_core') if c and not c.rgDiag then local o=c.NightPosLine c.rgDiag=true c.NightPosLine=function(t) local l=o(t) local ok,x=pcall(function() local p=Game.GetPlayer() local d=GetAllBlackboardDefs().PlayerStateMachine local b=Game.GetBlackboardSystem():GetLocalInstanced(p:GetEntityID(),d) local function g(n) if d[n]==nil then return 'x' end return tostring(b:GetInt(d[n])) end local s='psm L='..g('Locomotion')..' LD='..g('LocomotionDetailed')..' UB='..g('UpperBody')..' HL='..g('HighLevel')..' V='..g('Vision')..' TK='..g('Takedown')..' se=' for _,e in ipairs(Game.GetStatusEffectSystem():GetAppliedEffects(p:GetEntityID()) or {}) do s=s..TDBID.ToStringDEBUG(e:GetRecord():GetID())..',' end return s end) return (l or '[night] POS -')..' '..(ok and x or ('diagerr '..tostring(x))) end end
'@
        $q = "$car local ok,e=pcall(function() local f=p:GetWorldForward() local ev=OnCarHitPlayer.new() ev.hitDirection=Vector4.new(-f.x,-f.y,0,0) ev.seperationImpulse=Vector4.new(-f.x*6,-f.y*6,1,0) if v then ev.carId=v:GetEntityID() end Game.GetDelaySystem():DelayEvent(p, ev, 6.0, false) end) Game.UTPPLogLine('[car] carhit queued +6s ok='..tostring(ok)..' '..tostring(e)..' car='..tostring(v~=nil)..' mmOff=$off')"
        try {
            Invoke-Cet "$c-diag" @($diag)
            Invoke-Cet "$c-bton" @($btOnCmd)
            Invoke-Cet "$c-queue" @("$god " + $(if ($off) { $mmOff } else { '' }) + " $cfg$q")
            Invoke-Run -Tag $c -Scenarios @('carhit_still') -NoWake -Record
        }
        finally {
            $r = if ($off) { "$ungod $mmOn" } else { $ungod }
            $r += " $btOffCmd"
            if ($nopush) { $r += " Game.UTPPLogLine('[car] carhit getUpPush restored ok='..tostring(Game.UTPPRagdollSetConfig(true, 1.0, 2.5, true)))" }
            try { Invoke-Cet "$c-restore" @($r) } catch { Write-Warning "RESTORE FAILED for $c - run by hand in CET: $r" }
        }
    }
    else {
        Spawn-Car $c 0 $(if ($Variant -eq 'carhit_drive') { 8 } else { 10 }) 180
        $push ="$car local ok,e=pcall(function() local a=v:GetWorldPosition() local b=p:GetWorldPosition() local dx=b.x-a.x local dy=b.y-a.y local l=math.sqrt(dx*dx+dy*dy) local m=v:GetTotalMass() v:PhysicsWakeUp() v:AddCollisionForce(Vector4.new(dx/l*m*14,dy/l*m*14,0,0)) local u=v:GetLinearVelocity() return 'mass='..m..' dist='..l..' vel0='..u.x..','..u.y end) Game.UTPPLogLine('[car] carhit push ok='..tostring(ok)..' '..tostring(e))"
        if ($Variant -eq 'carhit_drive') {
            # carhit_drive: the car really drives into V (engine collision -> OnCarHitPlayer with the real impulse). A driverless
            # AI chase of V (distance 0..0.5 m, forced start 10 m/s) queued 6 s ahead through the DelaySystem, so the console is
            # closed at the impact. Not used: the autonomous drive-to-point heads for the road network (r13d run: the car left V and
            # ended 178 m away) and PhysicalImpulseEvent does not move a vehicle (same run, velocity 0). The command is typed before the recorded still run
            # (typing ~1 kB takes longer than the whole recording), and the get-up metric runs as for carhit_queue_mm.
            $push = "$car local ok,e=pcall(function() local a=v:GetWorldPosition() local b=p:GetWorldPosition() local l=math.sqrt((b.x-a.x)^2+(b.y-a.y)^2) v:TurnVehicleOn(true) v:TurnEngineOn(true) v:PhysicsWakeUp() local c=AIVehicleChaseCommand.new() c.target=p c.distanceMin=0.0 c.distanceMax=0.5 c.forcedStartSpeed=10.0 c.ignoreChaseVehiclesLimit=true c.boostDrivingStats=true c.needDriver=false local ev=AICommandEvent.new() ev.command=c Game.GetDelaySystem():DelayEvent(v,ev,6.0,false) return 'mass='..v:GetTotalMass()..' dist='..l..' chase delay=6' end) Game.UTPPLogLine('[car] carhit push ok='..tostring(ok)..' '..tostring(e))"
            $probe = "$car local ok,e=pcall(function() local u=v:GetLinearVelocity() local a=v:GetWorldPosition() local b=p:GetWorldPosition() return 'vel='..u.x..','..u.y..' dist='..math.sqrt((b.x-a.x)^2+(b.y-a.y)^2) end) Game.UTPPLogLine('[car] carhit after ok='..tostring(ok)..' '..tostring(e))"
            $null = Bt-Snapshot $c
            try {
                # same path as the spawn command (a short run with -Cet): two bare Invoke-Cet calls right after the spawn were
                # typed but never executed in the 2026-11-04 22:58 run (no bodytrace / push line, the car stayed parked)
                Invoke-Run -Tag "$c-push" -Scenarios @('car_idle') -Cet "$btOnCmd $god $push"
                Invoke-Run -Tag $c -Scenarios @('carhit_still') -NoWake -Record
                Invoke-Cet "$c-after" @($probe)
            }
            finally {
                try { Invoke-Cet "$c-restore" @("$ungod $btOffCmd") } catch { Write-Warning "RESTORE FAILED for $c - run by hand in CET: $ungod $btOffCmd" }
            }
            Save-Lines $c
            Collect-GetUp $c
            return
        }
        $probe = "$car local ok,e=pcall(function() local u=v:GetLinearVelocity() local a=v:GetWorldPosition() local b=p:GetWorldPosition() return 'vel='..u.x..','..u.y..' dist='..math.sqrt((b.x-a.x)^2+(b.y-a.y)^2) end) Game.UTPPLogLine('[car] carhit after ok='..tostring(ok)..' '..tostring(e))"
        try {
            Invoke-Run -Tag $c -Scenarios @('getup_probe_hold') -Cet "$god $push" -Record
            Invoke-Cet "$c-after" @($probe)
        }
        finally {
            try { Invoke-Cet "$c-restore" @($ungod) } catch { Write-Warning "RESTORE FAILED for $c - run by hand in CET: $ungod" }
        }
    }
    Save-Lines $c
    if ($Variant -like 'carhit_queue*') { Collect-GetUp $c }
}

function Run-Plain([string]$Tag, [string]$Scenario) {
    Invoke-Run -Tag $Tag -Scenarios @($Scenario) -Record
    Invoke-Cet "$Tag-status" @((Status-Line 'Game.UTPPGetMotionStatus()'))
}

$table = [ordered]@{
    shoulders = { Case-Shoulders }
    getup = { Case-GetUp }
    ragdolleng = { Case-RagdollEngine }
    carhit_queue = { Case-CarHit 'carhit_queue' }
    carhit_queue_mm = { Case-CarHit 'carhit_queue_mm' }
    carhit_queue_nopush = { Case-CarHit 'carhit_queue_nopush' }
    carhit_real = { Case-CarHit 'carhit_real' }
    carhit_drive = { Case-CarHit 'carhit_drive' }
    normal_cycle = { Case-NormalCycle }
    fpp_persist = { Case-FppPersist }
    ride_toggle = { Case-RideToggle }
    passenger = { Case-Passenger 'passenger' }
    jumpout_stand = { Case-JumpOut 'jumpout_stand' }
    jumpout_move = { Case-JumpOut 'jumpout_move' }
    weapon_exit = { Case-Weapon 'weapon_exit' }
    onfoot_fpp = { Case-Pref 'onfoot_fpp' }
    passenger_switch = { Case-Passenger 'passenger_switch' }
    weapon_exit_lb = { Case-Weapon 'weapon_exit_lb' }
    weapon_preentry = { Case-Weapon 'weapon_preentry' }
    hold_off = { Case-Pref 'hold_off' }
}
$mainSet = @('normal_cycle', 'fpp_persist', 'ride_toggle', 'passenger', 'jumpout_stand', 'jumpout_move', 'weapon_exit', 'onfoot_fpp')
$variantSet = @('passenger_switch', 'weapon_exit_lb', 'weapon_preentry', 'hold_off')
$run = @()
foreach ($c in @($Cases | ForEach-Object { $_ -split ',' } | Where-Object { $_ })) {   # -File passes 'a,b' as one string
    if ($c -eq 'all') { $run += $mainSet }
    elseif ($c -eq 'variants') { $run += $variantSet }
    elseif ($table.Contains($c)) { $run += $c }
    else { throw "unknown case '$c' (known: $($table.Keys -join ', '), all, variants)" }
}
# the first-person preference cases change a persisted setting: keep them last
$run = @($run | Where-Object { $_ -notin @('onfoot_fpp', 'hold_off') }) + @($run | Where-Object { $_ -in @('onfoot_fpp', 'hold_off') })
Write-Host ('cases: ' + ($run -join ', ') + "   out: $Out")
Write-Host ("vehicle: $Vehicle  preset={0} ({1}{2})  spawn correction forward={3} away={4}{5}  settle={6} s  door window: lon {7} +-0.25, lat {8}..{9} (door side); rig seat y={10} door x={11}" -f `
    $preset.key, $preset.label, $(if (-not $preset.known) { ', NOT a known preset' } else { '' }), (N $sf), (N $sa),
    $(if ($PSBoundParameters.ContainsKey('SpawnForward') -or $PSBoundParameters.ContainsKey('SpawnAway')) { ' (explicit parameter)' } else { '' }),
    (N $settleS), (N $preset.lon_target), (N ($preset.door_x - 0.1)), (N $preset.lat_max), (N $preset.seat_y), (N $preset.door_x))

if (-not $DryRun -and -not $NoLaunch) { Start-Game }
$fail = 0
try {
    if (-not $DryRun -and (-not $NoLaunch -or $ForceSetup)) {
        # the save loads with V mounted on a bike: leave it and walk away from it (fresh load only)
        Invoke-Run -Tag 'setup-exit' -Scenarios @('car_exit_only')
        Invoke-Run -Tag 'setup-walk' -Scenarios @('car_walk_away')
        Start-Sleep -Seconds 3
    }
    if ($CamTrace -and -not $DryRun) { Invoke-Cet 'camtrace-on' @("GetMod('utpp_core').Set('camera.trace', true) Game.UTPPLogLine('[car] camera.trace on')") }
    foreach ($c in $run) {
        Write-Host "=== case $c"
        try { & $table[$c] }
        catch { $fail++; Write-Warning "case $c failed: $_" }
    }
}
finally {
    if (-not $DryRun) {
        if ($CamTrace) { try { Invoke-Cet 'camtrace-off' @("GetMod('utpp_core').Set('camera.trace', false) Game.UTPPLogLine('[car] camera.trace off')") } catch { Write-Warning "camera.trace off failed: $_" } }
        try { Invoke-Cet 'nightauto-off' @("pcall(function() GetMod('utpp_core').NightAuto(false) end)") } catch { Write-Warning "NightAuto off failed (the game may be closed): $_" }
        Push-Location $gt
        try { & $Python run.py --padd-stop 2>&1 | Select-Object -Last 1 } catch { Write-Warning "padd stop failed: $_" } finally { Pop-Location }
        if ($CloseGame) { Stop-Process -Name Cyberpunk2077 -Force -ErrorAction SilentlyContinue }
    }
}
Write-Host "done: $($run.Count - $fail) of $($run.Count) cases ran; mount failures (NOT MOUNTED): $($script:notMounted); spawn skipped: $($script:spawnSkipped); out: $Out"
if ($fail -or $script:notMounted -or $script:spawnSkipped) { exit 1 }

# Car entry clips for the player rigs (female pwa / male pma), one class at a time (standard2 = Archer Hella basic, sport2): TPP 'enter'
# for the driver (seat_front_left) and the front passenger (seat_front_right) that END in the vanilla FPP enter's seated end pose
# (the game continues with the seated idle from it: 0 cm seat step in the logs) and START where the vanilla clip starts (the engine
# slides the entity to seat * inverse(root delta) before the clip plays).
# Source: the same class' NPC wa_/ma_<class>_passenger_r 'enter' (retargeted by bone name to the player rig). The driver clip is the
# MIRROR of that clip (as for sport2): the native wa_/ma_standard2_driver 'enter' starts 0.8 m from the seat turned 49 deg and has to
# be bent 1.1 m / 32 deg onto the vanilla player start (planted-foot slide 3.5 cm/frame, 25 deg/frame steps), the mirrored
# passenger clip needs 0.21 m / 2.7 deg (measured 2026-11-01, Build\work\vehicle-entry-car\t1).
# Builder options: root end matched to the vanilla clip (pos + yaw, smoothstep 25..80 %), head capped 2 cm under the vanilla head envelope
# at the same depth (the door opening / roof line the vanilla clip is collision-safe for), hands kept in world while ducking
# (--hand-ik), planted feet (--plant-hold, IK, min-jerk swings), end pose = LAST frame of the vanilla enter (--end-pose <vanilla>:enter@last)
# reached by a 0.4 s body / 0.6 s leg blend and a quintic end ease, quintic ease-in from the exact start pose, smoothed root
# (speed rule of the bike check), per-bone step limit 11 deg/frame, limb IK helper bones placed on the FINAL limbs (--limb-ik-final, c9c
# lesson). Deliberately NOT used (bike geometry): --root-yaw-total, --land-by, --replant-raised-by, --end-pose to a TPP pose.
# Every round-tripped clip runs check_car_entry.py; mutation self-tests (builder + check only) must fail: no --limb-ik-final (k), a head cap
# 12 cm too high (l), the clip checked against the other seat's vanilla clip (b, c). Any failure aborts before the pack step.
# Door clearance (check_door_clearance.py, timeline from door_timing.json: measured open / close of the vehicle door, the door CLOSE is delayed per
# seat by the redscript UTPPCarDoorCloseDeltaS = generated from the per-clip 'close_after_s' this script solves: delta = max over genders of close_after - vanilla close after the door event, added to evt.autoCloseTime): V's body capsules must not penetrate the
# front door of the seat she enters by while it is closed, opening, open and SWINGING SHUT. Per class and seat the class table holds the start-spot levers
# door = @{ 'g:seat' = @(start-out m, start-back m) }: --start-back (the start spot - where the engine places V before the clip - is that far farther BEHIND the
# door's rear edge, seat-frame +z, outside the swept arc of the opening door) and --start-out (that far farther from the car side). The root delta is longer by
# the same amount, so check (c) 'start in the seat frame' asserts exactly that shift (--expect-start-shift x / -z) instead of the vanilla start. No hold lever
# (V flows into the seat at the natural pace). Every clip runs the door model --solve (earliest close start whose whole closing sweep clears V + margin ->
# close_after_s, fragment door-close-<clip>.json, merge with gen_door_timing.py --merge <run dirs>), the full door verdict at that close_after_s and its
# mutation self-tests (never opens / opens late / closes at once / vanilla close / edge). No waivers: a clip that cannot be made clear fails the run.
# 2026-11-02 (rear gap + approach): door = @{ 'g:seat' = @(start-out m, start-back m [, ramp A s, ramp B s]) }; with the ramp the start-out / start-back part of the root correction has its
# OWN smoothstep window A..B s (--back-ramp, hands follow it fully: --hand-follow-shift 1) so the forward move completes before the source's sideways entry. Every clip then also runs
# the REAR gate (phase 'rear' of check_door_clearance.py: rear door skin / glass / glass slab, relative to the vanilla clip of the set, --rear-ref) inside the door --solve and the
# APPROACH gate (check_door_clearance.py --approach --hold-cap on: the 30 measured mount spots, vanilla slide, native root hold with the planned cap), no waiver.
# -AllowFail: names of check_car_entry checks that are waived for this run (checks under rework by another change, e.g. b_end_pose_ok,m_seat_settle_ok).
# Prerequisites: Extracted\game\vehicle-entry-car (01_extract_car.ps1; for the door check also 01_extract_car.ps1 -Doors -> door-data\; wa_sport2_passenger_r.anims.glb via the subset route, see README.txt),
# the WolvenKit bridge at Build\work\vehicle-entry-car\bridge-bin (dotnet publish of wkit_bridge). Writes only <Out> (default
# Build\work\vehicle-entry-car\run-<stamp>); nothing is installed. Pack with pack_car_workspot.ps1 -AnimsRun <Out> -Out <new dir>.
param(
  [string]$Stamp = (Get-Date -Format 'yyyyMMdd-HHmmss'),
  [string]$Out = '',
  [string[]]$Classes = @('standard2'),
  [string[]]$Genders = @('w', 'm'),
  [string[]]$Seats = @('driver', 'passenger_r'),
  # appended to the builder arguments of every clip of this run (tuning a class before it goes into the table below)
  [string[]]$ExtraArgs = @(),
  [string[]]$DuckExtra = @(),          # appended AFTER the class table head-duck options of this run (later argument wins): screening a duck setting without editing the table. 2026-11-04 r11: tokens are separated by ';' (NOT ',': an option value may hold commas, e.g. '--duck-chain=Spine2:0.2,Spine3:0.25'); an array argument is taken token by token. Needs a row with a duck table entry and no -NoDuck (otherwise refused / warned, never silently ignored)
  [double]$CapMargin = -0.02,
  [double]$RoofMargin = 0.03,      # --roof-margin of the GEOMETRIC head cap (build_entry_clip.py --roof-json): clearance kept under the car ceiling (head sphere = Head + 0.09 up, r 0.10 + 0.05 hair)
  [switch]$NoRoofCap,
  [switch]$NoDuck,                 # ignore the per-row neck / upper-spine head duck layer (classTable duck, head_duck.py): reproduces the round-1 clips
  [switch]$NoLimbPath,             # ignore the per-row hips path / limb escape layer (classTable lpath, limb_path.py): reproduces the round-2 clips
  [double]$RoofPostLeanDeg = 30,   # --roof-post-lean: trunk bend (deg) the post-end-blend roof correction may use
  [int]$RoofPasses = 2,            # closed-loop passes of the roof cap (roof_cap_iter.py); 2 = what the accepted rows were built with (standard3 m:driver differs by 0.21 m with 4)
  [double]$RoofMarginStep = 0.01,  # extra clearance of each roof cap pass after the second
  [double]$RoofSlackCm = 0.2,      # the roof phase must pass the gate with this many cm to spare in the builder GLB
  [switch]$SkipSelftest,
  [string[]]$AllowFail = @(),
  [double]$DoorTolCm = 1.0,
  [double]$DoorOpenTolCm = 2.0,
  [string]$ApproachAlign = 'T',   # approach gate clock alignment: T (clip time 0 = mount + slide, MEASURED) | door | both (see README)
  [switch]$ArrivalRing,            # approach gate (r13): also the ARRIVAL ring (starts 0.06-0.15 m around the clip start spot = where the native approach walk ends; check_door_clearance.py --arrival)
  [switch]$ApproachGrid,           # approach gate with the 0.1 m x 5 deg sector grid (default: the 30 measured mount spots of the class + seat only)
  [switch]$SkipApproach,
  [string[]]$DoorOverride = @(),   # 'class:g:seat=out,back[,rampA,rampB]': replaces that door row of the class table for this run (candidate search; the shipped table is edited by hand afterwards)
  [switch]$NoBodyWaive,            # evaluation run: ignore bodyWaive (raw body / roof numbers of the door check)
  [int]$ApproachJobs = 4,          # worker processes of the approach gate (was a fixed 8)
  [switch]$BuilderOnly,            # iteration mode: builder GLB + check_car_entry + roof / body phases only (no WolvenKit round trip, no door solve, nothing packed)
  [string]$ApproachBaselineSources = '',  # the CURRENTLY shipped set (pk-door7b = pk-door7-turbo-anims2): the approach gate is not an absolute gate (rides 12 / 15 / 16), a clip must not be WORSE than its shipped predecessor, capsule by capsule (door_approach_cli.py)
  [string]$FromGlbDir = '',        # default off: directory with <clip>.anims.glb + <clip>.build-report.json (builder output of a finished run, e.g. with spliced steps): the builder / roof loop is skipped for every clip of this run
  [string]$CheckScript = '',
  [string]$Bridge = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $ApproachBaselineSources) { $ApproachBaselineSources = Join-Path $root 'Build\work\vehicle-entry-car\pk-door7-turbo-anims2\sources.txt' }
if (-not $Bridge) { $Bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }
$ErrorActionPreference = 'Stop'
$ExtraArgs = @($ExtraArgs | ForEach-Object { $_ -split ',' } | Where-Object { $_ -ne '' }); $DuckExtra = @($DuckExtra | ForEach-Object { $_ -split ';' } | Where-Object { $_ -ne '' }); $Classes = @($Classes | ForEach-Object { $_ -split ',' }); $Genders = @($Genders | ForEach-Object { $_ -split ',' }); $Seats = @($Seats | ForEach-Object { $_ -split ',' })  # powershell -File passes a comma list as one string
if ($NoDuck -and $DuckExtra.Count -gt 0) { throw '-DuckExtra has no effect with -NoDuck (the head duck layer is off): drop one of them' }  # 2026-11-04 r11: never silently ignored
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$checkCar = if ($CheckScript) { $CheckScript } else { Join-Path $tools 'check_car_entry.py' }
$ex = Join-Path $root 'Extracted\game\vehicle-entry-car'
$glb = Join-Path $ex 'glb'
$jsonv = Join-Path $ex 'json'
$run = if ($Out) { $Out } else { Join-Path $root "Build\work\vehicle-entry-car\run-$Stamp" }
if (Test-Path $run) { throw "output exists, choose a new -Out: $run" }
$anim = Join-Path $run 'glb'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
if (!(Test-Path $Bridge)) { throw "bridge missing: $Bridge (dotnet publish wkit_bridge)" }
$stage = Join-Path $run 'utpp_vehicle_car'
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot, $anim, "$run\selftest" | Out-Null
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$common = @('--target-duration', '--end-blend', '0.4', '--end-blend-legs', '0.6', '--hand-ik', '--corr-start', '0.25', '--corr-end', '0.8',
  '--smooth-rot-limit', '11', '--warp-interp', 'cubic', '--limit-pad', '3', '--plant-hold', '--ik-max-extend', '0.999', '--ik-bend-sigma', '2',
  '--root-smooth', '--ease-in', '0.2', '--end-ease', '0.2', '--end-ease-helpers', '0.6', '--ref-joint-fixed')
# Per class: dir; all / pr = builder args (all clips / passenger clips); od = tolerance of check (n) over-ducking (the pk-classes6 maximum of the class + 4 cm);
# door = @{ 'g:seat' = @(start-out m, start-back m) } per gender (w / m) and seat (start-back: + behind the vanilla start spot = behind the door's rear edge,
# - toward the front), chosen 2026-11-02 by door_prescreen.py / door_rescreen.py (builder GLBs, no round trip; Build\work\vehicle-entry-car\door3-prescreen) as the
# smallest shift for which the door model passes every phase at the solved close_after_s at V lag 0 AND 0.3 s with >= 3 cm clearance in the opening / open
# phases (the start pop of the engine placement = |shift| exactly, asserted by check (c)); no hold lever.
# doorWaive = @{ 'g:seat' = max cm }: UNRESOLVED opening / open phase penetrations (sport1 lifting door: chest vs the rising door skin / shin vs the sill piece;
# sport2 w:passenger_r: shin vs the open door skin; README 'Model limits'); it never covers the closing sweep / the closed door. A waiver W is a flat
# gate and a capsule can never penetrate deeper than its radius (hand/forearm 4, upper arm 5, shoulder/shin 5.5-6, neck 7, thigh 8.5, head 9.5, hips 11, pelvis 13,
# spine 14, chest 15 cm): 15.0 (sport1 drivers) therefore gates NOTHING in the opening / open phases, 8.5 leaves pelvis / hips / spine / chest / head gated, 6.0 leaves the
# trunk, head, thighs and neck gated (check_door_clearance.py reports the 'ungated' capsules per phase). Values kept as they are.
$classTable = @{
  standard2  = @{ dir = 'standard';  # roofpack 2026-11-03: geometric roof cap ON for all four rows (m:driver 3.0 -> -2.7 cm, m:passenger_r 6.5 -> 4.4 = seated baseline; the pwa rows are unchanged), no waivers needed
    all = @(); pr = @(); clipx = @{ 'w:driver' = @('--cap-margin', '-0.05', '--start-hold', '0.3'); 'm:driver' = @('--start-hold', '0.3') };  # round 4 (startfwd): both drivers start AT the game's start (door row 0, 0) and hold the start pose 0.3 s while the door opens (--start-hold); w:driver cap margin (earlier note): the start shift (0.4 out) put the head 1.9 cm over the vanilla envelope (check l) - cap 3 cm lower
    door = @{ 'w:driver' = @(0.0, 0.0); 'w:passenger_r' = @(0.0, 0.4); 'm:driver' = @(0.0, 0.0); 'm:passenger_r' = @(0.0, 0.4) };  # r13: UNCHANGED (the game's start). Tried 0.10 / 0.15 m forward (start y -0.31 / -0.26): every door / approach / body gate passes, but the ARRIVAL ring (--arrival: the native approach ends 0.06-0.15 m from the spot, logged residuals of r6a) puts the left thigh 0.8-2.8 cm into the opening door (w, directions 45 / 90 deg, r 0.12 / 0.15): the game's start is already within 0.07 m of the arrival-safe spot (entry_feel E1, door disk + 0.35 m). 0.15 out / -0.10 passes the ring but starts 0.15 m farther from the car side. Round 4: the drivers were (0.4, 0.5, 0.3, 0.7) / (0.6, 0.4, 0.5, 0.9)
    doorWaive = @{  } }
  sport2     = @{ dir = 'sport'; roofOff = $true;  # roof cap OFF (2026-11-03 roofpack): drivers - the cap does not beat the shipped clips (pwa 3.96 vs 2.7 cm shipped, m 3.46 vs 4.1: the head must pass under the header 25-40 cm below the seated head, anatomical (n) fails by the pelvis floor); passengers - roof = seated baseline already (4.0 / 4.4 cm)
    all = @('--cap-margin', '-0.035', '--smooth-rot-limit', '8'); pr = @('--hand-follow-min', '0.8'); clipx = @{ 'w:driver' = @('--start-hold', '0.3'); 'm:driver' = @('--start-hold', '0.3') };
    door = @{ 'w:driver' = @(0.45, -0.09); 'w:passenger_r' = @(0.0, 0.6); 'm:driver' = @(0.45, -0.09); 'm:passenger_r' = @(0.0, 0.6) };  # r13c: the drivers 0.45 / -0.09 (was 0.4 / -0.05): start (-2.013, -0.782), 0.04 m farther from the car side and 0.04 m farther forward than r13b; entry_feel E1 gap 0.135 / E2 0.31 / E3-E5 ok, approach gate RELATIVE 0 FAIL for BOTH bodies (r13b: m ride23_parallel thigh_r +2.66 cm absent in R10 / w farm_r +0.21 cm), arrival ring 0 FAIL, door / body / roof / checks unchanged (close 1.878 / 2.053 s). Tried 0.5 / -0.05 (ride23 hand_r +2.53 vs base +2.37: relative FAIL), 0.5 / -0.12 (E1 gap 0.155 > 0.15), 0.6 / -0.05 (START slide 0.602 > 0.57, E1 0.335, E2 0.351). r13: the drivers start 0.19 m FORWARD of r10 (start y -1.01 -> -0.82 for both bodies; arrival-safe spot -0.69, entry_feel E1 gap 0.25 / 0.33 -> 0.14 / 0.14, along-car travel 0.44 / 0.46 -> 0.33 / 0.33 m; 0.35 / -0.08 for w put the spot 0.07 m from the harness preset of fixture ride24 and ApproachTest ('table sims') failed it: the facing carry overshoots 0.32 m). Passes every door / body / roof gate and the ARRIVAL ring (--arrival, 0.06-0.15 m around the spot). Tried w 0.3 / -0.05 (head 1.01 cm vs the body shell gate 1.0: rejected by 0.01 cm), farther forward (w -0.17 / -0.22, m -0.20 / -0.26): the door OPEN phase fails (hand_l / forearm vs the open door, -2.3 .. -3.7 cm) or the fallback ride23 (parallel) hold goes deeper into the opening door (m thigh_r +5.4 cm). The old fixture ride23 (+0.5 .. +2.7 cm vs a baseline that already penetrates 1.9-2.1 cm) is the one open item = fallback after an aborted approach. Round 4 (startfwd): the drivers were (0.0, 0.8) / (0.0, 0.2): the start-back of 0.8 pushed the pelvis / chest 12-14 cm into the quarter panel (bodyWaive 14.1); now 0.14 m behind the game's start (limit 0.15) + 0.3 / 0.4 m out + the 0.3 s start hold (door fully open before V moves) - body shell, joint rule, door and the full approach sweep pass without a waiver (the natural start at the game's start FAILS the full approach sweep: rides 23 / 25 hold thigh / pelvis 5-9 cm). The old comment (entryc, back-ramp 0.2-0.6 / start-back 0.6) is obsolete.  # entryc: UNSOLVED row w:driver (shipped): the body shell phase measures chest / spine / pelvis 12-14 cm in the quarter panel; --back-ramp 0.2-0.6 clears the shell but fails the door OPEN phase (-4.5 cm) without the roof cap and the full approach sweep with it (ride 23 / 25 thigh +4.6 / +6.6 cm vs baseline 3.6 / 3.2); start-back 0.6 fails the approach gate too
    doorWaive = @{ 'w:passenger_r' = 6.0 };
    # minimal waivers (waiver_table.py, measured + 0.3 cm): w:driver body 14.1 gates NOTHING (every body capsule radius <= 14.1: the quarter-panel row is UNSOLVED, see door comment), roof 3.0; m:driver body 4.8 / roof 4.4 (head vs header)
    bodyWaive = @{ 'm:passenger_r' = @('head=3.3') };  # round 4b: per-CAPSULE waiver (capsule=cm), measured + 0.3 cm; the pma passenger clip is unchanged and fails the body phase for the first time (head vs the shell). roofpack2: the roof element of the waivers is gone - the head duck layer (head_duck.py) brings the roof phase under its ABSOLUTE gate (pwa 2.7 -> 0.5 cm, pma 4.1 -> 0.7 cm)
    duck = @{ 'w:driver' = @('--duck-chain', 'Spine2:0.2,Spine3:0.25,Neck:0.35,Neck1:0.2', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '40', '--duck-fade', '0.034', '--duck-vmax-deg', '4', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8'); 'm:driver' = @('--duck-chain', 'Spine2:0.2,Spine3:0.25,Neck:0.35,Neck1:0.2', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '40', '--duck-fade', '0.034', '--duck-vmax-deg', '4', '--duck-smooth', '1.0', '--duck-slack-cm', '1.2') } }
  standard25 = @{ dir = 'standard'; roofOff = $true;  # roof phase already within the gate (-1.5 .. -2.3 cm) without a cap; the shipped clips stay
    all = @(); pr = @(); clipx = @{ 'm:passenger_r' = @('--smooth-rot-limit', '9'); 'w:passenger_r' = @('--smooth-rot-limit-global', '11', '--smooth-rot-limit2', '4', '--smooth-keep-first', '0.35');
      'w:driver' = @('--hips-path', '0.02', '0.01', '2') };  # r13d: hips path held inside [c - 1 cm, c] along the car-forward axis (c = min(running max, seat + 2 cm)): entry_feel E3 sink slide of the Hips 0.051 -> 0.047 (gate 0.05), the door verdict is identical and the approach gate has 0 RELATIVE FAIL
    door = @{ 'w:driver' = @(0.0, 0.0); 'w:passenger_r' = @(0.0, 0.3, 0.4, 0.8); 'm:driver' = @(0.0, 0.0); 'm:passenger_r' = @(0.0, 0.3) };
    doorWaive = @{  } }
  standard3  = @{ dir = 'standard'; roofOffRows = @('w:driver', 'w:passenger_r'); all = @(); pr = @(); clipx = @{ 'w:driver' = @('--smooth-rot-limit-global', '11', '--smooth-rot-limit2', '4', '--smooth-keep-first', '0.35', '--hand-out', '0.14', '--hand-out-window', '0.6', '1.0', '--out-axis', '-1', '--hips-path', '0.01', '0.01', '2');
      'm:driver' = @('--smooth-rot-limit-global', '11', '--smooth-rot-limit2', '4', '--smooth-keep-first', '0.1', '--out-ramp', '0.5', '0.9', '--hips-path', '0.02', '0.01', '2');
      'w:passenger_r' = @('--smooth-rot-limit-global', '11', '--smooth-rot-limit2', '4', '--smooth-keep-first', '0.35', '--out-ramp', '0.4', '0.8'); 'm:passenger_r' = @('--smooth-rot-limit-global', '11', '--smooth-rot-limit2', '4', '--smooth-keep-first', '0.35', '--out-ramp', '0.5', '0.9') };
    door = @{ 'w:driver' = @(0.50, 0.066, 0.15, 0.45); 'w:passenger_r' = @(0.25, 0.7, 0.3, 0.7); 'm:driver' = @(0.7, 0.9, 0.3, 0.7); 'm:passenger_r' = @(0.5, 0.6, 0.3, 0.7) };  # r13h: w:driver 0.51 / 0.072 -> 0.50 / 0.066 (start 1.0 cm inward, 0.6 cm forward): entry_feel E1 gap 0.159 -> 0.143 (continuous door-wedge boundary 0.160 -> 0.146), the inward / forward mix with the smallest fixture-ride growth of the rigid placement map (reduced screen sweep, r13h/scr); official sweep vs R13g: ride14 / ride15 slide-phase contacts +0.1-0.6 cm, door opening phase +0.7 -> -0.3 cm (gate -1.0)  # round 10 (r10): w:driver = door 0.51 / 0.072 / 0.15 / 0.45 + clipx --hips-path 0.01 0.01 2 (hips path held inside [c - 1 cm, c] along the car-forward axis: hips excursion 0.22 -> 0.04 m, sink slide 0.21 -> 0.05 m; start dy -0.072 m, slide 0.515 m, no row_limits entry; approach RELATIVE conditional 0.07-0.61 cm); m:driver = the shipped door row + clipx --hips-path 0.02 0.01 2 (rearward gate passes, start position stays documented in row_limits.json, approach RELATIVE conditional 0.56 cm); the lever is OFF (default) everywhere else.  # round 4b: the standard3 drivers are the PREVIOUS shipped rows again. The round-4 candidates (w 0.35 / 0.14 / ramp 0.3-0.6, m 0.4 / 0.35 / ramp 0.3-0.7 + hold 0.15) fail the approach gate (rides 12 / 14 / 15) and slide the hips rearward as much; rebuilding with them does not give shipped clips
    doorWaive = @{  };
    # roofpack 2026-11-03: roof cap ON for m:passenger_r only (roof 14.5 -> 4.4 cm, body head 9.3 -> 5.6, approach / door pass). OFF for w:passenger_r (cap: roof 13.4 -> 4.8 but approach RELATIVE fail: chest +1.95 cm in ride16_passenger),
    # w:driver (roof 7.9 still over the 1.2 gate, d/f checks fail, approach rear_entity farm_r +0.30 cm = the Emperor 0.25 cm miss) and m:driver (roof 14.0 -> 0.6 cm, body head 9.0 -> 2.6, but approach RELATIVE +0.57 cm pelvis / +0.33 hips_l in ride12_parallel (hold) -
    # CONDITIONAL: remove 'm:driver' from roofOffRows once a relative approach tolerance of 0.6 cm is accepted; the built clip is in roof-pack\full-standard3 and in packed-plus).
    # minimal waivers (waiver_table.py, measured + 0.3 cm); the roof waiver of w:passenger_r (13.7) and m:driver (14.3) stays close to the 15 cm sphere radius but still gates
    bodyWaive = @{ 'm:driver' = @('head=2.9'); 'm:passenger_r' = @('head=6.0') };  # round 4b: per-CAPSULE waivers (capsule=cm), measured + 0.3 cm (m:driver = the PREVIOUS shipped clip, head vs the shell)  # round 4 (startfwd): the roof cap is ON for m:driver again (the conditional roof-pack row) and its 9.3 / 14.3 waivers are gone;  # roofpack2: the female rows need NO waiver any more - the head duck layer (neck + upper spine, head_duck.py) clears the roof (w:driver 9.8 -> 0.7 cm, gate 1.22; w:passenger_r 13.4 -> 4.8, gate 5.26) and the body shell (head 8.6 -> -1.0 / 9.1 -> 5.4 cm)
    duck = @{ 'w:driver' = @('--duck-chain', 'Spine3:0.4,Neck:0.35,Neck1:0.25', '--duck-az=0,30,60', '--duck-max-deg', '70', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '6', '--duck-smooth', '0.8', '--duck-slack-cm', '4.5', '--duck-sink-profile', '20:0,23:5,30:5,34:0'); 'm:driver' = @('--duck-chain', 'Neck:0.55,Neck1:0.45', '--duck-az=0,30,60', '--duck-max-deg', '40', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '3', '--duck-smooth', '1.0', '--duck-slack-cm', '4.0', '--duck-slack-until', '43', '@last-pass');  'w:passenger_r' = @('--duck-chain', 'Spine3:0.4,Neck:0.35,Neck1:0.25', '--duck-az=0,30,60', '--duck-max-deg', '70', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '6', '--duck-smooth', '0.8', '--duck-slack-cm', '1.5') } }  # r13: hair margin at the door header (entry_feel E5: head sphere 0.10 + 0.05 hair keeps >= 2 cm under the roof before the seated tail): Hips sink 5 cm over frames 23-30 (feet held) + the duck aims 4.5 cm under the roof gate: transition max penetration +0.2 -> -2.9 cm, every other phase equal to r10 (builder screen r13/logs6); slack 1.5 / no sink was r10
  sport1     = @{ dir = 'sport'; roofOff = $true; all = @(); pr = @('--smooth-rot-limit', '4');
    clipx = @{ 'w:driver' = @('--hips-path', '0.04', '1.0', '2', '--hips-path-hands', 'LeftHand:1+RightHand:0.4', '--hips-path-keep-start'); 'm:driver' = @('--hips-path', '-0.02', '0.04', '2', '--hips-path-hands', 'LeftHand:1+RightHand:0') };
    # r13j: w:driver = the plain cap (--hips-path 0.04 1.0 2 + hands) + --hips-path-keep-start: the r13i arrival ring failure of the plain cap (r0.15_d000 thigh_r +1.81 vs +1.48 cm) was NOT the frame-4 Hips:
    # the engine holds clip frame 0 during the 0.8 s Turbo-R slide while the door opens, and the IK knee bend smoothing (--ik-bend-sigma) had carried the layer into frame 0 (right knee 0.43 cm).
    # With frame 0 = the no-layer start pose: Hips back excursion 0.186 -> 0.081, sink slide 0.093 -> 0.091 m, ring thigh_r +1.48 (= R13h), door close 2.219 s and every phase unchanged; the r13i hold / fade below is replaced.
    # r13i: drivers - the forward Hips lunge of frames 0-9 is capped at seat + EPS (w 4 cm, m -2 cm; m also holds the Hips at most 4 cm behind that cap = 6 cm behind the seat, DELTA), the hands take
    # the hips path translation (left 1 = no over-reach of the door-side arm, right w 0.4 / m 0: the right forearm stays off the open door at frame 5): entry feel sink slide w 0.221 -> 0.093, m 0.219 -> 0.093 m;
    # m Hips back excursion 0.254 -> 0.098. w keeps the source path to frame 4 and holds that level to frame 6 (--hips-path-hold 5 7 0.02, fade 4-5): the arrival ring start r0.15_d000 puts the right thigh
    # into the opening door when the Hips are more than ~1.5 cm behind the source at frame 4 (cap from frame 0: thigh_r +1.81 vs +1.48 cm), so 0.10 m of the remaining back travel is outside the car side line,
    # at most 2.0 cm/frame (+ smoothing: 2.46 cm/frame peak, the shipped sit-down slide peaks at 2.40) while the Hips rise (Hips back excursion 0.237 -> 0.186, inside the side line 0.276 -> 0.143 m);
    # m:driver duck slack 6.5 -> 8.0 cm and the Hips sink profile starts at frame 24 (was 27): the raised Hips dip of frames 22-30 keeps E5 at -3.1 cm (was -2.4)
    # r13f: drivers duck slack 1.2 -> 6.5 cm + --duck-slack-until 43 (the transition frames before the 18-frame seated tail keep the slack): entry feel E5 +2.3 / +2.7 -> -2.5 / -2.4 cm (head + hair under the roof); body within the waivers (w uarm_r 6.57 -> 6.9 = the waiver limit, m -3.7 -> -0.2 cm), approach 0 FAIL relative, d passes
    door = @{ 'w:driver' = @(0.0, -0.8, 0.4, 1.7); 'w:passenger_r' = @(0.0, -0.8); 'm:driver' = @(0.0, -0.8, 0.4, 1.7); 'm:passenger_r' = @(0.0, -0.8) };  # entryc: shipped. TRIED passengers -0.6 + ramp 0.4-1.7 (seated pelvis slide 21 -> 5-7 cm): the door check of both passenger clips FAILS (door=1)
    doorWaive = @{  };
    # roofpack 2026-11-03: roof cap OFF. With it the drivers clear the roof (14.1 -> 2.6 / 14.0 -> 2.8 cm) but the lifting door OPEN phase fails (-5.9 / -5.8 vs -1.2 shipped) and w:driver fails d (arm flip) and loses the hand_l gate;
    # the passengers stay at the seated baseline + 0.5 (13.9 vs gate 13.44: the seated head itself is 12.9 cm inside the lining model) with shin_l +0.6 cm worse.
    # minimal waivers (waiver_table.py, measured + 0.3 cm): the roof waivers 14.4-14.7 are below the 15 cm sphere radius and gate; body 11.4-11.5 = head vs the lining model
    bodyWaive = @{ 'w:driver' = @('thigh_r=4.8,shoulder_r=5.8,uarm_r=6.9,farm_r=5.7,hand_r=5.3,head=2.8'); 'm:driver' = @('shoulder_r=4.0,uarm_r=3.5,head=3.1'); 'w:passenger_r' = @('hips_l=4.6,spine=2.2,chest=3.3,neck=8.4,thigh_l=8.4,shin_l=3.8,shoulder_l=7.3,uarm_l=6.7,farm_l=5.9,hand_l=6.0,head=11.7'); 'm:passenger_r' = @('hips_l=3.0,chest=3.9,neck=8.6,thigh_l=6.3,shoulder_l=7.1,uarm_l=6.9,farm_l=6.0,hand_l=5.9,head=11.4') };  # round 4b: the flat waivers (6.9 / 4.0 / 11.5 / 11.4: every capsule) are replaced by per-CAPSULE waivers (capsule=cm), measured + 0.3 cm - only the capsules that need one are waived  # roofpack2: w:passenger_r - the head duck layer brings the roof phase to 13.39 cm (absolute gate 13.44 = seated 12.94 + 0.5), the roof waiver is gone; roofpack3: the driver roof waivers (14.4 / 14.5) are gone too (head duck + Hips sink profile + elbow lock, below) and the driver body waivers drop 11.4 / 11.5 -> 6.9 / 4.0 cm (measured 6.57 uarm_r / 3.7 shoulder_r + 0.3 after the monotone sink plan, sink_plan.py: the head left the lining model, 11.0 -> 5.0-5.5 cm)
    duck = @{ 'w:driver' = @('--duck-chain', 'Spine2:0.2,Spine3:0.25,Neck:0.35,Neck1:0.2', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '50', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '6', '--duck-smooth', '1.0', '--duck-slack-cm', '6.5', '--duck-slack-until', '43', '--duck-verify-cm', '0.3', '--duck-relock-elbow', '--duck-relock-limbs', 'LeftHand', '--duck-sink-profile', '27:0.0,31:9.0,32:9.0,33:9.0,34:9.0,35:8.2,36:5.9,37:6.9,38:7.3,39:7.6,40:7.4,41:7.1,42:6.7,43:6.9,44:7.0,45:7.1,46:7.4,47:7.6,48:7.9,49:7.7,50:7.1,51:6.1,52:5.0,53:3.8,54:2.4,55:0.9,56:0.0,57:0.0,58:0.0');
      'm:driver' = @('--duck-chain', 'Spine2:0.2,Spine3:0.25,Neck:0.35,Neck1:0.2', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '50', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '6', '--duck-smooth', '1.0', '--duck-slack-cm', '8.0', '--duck-slack-until', '43', '--duck-verify-cm', '0.3', '--duck-relock-elbow', '--duck-relock-limbs', 'LeftHand', '--duck-sink-profile', '24:0.0,27:2.5,29:4.5,31:6.0,32:6.0,33:6.0,34:6.0,35:6.0,36:5.7,37:6.3,38:6.5,39:6.5,40:6.2,41:5.8,42:5.5,43:5.4,44:5.3,45:5.2,46:5.3,47:5.4,48:5.5,49:5.4,50:5.0,51:4.4,52:3.7,53:2.9,54:2.0,55:1.0,56:0.0,57:0.0,58:0.0');
      'w:passenger_r' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=-30,0,30', '--duck-max-deg', '60', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '5', '--duck-smooth', '0.8', '--duck-slack-cm', '1.0', '--duck-repair', '400'); 'm:passenger_r' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=0,30,60,90', '--duck-max-deg', '60', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '5', '--duck-smooth', '0.8', '--duck-slack-cm', '1.0', '--duck-repair', '400') } }
  utility    = @{ dir = 'utility'; roofOff = $true; all = @('--smooth-rot-limit', '6'); pr = @();
    door = @{ 'w:driver' = @(0.4, 0.22); 'w:passenger_r' = @(0.0, 0.14); 'm:driver' = @(0.4, 0.22); 'm:passenger_r' = @(0.0, 0.8) };  # r13e: drivers 0.4 / 0.12 -> 0.4 / 0.22 (start y 3.756 -> 3.656, 0.031 m in front of the seat; with --root-monotone the entity no longer travels 0.136 m to the rear; START reference = the aperture rear edge, start_gate.py r13e)  # round 7 (r7util): w:driver 0.4 / 0.12 (was 0.4 / 0.35: the start is 0.12 m behind the game's start, inside the 0.15 limit, and the full approach sweep passes); the hips path layer (lpath) removes the sink slide; w:passenger_r 0.0 / 0.14 (was 0.0 / 0.8); m:passenger_r stays at the SHIPPED 0.0 / 0.8 (previous clip, conditional door in the pack). r7util candidates for it, NOT shipped: full-ut-mp3 = out 0.15 / back 0.14 + hold 0.3 + lpath 'LeftFoot' (--lp-sink-profile 0:0,1:0,5:5,26:5,38:0 --lp-hips-cap 1.0 --lp-hips-mode iso --lp-hips-start 12 --lp-hips-sigma 1.5 --lp-leg-step-limit 13 --lp-limbs LeftFoot --lp-dir 0,1,0 --lp-iters 20 --lp-slack-cm 2.0 --lp-damp 1.0 --lp-fwd-bump 16:0,24:2,34:0): approach 0 FAIL, jamb hand_l 2.59 cm against the gate 1.0 (hand nudges of up to 14 cm do not clear it and break check (d)); full-ut-mp2 = out 0.0: jamb passes, approach ride32 hold head +0.96 cm (out 0.0 / 0.05 / 0.08 / 0.12 / 0.15: +0.96 / 0.64 / 0.42 / 0.12 / pass)
    lpath = @{ 'w:driver' = @('--lp-hips-cap', '1.0', '--lp-hips-mode', 'iso', '--lp-hips-start', '20', '--lp-hips-sigma', '1.5', '--lp-limbs', 'RightHand', '--lp-dir', '0,1,0', '--lp-iters', '40', '--lp-slack-cm', '3.0', '--lp-damp', '1.0', '--lp-max-cm', '20');
      # m:driver: Hips drop of 9 cm over frames 1-5 (feet held; 0 at frames 0-1: check (f) first step) keeps the head of the tall body out of the opening door in the approach gate ('hold' class, ride30_door, clip frame ~4: 2.2 cm over the baseline without it, 0.18 cm with 7 cm at 40 fps)
      'm:driver' = @('--lp-sink-profile', '0:0,1:0,5:9,26:9,38:0', '--lp-hips-cap', '1.0', '--lp-hips-mode', 'iso', '--lp-hips-start', '20', '--lp-hips-sigma', '1.5', '--lp-leg-step-limit', '13', '--lp-limbs', 'RightHand,RightFoot', '--lp-dir', '0,1,0', '--lp-iters', '20', '--lp-slack-cm', '2.0', '--lp-damp', '1.0');
      'w:passenger_r' = @('--lp-hips-cap', '1.0', '--lp-hips-mode', 'iso', '--lp-hips-start', '20', '--lp-hips-sigma', '1.5', '--lp-leg-step-limit', '13', '--lp-limbs', 'LeftFoot', '--lp-dir', '0,1,0', '--lp-iters', '12') };
    clipx = @{ 'w:driver' = @('--start-hold', '0.3', '--root-monotone', '2', '--root-monotone-door-json', "$ex\door-data\utility.json"); 'm:driver' = @('--start-hold', '0.3', '--root-monotone', '2', '--root-monotone-door-json', "$ex\door-data\utility.json"); 'w:passenger_r' = @('--start-hold', '0.3') };  # r13e: drivers --root-monotone (entity path monotone along the car, world body unchanged)  # round 4 (startfwd): drivers were (0.0, 0.6) / (0.0, 0.8); start hold 0.3 s  # entryc: shipped. TRIED: ramp 0.2-0.6 clears the body shell but the door OPENING phase fails (-5.0 cm); start-back 0.4 fails the approach gate (ride 30 head 7.2 cm)
    doorWaive = @{  };
    # roofpack 2026-11-03: roof cap OFF. With it m:driver roof 8.5 -> 0.5 cm but spine +4.2 / chest +3.0 cm into the cab (body regress), m:passenger_r roof 6.95 -> 6.7 (no gain, chest +4.5), w:driver roof 1.9 -> 1.2 (still over, closing phase 7.9 -> 1.1 cm, (n) fails),
    # w:passenger_r roof 7.1 -> 4.7 (still over 1.0; pelvis +1.8, (n) fails)
    # minimal waivers (waiver_table.py, measured + 0.3 cm)
    bodyWaive = @{ 'm:passenger_r' = @('pelvis=2.8,hips_l=7.3,spine=3.3,chest=4.6,neck=4.2,thigh_l=5.0,shoulder_l=5.7,uarm_l=5.1,farm_l=3.7,hand_l=2.8,head=8.2') };  # round 7 (r7util): the w:passenger_r waiver (hips_l 5.5 ... head 7.4 cm) is gone - the new clip starts 0.14 m behind the game's start (was 0.8) and passes the absolute body gate; m:passenger_r keeps the waiver of the previous clip (round 4b)  # round 4b: per-CAPSULE waivers (capsule=cm), measured + 0.3 cm (were flat 9.4 / 8.3)  # round 4: the driver waivers (2.7 / 9.5) are gone  # roofpack2: roof elements removed for the rows the head duck layer fixes (w:driver 1.9 -> 0.3, w:passenger_r 7.1 -> 0.1, m:passenger_r 6.95 -> 0.55 cm)
    duck = @{ 'w:driver' = @('--duck-chain', 'Spine2:0.2,Spine3:0.25,Neck:0.35,Neck1:0.2', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '40', '--duck-fade', '0.034', '--duck-vmax-deg', '4', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8'); 'w:passenger_r' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '60', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '5', '--duck-smooth', '1.0', '--duck-slack-cm', '1.0'); 'm:passenger_r' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=-90,-60,-30,0', '--duck-max-deg', '60', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '5', '--duck-smooth', '1.0', '--duck-slack-cm', '1.0'); 'm:driver' = @('--duck-chain', 'Spine:0.1,Spine1:0.15,Spine2:0.2,Spine3:0.2,Neck:0.2,Neck1:0.15', '--duck-az=-90,-60,-30', '--duck-max-deg', '70', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '6', '--duck-smooth', '0.7', '--duck-slack-cm', '1.5') } }
}
foreach ($o in $DoorOverride) {  # search / evaluation runs: 'class:g:seat=out,back[,rampA,rampB]' replaces the class table door row of this run only
  $kv = $o -split '=', 2; $kp = $kv[0] -split ':'
  if ($kp.Count -ne 3 -or !$classTable.ContainsKey($kp[0])) { throw "bad -DoorOverride $o" }
  $classTable[$kp[0]].door["$($kp[1]):$($kp[2])"] = @($kv[1] -split ',' | ForEach-Object { [double]$_ })
}
$checks = @()
foreach ($cls in $Classes) {
  if (!$classTable.ContainsKey($cls)) { throw "unknown class $cls" }
  foreach ($g in $Genders) {
    foreach ($seat in $Seats) {
      $tgtName = "p${g}a_${cls}_$seat"
      $npcName = "${g}a_${cls}_passenger_r"
      $clip = "utpp_car_$tgtName"
      # end pose (seat-settle step): driver ends on frame 0 of the class tpp_idle, the front passenger on frame 0 of 'idle' (make_passenger_idle.py)
      $endClip = if ($seat -eq 'driver') { 'tpp_idle' } else { 'idle' }
      $tgtGlb = "$glb\$tgtName.anims.glb"; $npcGlb = "$glb\$npcName.anims.glb"
      $out = Join-Path $anim "$clip.anims.glb"
      $extra = @() + $classTable[$cls].all
      if ($seat -eq 'driver') { $extra += '--mirror'; if ($classTable[$cls].dr) { $extra += $classTable[$cls].dr } }
      else {
        $extra += $classTable[$cls].pr
        if ($cls -eq 'sport2') { $extra += @('--cap-clip', "$glb\p${g}a_sport2_driver.anims.glb:enter") }
      }
      $dv = $classTable[$cls].door["${g}:$seat"]  # per gender + seat: @(start-out m, start-back m)
      $doorOut = 0.0; $doorBack = 0.0
      $doorRamp = @()
      if ($dv -and $dv.Count -ge 4) { $doorRamp = @('--back-ramp', "$($dv[2])", "$($dv[3])", '--hand-follow-shift', '1', '--smooth-rot-limit', '4') }  # smooth-rot-limit 4: check (d) - the forward move steps a foot (sport1 passenger uses the same)
      if ($dv) {
        if ($dv[0] -gt 0) { $doorOut = [double]$dv[0]; $extra += @('--start-out', "$($dv[0])", '--out-axis', $(if ($seat -eq 'driver') { '-1' } else { '1' })) }
        if ([Math]::Abs([double]$dv[1]) -gt 0) { $doorBack = [double]$dv[1]; $extra += @('--start-back', "$($dv[1])") }
      }
      $shifted = ($doorOut -gt 0) -or ($doorBack -ne 0)
      $extra += $doorRamp
      $cx = $null; if ($classTable[$cls].clipx) { $cx = $classTable[$cls].clipx["${g}:$seat"] }  # per-clip builder args (here: a lower per-bone step limit where the start shift pushed a foot step over check d)
      if ($cx) { $extra += $cx }
      $noRoof = $NoRoofCap -or $classTable[$cls].roofOff -or ($classTable[$cls].roofOffRows -contains "${g}:$seat")
      if (-not $noRoof) { $extra += @('--roof-json', "$ex\body-data\$cls.json", '--roof-seat', $(if ($seat -eq 'driver') { 'left' } else { 'right' }), '--roof-margin', "$RoofMargin", '--roof-relax-seated', '--roof-final') }
      if ($classTable[$cls].roofm -and $classTable[$cls].roofm["${g}:$seat"]) { $extra += @('--roof-margin', "$($classTable[$cls].roofm["${g}:$seat"])") }  # per-clip margin (later argument wins)
      $extra += $ExtraArgs
      # roofpack round 2: neck / upper-spine head duck layer on the finished pose (head_duck.py), classTable duck = @{ 'g:seat' = @(head_duck options) }; kept out of $extra (the selftest mutation builds do not carry it)
      $duckArgs = @()
      if (-not $NoDuck -and $classTable[$cls].duck -and $classTable[$cls].duck["${g}:$seat"]) {
        $duckArgs = @('--head-duck', '--duck-roof-json', "$ex\body-data\$cls.json", '--duck-roof-seat', $(if ($seat -eq 'driver') { 'left' } else { 'right' })) + $classTable[$cls].duck["${g}:$seat"]
      }
      # round 7: hips path / limb escape layer on the finished pose (limb_path.py), classTable lpath = @{ 'g:seat' = @(limb_path options) }; like the duck layer it is not part of the selftest mutation builds
      $lpArgs = @()
      if (-not $NoLimbPath -and $classTable[$cls].lpath -and $classTable[$cls].lpath["${g}:$seat"]) {
        $lpArgs = @('--limb-path', '--lp-json', "$ex\body-data\$cls.json", '--lp-seat', $(if ($seat -eq 'driver') { 'left' } else { 'right' })) + $classTable[$cls].lpath["${g}:$seat"]
      }
      if ($DuckExtra.Count -gt 0) {
        if ($duckArgs.Count -gt 0) { $duckArgs += $DuckExtra }
        else { Write-Warning "-DuckExtra NOT applied to ${clip}: $(if ($NoDuck) { '-NoDuck is set' } else { "the class table has no duck entry for ${cls} ${g}:$seat" })" }
      }
      # r13f: the token '@last-pass' in the duck options (table or -DuckExtra) applies the duck layer only in the LAST roof cap pass: the closed-loop cap is then measured on the
      # un-ducked clip (the same hips / legs as the build without the duck) and the duck only adds the neck / upper-spine bend; without the token nothing changes
      $duckLast = $duckArgs -contains '@last-pass'; $duckArgs = @($duckArgs | Where-Object { $_ -ne '@last-pass' })
      $bargs = @("$tools\build_entry_clip.py", '--src', $npcGlb, '--src-clip', 'enter', '--target', $tgtGlb, '--target-clip', 'enter',
        '--name', 'enter', '--end-pose', "${tgtGlb}:$endClip", '--cap-margin', "$CapMargin") + $common + $extra
      # closed loop of the geometric roof cap (roof_cap_iter.py): the cap is measured at the head of the FINISHED builder clip and fed to the next pass until the roof phase of the door
      # model (roof_report.py, absolute gate = seated head penetration + 0.5 cm) passes with RoofSlackCm to spare (the WolvenKit round trip moves the head by < 1 mm)
      if ($FromGlbDir) {   # 2026-11-06 (entry stepping): the builder GLB + its build report come from a finished run (splice of the MotionBricks steps, build_entry_steps.py); builder / roof loop are NOT run, everything after (round trip, gates) is unchanged
        $brep = "$anim\$clip.build-report.json"
        Copy-Item "$FromGlbDir\$clip.anims.glb" $out -Force; Copy-Item "$FromGlbDir\$clip.build-report.json" $brep -Force
        "from-glb ${clip}: builder GLB taken from $FromGlbDir (builder / roof loop not run)"
      } else {
      $capFile = $null
      $roofTrace = @()
      $bestPen = [double]::PositiveInfinity; $bestPass = -1
      $brep = "$anim\$clip.build-report.json"
      for ($pass = 0; $pass -lt $(if ($noRoof) { 1 } else { $RoofPasses }); $pass++) {
        $bx = @()
        if (-not $noRoof) {
          # passes >= 2 ask for RoofMarginStep more clearance each (the end blend lifts the head toward the seated pose after the cap acted)
          $mgn = $RoofMargin + $RoofMarginStep * [Math]::Max(0, $pass - 1)
          $bx = @('--roof-margin', "$mgn")
          if ($capFile) { $bx += @('--roof-cap-file', $capFile) }
        }
        $dA = $(if ($duckLast -and -not $noRoof -and $pass -lt $RoofPasses - 1) { @() } else { $duckArgs })
        python -I @bargs @bx @dA @lpArgs --limb-ik-final --out $out --report $brep | Out-Null; Check "build $clip"
        if ($noRoof) { break }
        $rj = "$anim\$clip.roof-pass$pass.json"
        python -I "$tools\roof_report.py" --class $cls --seat $seat --json $rj $out | Out-Null
        $rr = (Get-Content -Raw $rj | ConvertFrom-Json)[0]
        $over = [double]$rr.roof.pen_cm - [double]$rr.roof.gate_cm
        $roofTrace += ('pass {0} (margin {1}): roof {2} cm / gate {3} cm' -f $pass, $mgn, $rr.roof.pen_cm, $rr.roof.gate_cm)
        Copy-Item $out "$anim\$clip.pass$pass.glb" -Force; Copy-Item $brep "$anim\$clip.pass$pass.build-report.json" -Force
        if ($over -lt $bestPen -and $dA.Count -eq $duckArgs.Count) { $bestPen = $over; $bestPass = $pass }   # r13f: with '@last-pass' only the ducked pass is a candidate
        if ($over -le -$RoofSlackCm -and $dA.Count -eq $duckArgs.Count) { break }
        $nextCap = "$anim\$clip.roofcap$pass.json"
        $ia = @('--clip', $out, '--roof-json', "$ex\body-data\$cls.json", '--roof-seat', $(if ($seat -eq 'driver') { 'left' } else { 'right' }), '--end-pose', "${tgtGlb}:$endClip", '--roof-margin', "$mgn", '--roof-relax-seated', '--out', $nextCap)
        if ($capFile) { $ia += @('--prev', $capFile) }
        python -I "$tools\roof_cap_iter.py" @ia | Out-Null; Check "roof cap pass $clip"
        $capFile = $nextCap
      }
      if (-not $noRoof -and $bestPass -ge 0 -and ($over -gt $bestPen)) { Copy-Item "$anim\$clip.pass$bestPass.glb" $out -Force; Copy-Item "$anim\$clip.pass$bestPass.build-report.json" $brep -Force; $roofTrace += "kept pass $bestPass" }
      "roof loop ${clip}: " + $(if ($noRoof) { 'roof cap OFF for this row (see classTable)' } else { $roofTrace -join '; ' })
      }
      if ($BuilderOnly) {
        # iteration mode: the builder GLB against check_car_entry (all checks) and the roof / body phases of the door model (roof_report.py); no round trip
        $shiftXb = $doorOut * $(if ($seat -eq 'driver') { -1.0 } else { 1.0 })
        $ckb = @(); if ($shifted) { $ckb += @('--expect-start-shift', "$shiftXb", '--expect-start-shift-z', "$doorBack") }
        python -I $checkCar --clip $out --vanilla $tgtGlb --src-rig $npcGlb --clip-name enter --build-report "$anim\$clip.build-report.json" --tpp-idle $tgtGlb --tpp-idle-clip $endClip @ckb --report "$run\check-$clip.json" | Tee-Object -FilePath "$run\check-$clip.txt" | Select-Object -Last 1
        python -I "$tools\roof_report.py" --class $cls --seat $seat $out
        continue
      }
      $vj = Join-Path $run 'vanilla-json'
      New-Item -ItemType Directory -Force $vj | Out-Null
      Copy-Item "$jsonv\$tgtName.anims.json" $vj -Force
      python -I "$tools\animset_subset.py" "$vj\$tgtName.anims.json" "$run\base-json\$clip.anims.json" --keep enter | Out-Null; Check "subset $clip"
      New-Item -ItemType Directory -Force "$run\base" | Out-Null
      & $wkit convert deserialize "$run\base-json\$clip.anims.json" -o "$run\base" | Out-Null; Check "deserialize base $clip"
      & $Bridge import $out "$run\base\$clip.anims" "$run\import-$clip" | Select-String 'ANIMS_IMPORT_EXIT'; Check "import $clip"
      New-Item -ItemType Directory -Force "$run\import-json" | Out-Null
      & $wkit convert serialize "$run\import-$clip\$clip.anims" -o "$run\import-json" | Out-Null; Check "serialize $clip"
      python -I "$tools\patch_motion_extraction.py" "$run\import-json\$clip.anims.json" $out "$run\patched-json\$clip.anims.json" --clip enter | Out-Null; Check "patch $clip"
      & $wkit convert deserialize "$run\patched-json\$clip.anims.json" -o $depot | Out-Null; Check "deserialize final $clip"
      & "$tools\export_glb.ps1" -Files "$depot\$clip.anims" -OutDir "$run\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $clip"
      python -I "$tools\verify_roundtrip.py" $out "$run\roundtrip\$clip.anims.glb" "$run\patched-json\$clip.anims.json" --clip enter --report "$run\$clip.roundtrip-report.json" | Out-Null
      Check "verify $clip"
      # (c) for clips that start farther out on purpose: asserts exactly that shift (seat-frame x, +-1 cm), not a waiver; (n) per-class tolerance
      $shiftX = $doorOut * $(if ($seat -eq 'driver') { -1.0 } else { 1.0 })
      $ckx = @(); if ($classTable[$cls].od) { $ckx += @('--overduck-tol', "$($classTable[$cls].od)") }; if ($shifted) { $ckx += @('--expect-start-shift', "$shiftX", '--expect-start-shift-z', "$doorBack") }
      python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --clip-name enter `
        --build-report "$anim\$clip.build-report.json" --tpp-idle $tgtGlb --tpp-idle-clip $endClip @ckx --report "$run\check-$clip.json" | Tee-Object -FilePath "$run\check-$clip.txt" | Select-Object -Last 1
      $ce = $LASTEXITCODE
      # waived checks: -AllowFail only
      $allow = @($AllowFail)
      $resLine = (Get-Content "$run\check-$clip.txt" | Where-Object { $_ -match '^RESULT' } | Select-Object -Last 1)
      if ($ce -ne 0 -and $resLine -match '^RESULT FAIL: (.*)$') {
        $bad = @($Matches[1] -split ',' | Where-Object { $allow -notcontains $_ })
        if ($bad.Count -eq 0) { $ce = 0; "check waived for ${clip}: $($Matches[1])" }
      }
      $checks += "$clip exit=$ce"
      # door clearance (check_door_clearance.py, door_timing.json): --solve = earliest close start whose whole closing sweep clears V (+ margin) ->
      # close_after_s (fragment door-close-<clip>.json, merged by gen_door_timing.py --merge), then the full verdict at that close_after_s. No waivers,
      # a crash (no fresh fragment) is a failure; a close later than the clip end + 0.5 s is refused (a late close must not hide a bad clip).
      # doorWaive = @{ 'g:seat' = max cm }: ONLY the opening / open phases of the door (Turbo-R lifting door model limits, README); never the closing sweep
      $doorTol = @('--open-tol-cm', "$DoorOpenTolCm")
      # bodyWaive = @{ 'g:seat' = @(body cm, roof cm) }: DOCUMENTED EXCEPTIONS of the body shell / roof phases (flat gate, the absolute gate is 1 cm + the seated baseline); a waiver is printed with its numbers
      $bw = $null; if ($classTable[$cls].bodyWaive -and -not $NoBodyWaive) { $bw = $classTable[$cls].bodyWaive["${g}:$seat"] }
      # a waiver is @(body cm, roof cm); the roof element may be left out / $null (the roof phase then keeps its absolute gate), and so may the body element ($null)
      $bodyTol = @()
      if ($bw) {
        if ($null -ne $bw[0]) { $bodyTol += @('--body-tol-cm', "$($bw[0])") }
        if ($bw.Count -gt 1 -and $null -ne $bw[1]) { $bodyTol += @('--roof-tol-cm', "$($bw[1])") }
        "body / roof waiver for ${clip}: body $($bw[0]) cm, roof $(if ($bw.Count -gt 1) { $bw[1] } else { 'none' }) cm (documented exception)"
      }
      $dwv = $classTable[$cls].doorWaive["${g}:$seat"]
      if ($null -ne $dwv) { $doorTol = @('--open-tol-cm', "$dwv", '--opening-tol-cm', "$dwv"); "door waiver (opening/open phases only) for ${clip}: $dwv cm" }
      $doorFrag = "$run\door-close-$clip.json"
      if (Test-Path $doorFrag) { Remove-Item $doorFrag }
      python -I "$tools\check_door_clearance.py" --clip "$run\roundtrip\$clip.anims.glb" --class $cls --seat $seat --tol-cm $DoorTolCm @doorTol @bodyTol --rear-ref $tgtGlb --solve --fragment $doorFrag |
        Tee-Object -FilePath "$run\door-$clip.txt"
      $de = "$LASTEXITCODE"
      $caS = 0.0
      if (!(Test-Path $doorFrag)) { "door check CRASHED for ${clip} (no fragment)"; $de = 'CRASH' }
      else {
        $fr = Get-Content -Raw $doorFrag | ConvertFrom-Json
        if ($fr.feasible) {
          $caS = [double]$fr.close_after_s
          if (($caS + [double]$fr.event_clip_s) -gt ([double]$fr.clip_end_s + 0.5)) { "door close ${clip}: close_after $caS s is later than the clip end + 0.5 s"; $de = 'LATE' }
        } else { "door check ${clip}: no safe close time (the seated pose collides with the closing door)"; $de = 'NOFEAS' }
      }
      $checks += "$clip door=$de close_after=$caS"
      # approach gate: V from the mount spot (vanilla slide + native root hold with the planned door-aware cap + the door from the mount line), same opening / open tolerances, no waiver
      if (-not $SkipApproach) {
        $baseLine = Get-Content $ApproachBaselineSources | Where-Object { $_ -like "$clip  *" } | Select-Object -First 1
        if (-not $baseLine) { throw "no shipped baseline for $clip in $ApproachBaselineSources" }
        $baseGlb = (($baseLine -split '  ', 2)[1]).Trim() + "\roundtrip\$clip.anims.glb"
        $apx = @('--approach-baseline', $baseGlb, '--hold-cap', 'on', '--align', $ApproachAlign, '--jobs', "$ApproachJobs", '--rear-ref', $tgtGlb, '--report', "$run\approach-$clip.json")
        if (-not $ApproachGrid) { $apx += '--no-grid' }
        if ($ArrivalRing) { $apx += '--arrival' }
        python -I "$tools\check_door_clearance.py" --approach --clip "$run\roundtrip\$clip.anims.glb" --class $cls --seat $seat --tol-cm $DoorTolCm @doorTol @apx | Tee-Object -FilePath "$run\approach-$clip.txt" | Select-Object -Last 3
        $checks += "$clip approach=$LASTEXITCODE"
      }
      if (!$SkipSelftest -and $caS -gt 0) {
        python -I "$tools\check_door_clearance.py" --clip "$run\roundtrip\$clip.anims.glb" --class $cls --seat $seat --tol-cm $DoorTolCm @doorTol @bodyTol --rear-ref $tgtGlb --close-after $caS --selftest | Select-Object -Last 8
        if ($LASTEXITCODE -ne 0) { throw "selftest: door check does not reject the never-opens / opens-late / closes-at-once / vanilla-close / edge mutations ($clip)" }
        # (n) anatomical-limit mutations: Hips + UpLeg joints 45 cm lower (pelvis floor / knee + hip flexion) and the trunk bent 60 deg further forward (trunk lean / neck flexion)
        # must each be rejected; (c): the shift assertion must reject a wrong shift
        python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --tpp-idle $tgtGlb --tpp-idle-clip $endClip --duck-mut 0.45 @ckx --expect-fail n_no_overduck_ok | Select-Object -Last 1
        if ($LASTEXITCODE -ne 0) { throw "selftest: (n) accepts a clip with the pelvis 45 cm lower ($clip)" }
        python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --tpp-idle $tgtGlb --tpp-idle-clip $endClip --lean-mut 60 @ckx --expect-fail n_no_overduck_ok | Select-Object -Last 1
        if ($LASTEXITCODE -ne 0) { throw "selftest: (n) accepts a clip with the trunk bent 60 deg further forward ($clip)" }
        python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --tpp-idle $tgtGlb --tpp-idle-clip $endClip --neck-mut 60 @ckx --expect-fail n_no_overduck_ok | Select-Object -Last 1
        if ($LASTEXITCODE -ne 0) { throw "selftest: (n) accepts a clip with the head bent 60 deg forward about the neck ($clip)" }
        if ($shifted) {
          python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --tpp-idle $tgtGlb --tpp-idle-clip $endClip --expect-start-shift ($shiftX + 0.05) --expect-start-shift-z $doorBack --expect-fail c_start_in_seat_ok | Select-Object -Last 1
          if ($LASTEXITCODE -ne 0) { throw "selftest: (c) accepts a start shift 5 cm off the asserted one ($clip)" }
          python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --tpp-idle $tgtGlb --tpp-idle-clip $endClip --expect-fail c_start_in_seat_ok | Select-Object -Last 1
          if ($LASTEXITCODE -ne 0) { throw "selftest: (c) without the shift assertion accepts the shifted start ($clip)" }
        }
      }
      if (!$SkipSelftest) {
        # mutations (builder + check only): each must be rejected by exactly the check it targets
        $st = "$run\selftest"
        $noik = @("$tools\build_entry_clip.py", '--src', $npcGlb, '--src-clip', 'enter', '--target', $tgtGlb, '--target-clip', 'enter', '--name', 'enter',
          '--end-pose', "${tgtGlb}:$endClip", '--cap-margin', "$CapMargin") + $common + $extra
        python -I @noik --out "$st\$clip.nolimbik.glb" --report "$st\$clip.nolimbik.json" | Out-Null; Check "selftest build nolimbik $clip"
        python -I $checkCar --clip "$st\$clip.nolimbik.glb" --vanilla $tgtGlb --src-rig $npcGlb --build-report "$st\$clip.nolimbik.json" --tpp-idle $tgtGlb --tpp-idle-clip $endClip `
          --expect-fail k_limb_ik_helpers_ok | Select-Object -Last 1; if ($LASTEXITCODE -ne 0) { throw "selftest: (k) passes without --limb-ik-final ($clip)" }
        # the relaxed-cap mutation must not carry the GEOMETRIC roof cap (it would lower the head anyway and the mutation would no longer be '12 cm too high')
        $exNR = @(); $skipN = 0; foreach ($ea in $extra) { if ($skipN -gt 0) { $skipN--; continue }; if ($ea -match '^--roof-(json|seat|margin)$') { $skipN = 1; continue }; $exNR += $ea }
        $hi = @("$tools\build_entry_clip.py", '--src', $npcGlb, '--src-clip', 'enter', '--target', $tgtGlb, '--target-clip', 'enter', '--name', 'enter',
          '--end-pose', "${tgtGlb}:$endClip") + $common + $exNR + @('--cap-margin', '0.12')
        python -I @hi --limb-ik-final --out "$st\$clip.hicap.glb" --report "$st\$clip.hicap.json" | Out-Null; Check "selftest build hicap $clip"
        python -I $checkCar --clip "$st\$clip.hicap.glb" --vanilla $tgtGlb --src-rig $npcGlb --build-report "$st\$clip.hicap.json" --tpp-idle $tgtGlb --tpp-idle-clip $endClip `
          --expect-fail l_head_roof_ok | Tee-Object -Variable hiOut | Select-Object -Last 1
        if ($LASTEXITCODE -ne 0) {
          # Allowed only when the check really passed ('EXPECT-FAIL ... FAIL (passes: ...)', not a crash) AND the relaxed cap lowered
          # nothing (hicap build report duck.hips_drop_max_m < 5 mm): the cap is not binding for this clip (its source head already
          # stays under the vanilla envelope, e.g. a tall cab), so the relaxed-cap mutation is a no-op. Then prove instead that (l)
          # rejects this clip's own head 12 cm higher on the entry-motion frames (the last 18 frames - end blend 0.4 s + end ease 0.2 s,
          # pinned to the vanilla seated pose by (b) - stay unraised, so the end pose cannot decide it).
          $hiDrop = [double](Get-Content -Raw "$st\$clip.hicap.json" | ConvertFrom-Json).duck.hips_drop_max_m
          if (-not (($hiOut | Out-String) -match 'EXPECT-FAIL l_head_roof_ok FAIL \(passes') -or $hiDrop -ge 0.005) {
            throw "selftest: (l) passes with a head cap 12 cm too high ($clip, hicap hips drop $hiDrop m)"
          }
          python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $tgtGlb --src-rig $npcGlb --head-raise 0.6 --tpp-idle $tgtGlb --tpp-idle-clip $endClip `
            --head-raise-skip-end 4 --expect-fail l_head_roof_ok | Select-Object -Last 1
          if ($LASTEXITCODE -ne 0) { throw "selftest: (l) passes with the head 60 cm higher before the end blend ($clip)" }
          "selftest (l): cap not binding for $clip (hicap hips drop $hiDrop m), head-raise mutation rejected"
        }
        $other = if ($seat -eq 'driver') { "$glb\p${g}a_${cls}_passenger_r.anims.glb" } else { "$glb\p${g}a_${cls}_driver.anims.glb" }
        $otherClip = if ($seat -eq 'driver') { 'idle' } else { 'tpp_idle' }
        python -I $checkCar --clip "$run\roundtrip\$clip.anims.glb" --vanilla $other --src-rig $npcGlb --tpp-idle $other --tpp-idle-clip $otherClip `
          --expect-fail b_end_pose_ok,c_start_in_seat_ok,m_seat_settle_ok | Select-Object -Last 1; if ($LASTEXITCODE -ne 0) { throw "selftest: (b)/(c) pass against the other seat's vanilla clip ($clip)" }
      }
    }
  }
}
"CHECKS: " + ($checks -join '; ')
if ($checks -match 'exit=[^0]|door=(?!0 )|approach=[^0]') { throw 'offline check failed' }
Get-ChildItem $depot -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.Name }
"RUN=$run"

# Car EXIT clips with the head duck (2026-11-05): the vanilla 'exit' of the player driver set (p?a_<class>_driver) with a neck / upper-spine bend + Hips sink that keeps V's head under the
# roof model while she leans out of the seat (build_exit_duck.py -> head_duck.py --duck-anchor first; gates in check_car_exit.py). Output set: utpp_car_p?a_<class>_driver_exit.anims with ONE clip
# named 'exit', registered by make_car_workspot.py with the variables / priorities of the class driver 'enter' set (appended after every entry item; the entry sets are not touched).
# Same import route as build_car.ps1: vanilla set reduced to the clip (animset_subset.py) -> WolvenKit deserialize -> WkitBridge import of the authored GLB -> serialize -> patch_motion_extraction.py
# (here with --vanilla-json: the vanilla motionExtraction block is copied verbatim = root motion byte-identical) -> deserialize -> export -> verify_roundtrip.py -> check_car_exit.py on the ROUND-TRIPPED clip.
# -Identity builds the same pipeline without the duck (control: the pipeline's own round trip noise). Writes only <Out> (default Build\work\vehicle-entry-car\vehexit-<stamp>); nothing is installed.
# Enabled: every driver class x gender row of the table below (sport2 w = the test car, V = pwa, the first row); a missing key = that exit stays vanilla.
# -Steps <steps.npz> -StepsBase <run dir of the head-duck build>: real stepping instead of the glide (2026-11-05; Source/tools/ai_motion\exit_steps_gen.py = MotionBricks in-betweening of the exit tail, foot pinning, leg IK; exit_steps.py splices
# frames c..g of the generated Hips / spine / legs into the clip after the duck, re-anchors the limb IK helpers; Root, events, float tracks, every frame outside c..g stay the head-duck clip). The exit gates then run in steps mode:
# frames outside c..g identical to the head-duck run (-StepsBase), foot slide while carrying weight, step count, floor, knee rate (check_car_exit.py --steps), the roof / door / shell gates unchanged.
# Pack: pack_car_workspot.ps1 -AnimsRun <entry runs...>,<this run> -Out <new dir>.
param(
  [string]$Stamp = (Get-Date -Format 'yyyyMMdd-HHmmss'),
  [string]$Out = '',
  [string]$Class = 'sport2',
  [string]$Gender = 'w',
  [switch]$Identity,
  [string[]]$DuckExtra = @(),
  [string]$Steps = '',
  [string]$StepsBase = '',
  [int]$StepMarginDeg = 5,
  [switch]$BuilderOnly,   # screening: only the builder GLB and the gates on it (no WolvenKit round trip; the checker defaults as tolerances)
  [int]$MinSteps = 0,   # steps mode: --min-steps of check_car_exit (default 0 = the checker's own 2). A row whose vanilla exit already counts 2 steps from the chosen c (standard2, standard25, sport2 m with c=14) needs 3 (standard25 was built with 4, stricter than needed), or the selftest 'vanilla_glide_fails_steps' cannot hold; pass it with -Steps.
  [string]$Bridge = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Bridge) { $Bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$ex = Join-Path $root 'Extracted\game\vehicle-entry-car'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$run = if ($Out) { $Out } else { Join-Path $root "Build\work\vehicle-entry-car\vehexit-$Stamp" }
if (Test-Path $run) { throw "output exists, choose a new -Out: $run" }
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
# exit duck table: 'class:g' -> head_duck options (the driver seat only). sport2 / w: neck + upper spine chain, bend toward the door side (azimuth -30 = door and forward), Hips sink profile
# capped by the rising Hips (never below the seated Hips - 1 cm: check_car_exit l_hips_seat) and gone by frame 26 before the pelvis crosses the sill (check_car_exit c2_full_shell_rel), no hand relock (the arms ride with the torso; the relock steps the forearm / hand over the clip's own step).
$exitTable = @{
  'sport2:w' = @('--duck-chain', 'Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.2,Neck1:0.1', '--duck-az=-30', '--duck-max-deg', '50', '--duck-step-deg', '1', '--duck-fade', '0.034', '--duck-vmax-deg', '5',
                 '--duck-smooth', '1.0', '--duck-slack-cm', '0.8', '--duck-sink-profile', '0:0,6:2,10:4,14:7,17:9,20:5.5,23:2.5,26:0')
  # ---- 2026-11-05 all driver classes x genders (screened with check_car_exit on the builder GLB, verdict = the round-trip gate of this script; run dirs Build/work/vehicle-entry-car/vehexit-all-20261105/<class>-<g>).
  # Vanilla exits of ALL rows are over the roof gate (a_roof_gate on the vanilla clip), so every row has a duck. Options per class: standard2 (both genders): neck + whole spine chain, bend ~forward (az -5), NO Hips sink (a sink
  # pushes the thigh / shin into the door sill: c2_full_shell_rel); standard25: 2 cm Hips sink + <= 2.3 deg bend (the vanilla exit is only 1.6 cm over the gate for 0.17 s); sport1: Spine3 / neck chain only, forward, no sink (the vanilla Hips
  # already go 8-10 cm BELOW the seated Hips, check l_hips_seat allows only 1 cm below min(seated, vanilla) there; the lifting door gates the head / thigh: c_door_open_rel); sport2 m: the sport2 w row with the sink capped by the male Hips
  # (+ ik-bend-sigma 0.1 for the door gate of shin_l at frame 16); utility: forward bend, no sink.
  'standard2:w' = @('--duck-chain', 'Spine:0.1,Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.15,Neck1:0.05', '--duck-az=-5', '--duck-max-deg', '70', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8', '--duck-move-cost', '0.5')
  'standard2:m' = @('--duck-chain', 'Spine:0.1,Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.15,Neck1:0.05', '--duck-az=-5', '--duck-max-deg', '70', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8', '--duck-move-cost', '0.5')
  'standard25:w' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=0', '--duck-max-deg', '60', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '1.0', '--duck-slack-cm', '0.4', '--duck-sink-profile',
                 '8:0.00,9:0.13,10:0.50,11:1.00,12:1.38,13:1.70,14:2.00,15:2.00,16:2.00,17:2.00,18:2.00,19:2.00,20:2.00,21:1.87,22:1.50,23:1.00,24:0.50,25:0.13,26:0.00')
  'standard25:m' = @('--duck-chain', 'Spine1:0.15,Spine2:0.2,Spine3:0.25,Neck:0.25,Neck1:0.15', '--duck-az=0', '--duck-max-deg', '60', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '1.0', '--duck-slack-cm', '0.4', '--duck-sink-profile',
                 '8:0.00,9:0.13,10:0.50,11:0.50,12:0.50,13:0.63,14:1.28,15:2.00,16:2.00,17:2.00,18:2.00,19:2.00,20:2.00,21:1.87,22:1.50,23:1.00,24:0.50,25:0.13,26:0.00')
  'sport1:w' = @('--duck-chain', 'Spine3:0.4,Neck:0.35,Neck1:0.25', '--duck-az=0', '--duck-max-deg', '25', '--duck-step-deg', '0.5', '--duck-fade', '0.034',
                 '--duck-vmax-deg', '10', '--duck-smooth', '0.5', '--duck-slack-cm', '0.2', '--duck-move-cost', '0.3')
  'sport1:m' = @('--duck-chain', 'Spine3:0.4,Neck:0.35,Neck1:0.25', '--duck-az=0', '--duck-max-deg', '25', '--duck-step-deg', '0.5', '--duck-fade', '0.034',
                 '--duck-vmax-deg', '10', '--duck-smooth', '0.5', '--duck-slack-cm', '0.2', '--duck-move-cost', '0.3')
  'sport2:m' = @('--duck-chain', 'Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.2,Neck1:0.1', '--duck-az=-30', '--duck-max-deg', '50', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '5', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8', '--duck-sink-profile',
                 '0:0.00,1:0.13,2:0.50,3:1.00,4:1.50,5:1.87,6:2.00,7:2.29,8:3.00,9:3.71,10:4.00,11:4.44,12:5.50,13:6.32,14:6.90,15:7.47,16:8.05,17:8.67,18:8.12,19:6.38,20:5.50,21:4.75,22:3.25,23:2.50,24:1.88,25:0.63,26:0.00',
                 '--ik-bend-sigma', '0.1', '--ik-max-extend', '0.999')
  'utility:w' = @('--duck-chain', 'Spine:0.1,Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.15,Neck1:0.05', '--duck-az=0', '--duck-max-deg', '60', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '8', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8')
  'utility:m' = @('--duck-chain', 'Spine:0.1,Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.15,Neck1:0.05', '--duck-az=0', '--duck-max-deg', '60', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '8', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8')
  # standard3: the hardest row (the vanilla head is 13.6 cm into the roof at frame 20 against a 1 cm gate): ~52 deg bend toward the door side + a 1.5 cm Hips sink (capped by the vanilla Hips, which dip 5 cm below the seat); the chest
  # shell gate (c2_full_shell_rel) binds: a larger door-side azimuth pushes the chest into the sill, a smaller one needs > 60 deg
  'standard3:w' = @('--duck-chain', 'Spine:0.1,Spine1:0.2,Spine2:0.25,Spine3:0.25,Neck:0.15,Neck1:0.05', '--duck-az=-27', '--duck-max-deg', '80', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '1.0', '--duck-slack-cm', '0.8', '--duck-sink-profile',
                 '8:0.00,9:0.40,10:0.90,11:0.90,12:0.90,13:0.90,14:0.90,15:0.90,16:0.90,17:0.90,18:0.90,19:1.15,20:1.50,21:0.40,22:0.00', '--duck-move-cost', '0.3')
  'standard3:m' = @('--duck-chain', 'Spine:0.15,Spine1:0.2,Spine2:0.25,Spine3:0.2,Neck:0.15,Neck1:0.05', '--duck-az=-27', '--duck-max-deg', '80', '--duck-step-deg', '1',
                 '--duck-fade', '0.034', '--duck-vmax-deg', '12', '--duck-smooth', '0.5', '--duck-slack-cm', '0.8', '--duck-sink-profile',
                 '8:0.00,9:0.70,10:0.70,11:0.70,12:0.70,13:0.70,14:0.70,15:0.70,16:0.70,17:0.70,18:0.70,19:0.70,20:0.72,21:1.50,22:0.00', '--duck-move-cost', '0.3')
}
# every row except the first (sport2 w, kept byte-identical): the limb pole helpers are re-placed only on frames where that limb moved (build_exit_duck.py --duck-pole-min-move-cm; the unchanged leg of a row without a Hips sink keeps its vanilla pole)
foreach ($k in @($exitTable.Keys)) { if ($k -ne 'sport2:w') { $exitTable[$k] = @($exitTable[$k]) + @('--duck-pole-min-move-cm', '0.2') } }
$key = "${Class}:${Gender}"
if (-not $exitTable.ContainsKey($key)) { throw "no exit duck row for $key (enabled: $($exitTable.Keys -join ', '))" }
$DuckExtra = @($DuckExtra | ForEach-Object { $_ -split ';' } | Where-Object { $_ -ne '' })
$set = "p${Gender}a_${Class}_driver"
$clip = "utpp_car_${set}_exit"
$vanGlb = Join-Path $ex "glb\$set.anims.glb"
$vanJson = Join-Path $ex "json\$set.anims.json"
$roofJson = Join-Path $ex "body-data\$Class.json"
$stage = Join-Path $run 'utpp_vehicle_car'
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot, "$run\glb", "$run\base-json", "$run\base", "$run\import-json", "$run\patched-json", "$run\roundtrip" | Out-Null
$glb = "$run\glb\$clip.anims.glb"
$bargs = @("$tools\build_exit_duck.py", '--glb', $vanGlb, '--clip', 'exit', '--name', 'exit', '--out', $glb, '--report', "$run\$clip.build-report.json")
if ($Identity) { $bargs += '--identity' }
else { $bargs += @('--duck-roof-json', $roofJson, '--duck-roof-seat', 'left') + $exitTable[$key] + $DuckExtra }
$stepsArgs = @(); $stepsBaseRt = @(); $stepsBaseB = @()
# utility: the pipeline round trip of the UNDUCKED utility exit (-Identity) already moves the foot IK helper bones by 0.155 deg / 0.59 mm at frame 20 (81-frame clip; measured 2026-11-05, vehexit-all-20261105\utility-w-identity),
# above the 0.05 deg / 0.5 mm 'bone changed' threshold of the round-trip check: g_unchanged_bones would flag leg_*_ik / legs_ik_A_master / Torso_*_Foot_IK_JNT although the duck never touches them. Threshold = measured noise floor x ~1.3 (0.2 deg / 1 mm).
$chkExtra = @(); if ($Class -eq 'utility') { $chkExtra = @('--chg-deg', '0.2', '--chg-m', '0.001') }
if ($Steps) {
  if ($Identity) { throw '-Steps and -Identity exclude each other' }
  if (-not $StepsBase) { throw '-Steps needs -StepsBase <head-duck run dir> (frames outside the stepping must equal it)' }
  $cg = (python -I -c "import numpy as np,sys;s=np.load(sys.argv[1],allow_pickle=True);print('%d:%d' % (int(s['c']), int(s['g'])))" $Steps); Check 'steps span'
  $bargs += @('--steps', $Steps)
  $stepsArgs = @('--steps', $cg, '--step-margin-deg', "$StepMarginDeg")
  if ($MinSteps -gt 0) { $stepsArgs += @('--min-steps', "$MinSteps") }
  $stepsBaseRt = @('--base', (Join-Path $StepsBase "roundtrip\$clip.anims.glb"))
  $stepsBaseB = @('--base', (Join-Path $StepsBase "glb\$clip.anims.glb"))
}
python -I @bargs | Tee-Object -FilePath "$run\build-$clip.txt" | Select-Object -Last 3; Check "build $clip"
if ($BuilderOnly) {
  python -I "$tools\check_car_exit.py" --clip $glb --vanilla $vanGlb --class $Class --seat driver --clip-name exit @stepsArgs @stepsBaseB @chkExtra --report "$run\check-exit-builder.json" --work "$run\chk" | Tee-Object -FilePath "$run\check-exit-builder.txt" | Select-Object -Last 1
  "BUILDER_ONLY run=$run"; exit 0
}
python -I "$tools\animset_subset.py" $vanJson "$run\base-json\$clip.anims.json" --keep exit | Out-Null; Check "subset $clip"
& $wkit convert deserialize "$run\base-json\$clip.anims.json" -o "$run\base" | Out-Null; Check "deserialize base $clip"
& $Bridge import $glb "$run\base\$clip.anims" "$run\import-$clip" | Select-String 'ANIMS_IMPORT_EXIT'; Check "import $clip"
& $wkit convert serialize "$run\import-$clip\$clip.anims" -o "$run\import-json" | Out-Null; Check "serialize $clip"
python -I "$tools\patch_motion_extraction.py" "$run\import-json\$clip.anims.json" $glb "$run\patched-json\$clip.anims.json" --clip exit --vanilla-json "$run\base-json\$clip.anims.json" | Out-Null; Check "patch $clip"
& $wkit convert deserialize "$run\patched-json\$clip.anims.json" -o $depot | Out-Null; Check "deserialize final $clip"
& "$tools\export_glb.ps1" -Files "$depot\$clip.anims" -OutDir "$run\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $clip"
python -I "$tools\verify_roundtrip.py" $glb "$run\roundtrip\$clip.anims.glb" "$run\patched-json\$clip.anims.json" --clip exit --report "$run\$clip.roundtrip-report.json" | Out-Null; Check "verify $clip"
if ($Identity) { "IDENTITY_OK RUN=$run"; exit 0 }
python -I "$tools\check_car_exit.py" --clip "$run\roundtrip\$clip.anims.glb" --vanilla $vanGlb --class $Class --seat driver --clip-name exit --selftest --tol-open-cm 0.7 --tol-end-deg 0.05 --chg-deg 0.05 --chg-m 0.0005 --tol-end-m 0.0005 @stepsArgs @stepsBaseRt @chkExtra --report "$run\check-exit-roundtrip.json" --work "$run\chk" | Tee-Object -FilePath "$run\check-exit-roundtrip.txt" | Select-Object -Last 16
$ce = $LASTEXITCODE   # --tol-open-cm 0.7 (default 0.5): the door cloud penetration jumps by 0.3-0.6 cm between neighbouring poses; with 0.5 shin_l fails by 0.13 cm at ONE frame (door start delay 0.2 s), documented in the report
python -I "$tools\check_car_exit.py" --clip $glb --vanilla $vanGlb --class $Class --seat driver --clip-name exit @stepsArgs @stepsBaseB --report "$run\check-exit-builder.json" --work "$run\chk" | Tee-Object -FilePath "$run\check-exit-builder.txt" | Select-Object -Last 1
"EXIT_BUILD run=$run check_roundtrip_exit=$ce"

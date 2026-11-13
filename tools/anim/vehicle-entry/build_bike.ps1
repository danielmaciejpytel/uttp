# Bike entry clips for every player bike (sportbike1 Yaiba Kusanagi, sportbike2, sportbike3) and both V genders: TPP 'enter_l' /
# 'enter_r' that END in the TPP standing-on-bike pose the game continues with. Source: NPC wa_/ma_sportbikeN enter_l / enter_r
# (retargeted to the player rig, time-warped to the vanilla 1.4 s with Catmull-Rom resampling), root end matched to the vanilla
# p{w,m}a_sportbikeN clip and then (diag 2026-10-28 c9, WP3):
#  - total root yaw scaled to 10 deg (vanilla 67): the engine slides the entity to seat * inverse(root delta) before the clip plays,
#    so the start spot beside the bike stays where the vanilla clip puts it, but V now faces along the bike instead of 67 deg away;
#  - feet: plant keyframes held exactly, min-jerk swings, the last swing lands by t = 1.0 s on the end pose foot (a re-plant step
#    up onto the peg at 1.2 s, after the hips have settled; two-bone IK);
#  - root replaced by an endpoint-preserving smoothed root (speed rule of check_bike_entry.py (h)), world body unchanged;
#  - end pose for both sides: tpp idle_stand_bike_begin_l, the pose the game continues with (in-game traces 2026-10-28: enter_l
#    and enter_r rides both continue with it); the player graph (base\gameplay\anim_graphs\player_vehicle.animgraph) picks the standing idle with an animAnimNode_Switch
#   (weight latched at entry) = (vehicleRoll < -1) or (vehicleRoll < 1 and inputLR < -0.3), independent of the entry side:
#   a parked bike leans left -> idle_stand_bike_begin_l (left foot on the ground, right foot on the peg);
#    legs blend over 0.6 s, body 0.4 s;
#  - quintic ease-in (0.2 s) from the exact start pose and ease-out into the exact end pose (helpers 0.6 s);
#  - per-bone step limit 11 deg/frame (feathered local passes, planted leg frames frozen); reference_joint fixed at the seat;
#  - limb IK helpers (feet / hands: vanilla offsets to the bone; leg / arm poles: vanilla coordinates in the limb plane) placed
#    on the FINAL pose (the game's leg IK pulls the feet onto leg_*_ik).
# check_bike_entry.py (a)..(j) runs on every round-tripped pair; any violation fails the run before the .xl / pack steps
# (-MaxJointAccelCm sets the (i) limit, default 16 cm/frame^2 = vanilla maximum, decision 2026-10-28). Self-tests: --against-fpp
# must fail, --selftest-old on the pre-c9 clips of -OldRun must fail checks (e)..(h) and (k), and (k) must fail on the c9 clips of
# -C9Run (limb IK helpers placed before the final foot edits: 3-23 cm off the feet).
# Prerequisites: Extracted\game\vehicle-bike\raw + glb (WolvenKit unbundle/export with root motion, game read-only),
# Build\work\vehicle-entry-archive\bridge-bin\WkitBridge.exe. Writes only <Out> (default Build\work\vehicle-bike-archive\c9-clips-<stamp>,
# GLBs and build reports in <Out>\glb) and refreshes tools\utpp_vehicle_bike.archive.xl (same content for unchanged clip paths).
# -EnterLift (opt-in, default off): bike_enter_lift.py turns ground glide of a foot into a short step before the WolvenKit import (<Out>\glb\*.lift-report.json). Nothing is installed. Pack with the workspot override: pack_bike_workspot.ps1 -AnimsRun <Out> -Out <new dir>.
param(
  [string]$Stamp = (Get-Date -Format 'yyyyMMdd-HHmmss'),
  [string]$Out = '',
  [double]$MaxJointAccelCm = 16.0,
  [switch]$EnterLift,   # opt-in (2026-11-06): glide episodes of a foot in ground contact become short steps (Source\tools\ai_motion\bike_enter_lift.py: vertical arc + leg IK, horizontal path / hips / arms unchanged); default off = the c9 clips unchanged
  [string]$C9Run = '',
  [string]$OldRun = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $C9Run) { $C9Run = Join-Path $root 'Build\work\vehicle-bike-archive\c9-clips-e2' }
if (-not $OldRun) { $OldRun = Join-Path $root 'Build\work\vehicle-bike-archive\run-20261028-074600' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$glb = Join-Path $root 'Extracted\game\vehicle-bike\glb'
$rawv = Join-Path $root 'Extracted\game\vehicle-bike\raw\base\animations\vehicle'
$run = if ($Out) { $Out } else { Join-Path $root "Build\work\vehicle-bike-archive\c9-clips-$Stamp" }
if (Test-Path $run) { throw "output exists, choose a new -Out: $run" }
$anim = Join-Path $run 'glb'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$bridge = Join-Path $root 'Build\work\vehicle-entry-archive\bridge-bin\WkitBridge.exe'
if (-not (Test-Path $bridge)) { $bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }   # 2026-11-06: the entry-archive copy was removed; the car copy is the same tool
$stage = Join-Path $run 'utpp_vehicle_bike'
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot, $anim | Out-Null
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$variants = @()
foreach ($g in @('w', 'm')) { foreach ($b in @('', '2', '3')) { $variants += , @($g, $b) } }
$xl = @"
# UTPP third-person motorcycle entry (female and male V, sportbike1 Kusanagi / sportbike2 / sportbike3). Own clip sets, one clip
# each ('enter_l' / 'enter_r'), built offline from the game's NPC wa_/ma_sportbike entry clips (retargeted to the player rig, 1.4 s
# like the vanilla clips) whose last 0.4 s blend into the TPP seated pose (tpp_p?a_sportbike 'drive_slow_bike' frame 0). The vanilla
# FPP enter_l / enter_r end in the lower, hunched FPP seat pose and the game swaps to the TPP riding pose ~0.3 s later (recording
# 2026-10-28, sportbike1: Hips z -0.007 -> +0.082; logged seat step 9.1 cm / 28 deg).
# Route 'root' (A/B for the precedence direction): these entries are merged into the root component's own list; at priority 94 they
# never played (every TPP ride of 2026-10-28 still showed the FPP enter). Priority 130 tests 'higher number wins' (above the vanilla
# FPP 119 / TPP 100 entries of MotorcycleAnimsets). The other route, the ``resource: patch:`` section at the end of this file, puts
# the same sets on an own extension component next to MotorcycleAnimsets (priority 94, lower number wins as in the vanilla data).
# Activation: wrapper variable utpp_vehicle_tpp = 1 AND the route variable utpp_bike_route_root = 1 (utpp_vehicle_anim.reds, bike
# mount in UTPP third person, route bit 2 of Game.UTPPSetVehicleBikeRoute) AND the vanilla seat variable, so first-person riding
# keeps the vanilla clips. The route in use is listed in the log line "[utpp_vehicle_anim] ride N: workspot animVariables [...]".
animations:
"@
$checks = @()
foreach ($v in $variants) {
  $g = $v[0]; $b = $v[1]; $seatNo = if ($b -eq '') { '1' } else { $b }
  $dir = if ($b -eq '') { 'sportbike' } else { "sportbike$b" }
  $tgtName = "p${g}a_sportbike$b"; $npcName = "${g}a_sportbike$b"; $tppName = "tpp_p${g}a_sportbike$b"
  $tgtGlb = "$glb\$tgtName.anims.glb"; $npcGlb = "$glb\$npcName.anims.glb"; $tppGlb = "$glb\$tppName.anims.glb"
  $rawset = Join-Path $rawv "$dir\character\$tgtName.anims"
  $vj = Join-Path $run "vanilla-json-$tgtName"
  New-Item -ItemType Directory -Force $vj | Out-Null
  & $wkit convert serialize $rawset -o $vj | Out-Null; Check "serialize vanilla $tgtName"
  $ent = if ($g -eq 'w') { @('base\characters\entities\player\player_wa_fpp.ent', 'ep1\characters\entities\player\player_wa_fpp_ep1.ent') }
         else { @('base\characters\entities\player\player_ma_fpp.ent', 'ep1\characters\entities\player\player_ma_fpp_ep1.ent') }
  foreach ($c in @('l', 'r')) {
    $clip = "utpp_tpp_p${g}a_sportbike${b}_enter_$c"
    $out = Join-Path $anim "$clip.anims.glb"
    $pose = 'idle_stand_bike_begin_l'  # both sides (see the header)
    python -I "$tools\build_entry_clip.py" --src $npcGlb --src-clip "enter_$c" --target $tgtGlb --target-clip "enter_$c" --out $out --name "enter_$c" `
      --target-duration --end-pose "${tppGlb}:$pose" --end-blend 0.4 --end-blend-legs 0.6 --head-cap none --corr-start 0.25 --corr-end 0.8 `
      --smooth-rot-limit 11 --warp-interp cubic --limit-pad 3 --root-yaw-total 10 --plant-hold --land-by 1.0 --replant-raised-by 1.2 --ik-max-extend 0.999 `
      --ik-bend-sigma 2 --limb-ik-final --root-smooth --ease-in 0.2 --end-ease 0.2 --end-ease-helpers 0.6 --ref-joint-fixed `
      --report "$anim\$clip.build-report.json" | Out-Null; Check "build $clip"
    if ($EnterLift) {
      Copy-Item $out "$anim\$clip.prelift.anims.glb"
      & (Join-Path $root 'Tools\ai-motion\envs\kimodo\Scripts\python.exe') -I (Join-Path $tools '..\..\ai_motion\bike_enter_lift.py')"$anim\$clip.prelift.anims.glb" "enter_$c" $out --report "$anim\$clip.lift-report.json" | Tee-Object -FilePath "$anim\$clip.lift.txt" | Out-Null; Check "enter lift $clip"
    }
    python -I "$tools\animset_subset.py" "$vj\$tgtName.anims.json" "$run\base-json\$clip.anims.json" --keep "enter_$c" | Out-Null; Check "subset $clip"
    New-Item -ItemType Directory -Force "$run\base" | Out-Null
    & $wkit convert deserialize "$run\base-json\$clip.anims.json" -o "$run\base" | Out-Null; Check "deserialize base $clip"
    & $bridge import $out "$run\base\$clip.anims" "$run\import-$clip" | Select-String 'ANIMS_IMPORT_EXIT'; Check "import $clip"
    New-Item -ItemType Directory -Force "$run\import-json" | Out-Null
    & $wkit convert serialize "$run\import-$clip\$clip.anims" -o "$run\import-json" | Out-Null; Check "serialize $clip"
    python -I "$tools\patch_motion_extraction.py" "$run\import-json\$clip.anims.json" $out "$run\patched-json\$clip.anims.json" --clip "enter_$c" | Out-Null; Check "patch $clip"
    & $wkit convert deserialize "$run\patched-json\$clip.anims.json" -o $depot | Out-Null; Check "deserialize final $clip"
    & "$tools\export_glb.ps1" -Files "$depot\$clip.anims" -OutDir "$run\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $clip"
    python -I "$tools\verify_roundtrip.py" $out "$run\roundtrip\$clip.anims.glb" "$run\patched-json\$clip.anims.json" --clip "enter_$c" --report "$run\$clip.roundtrip-report.json" | Out-Null
    Check "verify $clip"
    $xl += "`n  - entity:`n"
    foreach ($e in $ent) { $xl += "      - $e`n" }
    $xl += "    component: root`n    set: utpp\vehicle\$clip.anims`n    vars: [utpp_vehicle_tpp, utpp_bike_route_root, sportbike${seatNo}_vehicle_seat_front_left]`n    priority: 130`n"
  }
  $cn = "p${g}a_sportbike$b"
  python -I "$tools\check_bike_entry.py" --src-rig $npcGlb --enter-l "$run\roundtrip\utpp_tpp_${cn}_enter_l.anims.glb" `
    --enter-r "$run\roundtrip\utpp_tpp_${cn}_enter_r.anims.glb" --tpp $tppGlb --report "$run\check-$cn.json" `
    --build-report-l "$anim\utpp_tpp_${cn}_enter_l.build-report.json" --build-report-r "$anim\utpp_tpp_${cn}_enter_r.build-report.json" `
    --max-joint-accel-cm $MaxJointAccelCm | Tee-Object -FilePath "$run\check-$cn.txt"
  $checks += "$cn exit=$LASTEXITCODE"
  python -I "$tools\check_bike_entry.py" --src-rig $npcGlb --enter-l "$run\roundtrip\utpp_tpp_${cn}_enter_l.anims.glb" `
    --enter-r "$run\roundtrip\utpp_tpp_${cn}_enter_r.anims.glb" --tpp $tppGlb --fpp $tgtGlb --against-fpp | Out-Null
  if ($LASTEXITCODE -eq 0) { throw "check self-test: $cn passes against the FPP pose (the check does not detect the collapsed pose)" }
  $oldL = Join-Path $OldRun "roundtrip\utpp_tpp_${cn}_enter_l.anims.glb"
  $oldR = Join-Path $OldRun "roundtrip\utpp_tpp_${cn}_enter_r.anims.glb"
  # the self-test uses the specified limits (a relaxed -MaxJointAccelCm would let (i) pass on the old clips)
  if (Test-Path $oldL) {   # 2026-11-06: the pre-c9 roundtrip clips of -OldRun were removed from the work dir; the self-test runs when they exist
  python -I "$tools\check_bike_entry.py" --src-rig $npcGlb --enter-l $oldL --enter-r $oldR --tpp $tppGlb --selftest-old `
    | Out-File -Encoding ascii "$run\selftest-old-$cn.txt"
  if ($LASTEXITCODE -ne 0) { throw "check self-test: a new check (e..h, k) passes on the pre-c9 clips of $cn (see selftest-old-$cn.txt)" }
  } else { "self-test old skipped: $oldL missing" | Out-File -Encoding ascii "$run\selftest-old-$cn.txt" }
  $c9L = Join-Path $C9Run "roundtrip\utpp_tpp_${cn}_enter_l.anims.glb"
  $c9R = Join-Path $C9Run "roundtrip\utpp_tpp_${cn}_enter_r.anims.glb"
  if ((Test-Path $c9L) -and (Test-Path $c9R)) {   # 2026-11-06: the self-test needs the c9 clips of -C9Run; skipped when they are gone
  python -I "$tools\check_bike_entry.py" --src-rig $npcGlb --enter-l $c9L --enter-r $c9R --tpp $tppGlb --expect-fail k_limb_ik_helpers_ok `
    | Out-File -Encoding ascii "$run\selftest-c9-$cn.txt"
  if ($LASTEXITCODE -ne 0) { throw "check self-test: (k) passes on the c9 clips of $cn (see selftest-c9-$cn.txt)" }
  } else { "self-test c9 skipped: clips missing in $C9Run" | Out-File -Encoding ascii "$run\selftest-c9-$cn.txt" }
}
"CHECKS: " + ($checks -join '; ')
if ($checks -match 'exit=[^0]') { throw 'offline check failed' }
New-Item -ItemType Directory -Force "$run\xl" | Out-Null
$xlCand = Join-Path $run 'xl\utpp_vehicle_bike.archive.xl'
Set-Content -Path $xlCand -Value $xl -Encoding ascii
$tppXl = Join-Path $tools '..\..\..\plugin\UTPP\archive\utpp_vehicle_tpp.archive.xl'
$animReds = Join-Path $tools '..\..\..\plugin\UTPP\scripts\utpp_vehicle_anim.reds'
$gameDump = Join-Path $root 'Build\work\compat-2.5\codeware-modded-250.reds'
# route 'ext': the extension component patch (utpp\vehicle\utpp_bike_animsets_{wa,ma}.ent) is packed into the same archive
python -I "$tools\make_bike_ext_patch.py" --wkit $stage --wkit-exe $wkit; Check 'ext patch'
$extSection = (python -I "$tools\make_bike_ext_patch.py" --print-xl) -join "`n"; Check 'ext xl section'
Set-Content -Path $xlCand -Value ($xl.TrimEnd("`n") + "`n" + $extSection) -Encoding ascii
python -I "$tools\check_bike_xl.py" $xlCand --tpp-xl $tppXl --depot $depot --reds $animReds --game-dump $gameDump `
  --ext-json (Join-Path $tools 'patch-json') --vanilla-json (Join-Path $root 'Extracted\game\vehicle-anims-2.31\json') --selftest
Check 'xl validation'
Copy-Item $xlCand (Join-Path $tools 'utpp_vehicle_bike.archive.xl') -Force
& $wkit pack $stage -o "$run\packed"; Check 'pack'
Copy-Item (Join-Path $tools 'utpp_vehicle_bike.archive.xl') "$run\packed\"
& $wkit unbundle (Get-ChildItem "$run\packed\*.archive" | Select-Object -First 1).FullName -o "$run\unpacked" | Out-Null; Check 'unbundle'
Get-ChildItem $depot -File | ForEach-Object {
  $u = Join-Path "$run\unpacked\utpp\vehicle" $_.Name
  $a = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
  $bb = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  '{0} equal={1}' -f $_.Name, ($a -eq $bb)
}
Get-ChildItem "$run\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"RUN=$run"

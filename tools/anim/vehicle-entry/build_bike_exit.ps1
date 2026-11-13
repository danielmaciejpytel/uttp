# Motorcycle NORMAL get-off clips (exit_l / exit_r of the player sets p?a_sportbike[N]) re-stepped offline (2026-11-06): the vanilla clips glide the planted feet over the ground; Source\tools\ai_motion\bike_steps_gen.py
# (MotionBricks in-betweening of the exit tail + foot pinning + leg IK, candidates screened for glide / hips deviation / bike clearance proxy) writes the chosen GLB per clip to <PickRoot>\<g>a_sb<N>_<l|r>\pick.anims.glb
# (clip 'exit_l' / 'exit_r', frames outside the span identical to vanilla, Root untouched). This script runs the same import route as build_exit.ps1 / build_bike.ps1 for every available pick:
# vanilla set reduced to the clip (animset_subset.py) -> WolvenKit deserialize -> WkitBridge import -> serialize -> patch_motion_extraction.py --vanilla-json (root motion byte-identical) -> deserialize ->
# export -> verify_roundtrip.py -> check_bike_exit.py on the ROUND-TRIPPED clip (vs vanilla). Output sets: utpp_tpp_p?a_sportbike[N]_exit_<l|r>.anims, ONE clip exit_<l|r>.
# -Source tpp (default): the picks come from the TPP exits tpp_p?a_sportbike. The QUICK jump-out dismount clips (slide_exit / speed_exit / cool_exit / combat_exit) are never touched. A pick that is missing = that exit stays vanilla.
# Registration (opt-in, default off): make_bike_workspot.py --exit-sets / pack_bike_workspot.ps1 -ExitRun <this run dir>. Writes only <Out>; nothing is installed.
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [Parameter(Mandatory = $true)][string]$PickRoot,
  [ValidateSet('tpp', 'fpp')][string]$Source = 'tpp',   # vanilla set the picks were made from: tpp = tpp_p?a_sportbike (starts in the TPP riding pose, hips / hands / spine within a few cm, bike_startpose.py), fpp = p?a_sportbike (the clips the game plays today; start 5-23 cm / up to 32 deg off the TPP riding pose)
  [switch]$RootRetimed,   # walk-away variant (bike_exit_walk.py): LONGER clip + new Root path: the motion extraction is re-encoded from the GLB Root (no --vanilla-json), pole fix off (frame count differs), check_bike_exit.py --root-retimed
  [switch]$NoPoleFix,   # default (2026-11-06): bike_pole_fix.py on every pick before the import (the pole helpers of a limb that did not move keep the vanilla value; pma_sportbike3 exit_r had a 25 mm one-frame pole jump at frame 5 outside the changed span)
  [double]$MaxScore = 100,   # a pick whose best candidate scores >= this (bike_steps_gen.py pick: +1000 when the bike clearance proxy fails) is skipped = that exit stays vanilla
  [double]$VanillaPathTol = 0.10,   # -RootRetimed only: check_bike_exit.py --vanilla-path-tol (m); 0.11 for pma sportbike exit_r (clearance proxy loosened by 1 cm for that clip only)
  [string]$Bridge = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Bridge) { $Bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$aim = Join-Path $root 'Source\tools\ai_motion'
$kpy = Join-Path $root 'Tools\ai-motion\envs\kimodo\Scripts\python.exe'
$glbDir = Join-Path $root 'Extracted\game\vehicle-bike\glb'
$rawv = Join-Path $root 'Extracted\game\vehicle-bike\raw\base\animations\vehicle'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
$stage = Join-Path $Out 'utpp_vehicle_bike'
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot, "$Out\glb", "$Out\base-json", "$Out\base", "$Out\import-json", "$Out\patched-json", "$Out\roundtrip" | Out-Null
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$done = @(); $results = @()
foreach ($g in @('w', 'm')) { foreach ($b in @('', '2', '3')) { foreach ($c in @('l', 'r')) {
  $pick = Join-Path $PickRoot "p${g}a_sb${b}_$c\pick.anims.glb"
  if (-not (Test-Path $pick)) { "SKIP p${g}a_sportbike${b} exit_$c (no pick: stays vanilla)"; continue }
  $pj = Join-Path (Split-Path -Parent $pick) 'pick.json'
  if (Test-Path $pj) { $sc = (Get-Content $pj -Raw | ConvertFrom-Json).candidates[0].score; if ($sc -ge $MaxScore) { "SKIP p${g}a_sportbike${b} exit_$c (best candidate score ${sc} >= ${MaxScore}: stays vanilla)"; continue } }
  $dir = if ($b -eq '') { 'sportbike' } else { "sportbike$b" }
  $tgtName = "p${g}a_sportbike$b"; if ($Source -eq 'tpp') { $tgtName = "tpp_$tgtName" }; $clip = "utpp_tpp_p${g}a_sportbike${b}_exit_$c"; $cn = "exit_$c"
  $vanGlb = "$glbDir\$tgtName.anims.glb"
  $vj = Join-Path $Out "vanilla-json-$tgtName"
  if (-not (Test-Path $vj)) { New-Item -ItemType Directory -Force $vj | Out-Null; & $wkit convert serialize (Join-Path $rawv "$dir\character\$tgtName.anims") -o $vj | Out-Null; Check "serialize vanilla $tgtName" }
  $glb = "$Out\glb\$clip.anims.glb"
  if ($NoPoleFix -or $RootRetimed) { Copy-Item $pick $glb; $wj = Join-Path (Split-Path -Parent $pick) 'walk.json'; if ($RootRetimed -and (Test-Path $wj)) { Copy-Item $wj "$Out\glb\$clip.walk.json" } } else { & $kpy -I "$aim\bike_pole_fix.py" --glb $pick --vanilla $vanGlb --clip $cn --out $glb --report "$Out\polefix-$clip.json" | Select-Object -Last 1; Check "pole fix $clip" }
  python -I "$tools\animset_subset.py" "$vj\$tgtName.anims.json" "$Out\base-json\$clip.anims.json" --keep $cn | Out-Null; Check "subset $clip"
  & $wkit convert deserialize "$Out\base-json\$clip.anims.json" -o "$Out\base" | Out-Null; Check "deserialize base $clip"
  & $Bridge import $glb "$Out\base\$clip.anims" "$Out\import-$clip" | Select-String 'ANIMS_IMPORT_EXIT'; Check "import $clip"
  & $wkit convert serialize "$Out\import-$clip\$clip.anims" -o "$Out\import-json" | Out-Null; Check "serialize $clip"
  python -I "$tools\patch_motion_extraction.py" "$Out\import-json\$clip.anims.json" $glb "$Out\patched-json\$clip.anims.json" --clip $cn @(if ($RootRetimed) { @() } else { @("--vanilla-json", "$Out\base-json\$clip.anims.json") }) | Out-Null; Check "patch $clip"
  & $wkit convert deserialize "$Out\patched-json\$clip.anims.json" -o $depot | Out-Null; Check "deserialize final $clip"
  & "$tools\export_glb.ps1" -Files "$depot\$clip.anims" -OutDir "$Out\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $clip"
  python -I "$tools\verify_roundtrip.py" $glb "$Out\roundtrip\$clip.anims.glb" "$Out\patched-json\$clip.anims.json" --clip $cn --report "$Out\$clip.roundtrip-report.json" | Out-Null; Check "verify $clip"
  & $kpy -I "$aim\check_bike_exit.py" --clip "$Out\roundtrip\$clip.anims.glb" --vanilla $vanGlb --clip-name $cn --builder $glb @(if ($RootRetimed) { @("--root-retimed", "--max-glide-cm", "3", "--vanilla-path-tol", "$VanillaPathTol") } else { @() }) --report "$Out\check-$clip.json" | Tee-Object -FilePath "$Out\check-$clip.txt" | Select-Object -Last 1
  $results += "$clip exit=$LASTEXITCODE"
} } }
"CHECKS: " + ($results -join '; ')
Get-ChildItem $depot -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.Name }
if ($results -match 'exit=[^0]') { throw 'offline exit check failed' }
"RUN=$Out"

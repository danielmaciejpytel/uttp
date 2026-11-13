# Motorcycle ENTRY clips rebuilt by Source\tools\ai_motion\bike_mount_fix.py (bike13, 2026-11-09): imports the generated GLBs (<Cands>\<set>_enter_<l|r>.anims.glb) through the
# WolvenKit bridge route of build_bike_exit.ps1 / build_bike_enter3.ps1 and gates the ROUND-TRIPPED clips:
#   - check_bike_entry.py (a)..(k) per set (both sides; --max-step-deg 40: the vanilla NPC clips the legs / torso come from step 22.6-34.9 deg/frame, the vanilla FPP 46; the c9 gate of 15
#     was for the retargeted clips; the (m) contact gates against the shipped clip are off: they were made for gait-pass tweaks of the c9 clips, whose legs run through the bike),
#   - bike_mount_gate.py: shin / foot clearance from the real bike mesh (>= -Margin cm while the swing foot is in the air, no surface crossings, stance leg no crossings while
#     standing, hand / foot IK helpers on the bones on every frame).
# Base of every import = the SHIPPED .anims of that clip (-ShippedAnims; the motionExtraction block is copied verbatim from its serialized JSON: root motion byte-identical to the installed clip).
# A clip whose gates do not pass is NOT written to <Out>\utpp_vehicle_bike\utpp\vehicle (the caller packs the shipped clip instead).
# Output: <Out>\utpp_vehicle_bike\utpp\vehicle\<clip>.anims (passing clips only), <Out>\check-<set>.json / .txt, <Out>\gate-<clip>.json, <Out>\roundtrip\*.glb. Nothing is installed.
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [Parameter(Mandatory = $true)][string]$Cands,          # dir with <set>_enter_<l|r>.anims.glb (bike_mount_fix.py output)
  [string]$ShippedAnims = '',
  [string]$Bridge = '',
  [double]$MaxStepDeg = 40.0,
  [double]$Margin = 4.0,
  [string]$Sets = 'pwa_sportbike,pwa_sportbike2,pwa_sportbike3,pma_sportbike,pma_sportbike2,pma_sportbike3'
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $ShippedAnims) { $ShippedAnims = Join-Path $root 'Build\work\vehicle-bike-archive\bike4-20261107\enter-run\utpp_vehicle_bike\utpp\vehicle' }
if (-not $Bridge) { $Bridge = Join-Path $root 'Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$kpy = Join-Path $root 'Tools\ai-motion\envs\kimodo\Scripts\python.exe'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$glbDir = Join-Path $root 'Extracted\game\vehicle-bike\glb'
$gate = Join-Path $root 'Source\tools\ai_motion\bike_mount_gate.py'
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
$depot = Join-Path $Out 'utpp_vehicle_bike\utpp\vehicle'
New-Item -ItemType Directory -Force $depot, "$Out\base", "$Out\base-json", "$Out\import-json", "$Out\patched-json", "$Out\roundtrip", "$Out\glb" | Out-Null
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$results = @()
foreach ($set in ($Sets -split ',')) {
  $g = $set.Substring(1, 1)   # w | m
  $npc = "$glbDir\${g}a_" + $set.Substring(4) + '.anims.glb'
  $tpp = "$glbDir\tpp_$set.anims.glb"
  foreach ($c in @('l', 'r')) {
    $key = "${set}_enter_$c"; $clip = "utpp_tpp_${key}"; $cn = "enter_$c"
    $glb = Join-Path $Cands "$key.anims.glb"
    if (-not (Test-Path $glb)) { throw "missing candidate $glb" }
    Copy-Item $glb "$Out\glb\$clip.anims.glb"
    $shipped = Join-Path $ShippedAnims "$clip.anims"
    Copy-Item $shipped "$Out\base\$clip.anims"
    & $wkit convert serialize $shipped -o "$Out\base-json" | Out-Null; Check "serialize shipped $clip"
    & $Bridge import "$Out\glb\$clip.anims.glb" "$Out\base\$clip.anims" "$Out\import-$clip" | Select-String 'ANIMS_IMPORT_EXIT'; Check "import $clip"
    & $wkit convert serialize "$Out\import-$clip\$clip.anims" -o "$Out\import-json" | Out-Null; Check "serialize $clip"
    python -I "$tools\patch_motion_extraction.py" "$Out\import-json\$clip.anims.json" "$Out\glb\$clip.anims.glb" "$Out\patched-json\$clip.anims.json" --clip $cn --vanilla-json "$Out\base-json\$clip.anims.json" | Out-Null; Check "patch $clip"
    $stage = Join-Path $Out "stage-$clip"; New-Item -ItemType Directory -Force $stage | Out-Null
    & $wkit convert deserialize "$Out\patched-json\$clip.anims.json" -o $stage | Out-Null; Check "deserialize final $clip"
    & "$tools\export_glb.ps1" -Files "$stage\$clip.anims" -OutDir "$Out\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $clip"
    python -I "$tools\verify_roundtrip.py" "$Out\glb\$clip.anims.glb" "$Out\roundtrip\$clip.anims.glb" "$Out\patched-json\$clip.anims.json" --clip $cn --report "$Out\$clip.roundtrip-report.json" | Out-Null; Check "verify $clip"
    & $kpy -I $gate "$Out\roundtrip\$clip.anims.glb:$cn" --set $set --side $c --margin $Margin --report "$Out\gate-$clip.json" | Tee-Object -FilePath "$Out\gate-$clip.txt" | Select-Object -Last 1
    $results += "$clip gate=$LASTEXITCODE"
  }
  & $kpy -I "$tools\check_bike_entry.py" --src-rig $npc --enter-l "$Out\roundtrip\utpp_tpp_${set}_enter_l.anims.glb" --enter-r "$Out\roundtrip\utpp_tpp_${set}_enter_r.anims.glb" --tpp $tpp --max-step-deg $MaxStepDeg --report "$Out\check-$set.json" | Tee-Object -FilePath "$Out\check-$set.txt" | Select-Object -Last 1
  $results += "$set check=$LASTEXITCODE"
  $rep = Get-Content "$Out\check-$set.json" -Raw | ConvertFrom-Json
  foreach ($c in @('l', 'r')) {
    $clip = "utpp_tpp_${set}_enter_$c"
    $gj = Get-Content "$Out\gate-$clip.json" -Raw | ConvertFrom-Json
    if ($rep.clips."enter_$c".ok -eq $true -and $gj.ok -eq $true) { Copy-Item "$Out\stage-$clip\$clip.anims" $depot; "PASS $clip" } else { "FAIL $clip (stays shipped) check=$($rep.clips."enter_$c".ok) gate=$($gj.ok)" }
  }
}
"CHECKS: " + ($results -join '; ')
Get-ChildItem $depot -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.Name }
"RUN=$Out"

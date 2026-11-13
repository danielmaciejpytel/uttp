# Vehicle-entry pipeline, steps 2..7 (step 1 = 01_extract_export.ps1 + NPC subset, see README.txt).
# Build -> import (WolvenKit API bridge) -> root-motion patch -> round trip -> pack -> unpack check.
# Writes only under Animations\vehicle-entry\ and Build\work\vehicle-entry-archive\run-<stamp>\. Game folder: read only.
param([string]$Stamp = (Get-Date -Format 'yyyyMMdd-HHmmss'))
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
$tools = Join-Path $root 'Source\tools\anim\vehicle-entry'
$glb = Join-Path $root 'Extracted\game\vehicle-entry\glb'
$json = Join-Path $root 'Extracted\game\vehicle-entry\json'
$anim = Join-Path $root 'Animations\vehicle-entry'
$run = Join-Path $root "Build\work\vehicle-entry-archive\run-$Stamp"
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$bridge = Join-Path $root 'Build\work\vehicle-entry-archive\bridge-bin\WkitBridge.exe'
$stage = Join-Path $run 'utpp_vehicle_entry'  # pack names the archive after this folder
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot | Out-Null
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }

$npc = Join-Path $glb 'wa_sport2_passenger_r_entry.anims.glb'
$variants = @(
  @{ name = 'utpp_tpp_pwa_sport2_driver'; target = 'pwa_sport2_driver'; extra = @('--mirror') },
  @{ name = 'utpp_tpp_pwa_sport2_passenger_r'; target = 'pwa_sport2_passenger_r';
     extra = @('--cap-clip', "$glb\pwa_sport2_driver.anims.glb:enter", '--hand-follow-min', '0.8') }
)
foreach ($v in $variants) {
  $n = $v.name
  $out = Join-Path $anim "$n.anims.glb"
  python -I "$tools\build_entry_clip.py" --src $npc --target "$glb\$($v.target).anims.glb" --target-clip enter --out $out `
    --name enter --end-blend 0.4 --hand-ik --corr-start 0.25 --corr-end 0.8 --report "$anim\$n.build-report.json" @($v.extra) | Out-Null
  Check "build $n"
  python -I "$tools\animset_subset.py" "$json\$($v.target).anims.json" "$run\base-json\$n.anims.json" --keep enter | Out-Null; Check "subset $n"
  New-Item -ItemType Directory -Force "$run\base" | Out-Null
  & $wkit convert deserialize "$run\base-json\$n.anims.json" -o "$run\base" | Out-Null; Check "deserialize base $n"
  & $bridge import $out "$run\base\$n.anims" "$run\import-$n" | Select-String 'ANIMS_IMPORT_EXIT|updated' ; Check "import $n"
  New-Item -ItemType Directory -Force "$run\import-json" | Out-Null
  & $wkit convert serialize "$run\import-$n\$n.anims" -o "$run\import-json" | Out-Null; Check "serialize $n"
  python -I "$tools\patch_motion_extraction.py" "$run\import-json\$n.anims.json" $out "$run\patched-json\$n.anims.json"; Check "patch $n"
  & $wkit convert deserialize "$run\patched-json\$n.anims.json" -o $depot | Out-Null; Check "deserialize final $n"
  & "$tools\export_glb.ps1" -Files "$depot\$n.anims" -OutDir "$run\roundtrip" -RootMotion | Out-Null; Check "roundtrip export $n"
  python -I "$tools\verify_roundtrip.py" $out "$run\roundtrip\$n.anims.glb" "$run\patched-json\$n.anims.json" --report "$run\$n.roundtrip-report.json" | Out-Null
  Check "verify $n"
}
& $wkit pack $stage -o "$run\packed" ; Check 'pack'
Copy-Item (Join-Path $tools 'utpp_vehicle_entry.archive.xl') "$run\packed\"
# unpack check: extract the packed archive and compare file hashes with the staged files
& $wkit unbundle (Get-ChildItem "$run\packed\*.archive" | Select-Object -First 1).FullName -o "$run\unpacked" | Out-Null; Check 'unbundle'
Get-ChildItem $depot -File | ForEach-Object {
  $u = Join-Path "$run\unpacked\utpp\vehicle" $_.Name
  $a = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
  $b = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  '{0} staged={1} unpacked={2} equal={3}' -f $_.Name, $a, $b, ($a -eq $b)
}
Get-ChildItem "$run\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"RUN=$run"

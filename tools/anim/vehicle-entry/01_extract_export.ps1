# Step 1 of the vehicle-entry pipeline: extract (read-only) the sport2 car clip sets + rigs from the game archives and
# export them to glTF binary, optionally WITH root motion (AnimationExportArgs.incRootMotion bound from the
# environment; the WolvenKit.CLI GenericHost reads options from configuration/env).
# Writes only under Extracted\game\vehicle-entry\. The game folder is only read.
param(
  [string]$Game = '',
  [switch]$SkipExtract,
  [string[]]$Sets = @('wa_sport2_passenger_r', 'pwa_sport2_passenger_r', 'pwa_sport2_driver', 'wa_sport2_driver'),
  [string]$OutSub = 'glb',
  [switch]$NoRootMotion,
  [string]$Verbosity = 'Normal'
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Game) { $Game = Get-UtppGameDir }
$ErrorActionPreference = 'Stop'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$raw = Join-Path $root 'Extracted\game\vehicle-entry\raw'
$glb = Join-Path $root ('Extracted\game\vehicle-entry\' + $OutSub)
foreach ($d in @($raw, $glb)) { if (!(Test-Path $d)) { New-Item -ItemType Directory -Path $d | Out-Null } }
$content = Join-Path $Game 'archive\pc\content'
if (!$SkipExtract) {
  & $wkit unbundle (Join-Path $content 'basegame_1_engine.archive') -o $raw -r '(vehicle\\sport\\character\\pwa_sport2_[^\\]*\.anims$)|(player_woman_skeleton\.rig$)|(woman_base\\woman_base\.rig$)'
  if ($LASTEXITCODE -ne 0) { throw "unbundle engine failed $LASTEXITCODE" }
  & $wkit unbundle (Join-Path $content 'basegame_4_animation.archive') -o $raw -r '(vehicle\\sport\\character\\wa_sport2_[^\\]*\.anims$)|(vehicle\\sport\\vehicle\\v_sport2_[^\\]*\.anims$)'
  if ($LASTEXITCODE -ne 0) { throw "unbundle animation failed $LASTEXITCODE" }
}
$chr = Join-Path $raw 'base\animations\vehicle\sport\character'
if (!$NoRootMotion) { $env:AnimationExportArgs__incRootMotion = 'true' } else { $env:AnimationExportArgs__incRootMotion = 'false' }
$inputs = @($Sets | ForEach-Object { Join-Path $chr ($_ + '.anims') })
& $wkit export @inputs -o $glb -gp $Game -v $Verbosity
$rc = $LASTEXITCODE
Get-ChildItem $glb -Recurse -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
exit $rc

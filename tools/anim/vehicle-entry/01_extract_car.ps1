# Step 0/1 of the car entry pipeline: extract (read-only) the standard2 / sport2 player + NPC clip sets, the shared vehicle workspot,
# the rigs and the Archer Hella entities from the installed game archives (content + ep1; archive\pc\mod is NOT read, so the
# data are vanilla). Writes only under Extracted\game\vehicle-entry-car\ (raw, json, glb). The game folder is only read.
# Record of what was extracted: Extracted\game\vehicle-entry-car\provenance.txt (sha256 of every raw file).
param(
  [string]$Game = '',
  # clip set class names (file prefix p?a_/?a_<class>_); the player utility sets are 'utility' (workspot var utility4)
  [string[]]$Classes = @('standard2', 'sport2'),
  [switch]$SkipUnbundle,
  [switch]$SkipExport,
  # door clearance data (check_door_clearance.py): vehicle clip sets, door entities + meshes of the six test vehicles -> raw-veh, json-veh, glb-veh,
  # glb-doors, door-data\<class>.json, door-provenance.txt (extract_door_assets.py). Independent of the player / NPC clip extraction above.
  [switch]$Doors,
  [string]$Verbosity = 'Normal'
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Game) { $Game = Get-UtppGameDir }
$ErrorActionPreference = 'Stop'
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\vehicle-entry-car'
$raw = Join-Path $out 'raw'
$json = Join-Path $out 'json'
$glb = Join-Path $out 'glb'
foreach ($d in @($raw, $json, $glb)) { New-Item -ItemType Directory -Force $d | Out-Null }
$arch = @('content\basegame_1_engine', 'content\basegame_3_nightcity', 'content\basegame_4_animation', 'content\basegame_4_appearance',
          'content\basegame_4_gamedata', 'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata')
$cls = ($Classes | ForEach-Object { [regex]::Escape($_) }) -join '|'
$re = '(animations\\vehicle\\(standard|sport|utility)\\character\\(pwa|pma|wa|ma)_(' + $cls + ')_[^\\]*\.anims$)' +
      '|(player_woman_skeleton\.rig$)|(player_man_skeleton\.rig$)|(woman_base\\woman_base\.rig$)|(man_base\\man_base\.rig$)' +
      '|(vehicles\\common\\gameplay\\vehicle_workspot\.workspot$)' +
      '|(vehicles\\standard\\v_standard2_archer_hella\\v_standard2_archer_hella[^\\]*\.ent$)' +
      '|(characters\\entities\\player\\player_(wa|ma)_fpp(_ep1)?\.ent$)'
if (!$SkipUnbundle) {
  foreach ($a in $arch) {
    $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
    if (!(Test-Path $p)) { "skip (missing) $p"; continue }
    "unbundle $a"
    & $wkit unbundle $p -o $raw -r $re
    if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
  }
}
$files = Get-ChildItem $raw -Recurse -File
$files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
  Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
"raw files: $($files.Count)"
# serialize the json bases (workspot, ent, anims sets)
foreach ($f in ($files | Where-Object { $_.Extension -in '.anims', '.workspot', '.ent', '.rig' })) {
  & $wkit convert serialize $f.FullName -o $json | Out-Null
  if ($LASTEXITCODE -ne 0) { throw "serialize $($f.Name) failed" }
}
if (!$SkipExport) {
  $env:AnimationExportArgs__incRootMotion = 'true'
  $chr = Get-ChildItem $raw -Recurse -Filter '*.anims' | Where-Object { $_.Name -match ('^p?(w|m)a_(' + $cls + ')_(driver|passenger_r)\.anims$') }
  foreach ($c in $chr) {
    "export $($c.Name)"
    & $wkit export $c.FullName -o $glb -gp $Game -v $Verbosity
    if ($LASTEXITCODE -ne 0) { "EXPORT FAILED $($c.Name) $LASTEXITCODE" }
  }
}
if ($Doors) {
  python -I (Join-Path $PSScriptRoot 'extract_door_assets.py') --wkit $wkit --game $Game --root $out
  if ($LASTEXITCODE -ne 0) { throw "door asset extraction failed ($LASTEXITCODE)" }
}
"DONE"

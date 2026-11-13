param(
  [string]$Game = '',
  [switch]$SkipUnbundle
)
# Extract the vanilla NPC third-person melee animsets (woman_average / man_average: unarmed, knife, katana; action + locomotion
# combat) and the NPC base rigs from the installed game archives (content + ep1; archive\pc\mod is NOT read, so the data are
# vanilla), then serialize them to json (events, durations) and export them to .anims.glb (root motion included).
# Writes only under Extracted\game\npc-melee\ (raw, json, glb, logs, provenance.txt). The game folder is only read.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\npc-melee'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $glb = Join-Path $out 'glb'; $logs = Join-Path $out 'logs'
foreach ($d in @($raw, $json, $glb, $logs)) { New-Item -ItemType Directory -Force $d | Out-Null }
$arch = @('content\basegame_4_animation', 'content\basegame_1_engine', 'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata')
$re = '(animations\\npc\\gameplay\\(woman_average|man_average)\\(gang\\(unarmed|knife)|corpo\\katana)\\(wa|ma)_[^\\]*_(action|locomotion)_combat(_blocking)?\.anims$)' +
      '|(woman_base\\woman_base\.rig$)|(man_base\\man_base\.rig$)'
if (!$SkipUnbundle) {
  foreach ($a in $arch) {
    $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
    if (!(Test-Path $p)) { "skip (missing) $p"; continue }
    "unbundle $a"
    & $wkit unbundle $p -o $raw -r $re | Out-File (Join-Path $logs ('unbundle_' + ($a -replace '\\', '_') + '.log')) -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
  }
}
$files = Get-ChildItem $raw -Recurse -File
$files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
  Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
"raw files: $($files.Count)"
$env:AnimationExportArgs__incRootMotion = 'true'
$anims = $files | Where-Object { $_.Extension -eq '.anims' }
foreach ($f in $anims) {
  & $wkit convert serialize $f.FullName -o $json | Out-File (Join-Path $logs ('json_' + $f.Name + '.log')) -Encoding utf8
}
$jobs = foreach ($f in $anims) {
  $a = @('export', ('"' + $f.FullName + '"'), '-o', ('"' + $glb + '"'), '-gp', ('"' + $Game + '"'))
  Start-Process -FilePath $wkit -ArgumentList $a -RedirectStandardOutput (Join-Path $logs ('export_' + $f.Name + '.log')) `
    -RedirectStandardError (Join-Path $logs ('export_' + $f.Name + '.err')) -PassThru -WindowStyle Hidden
}
$jobs | Wait-Process
Get-ChildItem $glb, $json -File | Select-Object Name, Length | Format-Table -AutoSize | Out-String -Width 200

# Read-only retail extraction for the melee export (2026-11-09): unbundles from the installed game archives every animset the four 2.5 player
# entities register (list = player_entity_sets_unique.json, made by registration_evidence.py) + the four player entities,
# serializes them to JSON and writes <Out>\retail-provenance.txt (sha256, size, depot path). Nothing in the game folder is written.
#   powershell -File extract_retail.ps1 -Out <dir> -SetsJson <player_entity_sets_unique.json> [-Archives a,b] [-SkipUnbundle]
# (The ep1 sets / entities are in ep1\ep1_2_gamedata.)
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [Parameter(Mandatory = $true)][string]$SetsJson,
  [string]$Game = '',
  [string[]]$Archives = @('content\basegame_4_animation', 'content\basegame_1_engine', 'content\basegame_4_gamedata', 'content\basegame_3_nightcity', 'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata'),
  [switch]$SkipUnbundle
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Game) { $Game = Get-UtppGameDir }
$ErrorActionPreference = 'Stop'
$full = [IO.Path]::GetFullPath($Out)
if (!$full.StartsWith($root + '\')) { throw 'Out outside the project' }
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$raw = Join-Path $full 'raw'; $json = Join-Path $full 'json'
New-Item -ItemType Directory -Force $raw, $json | Out-Null
$doc = Get-Content $SetsJson -Raw | ConvertFrom-Json
$paths = @{}
foreach ($p in $doc.PSObject.Properties) { foreach ($r in $p.Value) { if ($r.set) { $paths[$r.set.ToLower()] = 1 } } }
$alts = @($paths.Keys | Sort-Object | ForEach-Object { [regex]::Escape($_) + '$' })
$alts += 'characters\\entities\\player\\player_(wa|ma)_fpp(_ep1)?\.ent$'
$re = '(' + ($alts -join ')|(') + ')'
"sets registered: $($paths.Count); regex length $($re.Length)"
if (!$SkipUnbundle) {
  foreach ($a in $Archives) {
    $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
    if (!(Test-Path $p)) { "skip (missing) $p"; continue }
    "unbundle $a"
    & $wkit unbundle $p -o $raw -r $re | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
  }
}
$files = Get-ChildItem $raw -Recurse -File
$files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
  Set-Content (Join-Path $full 'retail-provenance.txt') -Encoding ascii
"raw files: $($files.Count)"
$missing = @($paths.Keys | Where-Object { !(Test-Path (Join-Path $raw $_)) })
"missing registered sets: $($missing.Count)"; $missing | ForEach-Object { "  MISSING $_" }
& $wkit convert serialize $raw -o $json | Out-Null
"serialize exit $LASTEXITCODE"
"json files: $((Get-ChildItem $json -Recurse -File).Count)"

# Read-only extraction of the player body part sources from the installed game 2.50 (content + ep1 archives; archive\pc\mod is NOT read).
# Step 1 of recreating the 2.50 data for gen_bodyparts.py (step 2: serialize_mirrored.ps1, step 3: python gen_bodyparts.py --check).
# Output: Extracted\game\body-parts-250\raw (depot layout) + json (mirrored), provenance.txt (SHA-256).
param([string]$Game = '')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path  # Source\tools\bodyparts -> project root
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\body-parts-250'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $logs = Join-Path $out 'logs'
foreach ($d in @($raw, $json, $logs)) { New-Item -ItemType Directory -Force $d | Out-Null }
$re = 'player_base_bodies.appearances.*\.app$|player_(wa|ma)_fpp(_ep1)?\.ent$|player_(wa|ma)_tpp_head\.ent$'
$arch = @('content\basegame_1_engine', 'content\basegame_3_nightcity', 'content\basegame_4_appearance', 'content\basegame_4_gamedata',
          'content\basegame_4_animation', 'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata')
foreach ($a in $arch) {
  $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
  if (!(Test-Path $p)) { "skip (missing) $p"; continue }
  "unbundle $a"
  & $wkit unbundle $p -o $raw -r $re | Out-File (Join-Path $logs ('unbundle_' + ($a -replace '\\', '_') + '.log')) -Encoding utf8
  if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
}
$files = Get-ChildItem $raw -Recurse -File
$files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
  Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
"raw files: $($files.Count)"
& $wkit convert serialize $raw -o $json | Out-File (Join-Path $logs 'serialize.log') -Encoding utf8
"serialize exit $LASTEXITCODE"
"json files: $((Get-ChildItem $json -Recurse -File -Filter *.json).Count)"

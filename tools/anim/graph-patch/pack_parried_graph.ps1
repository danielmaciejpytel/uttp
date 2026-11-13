# Build utpp_melee_parried_graph.archive: base\gameplay\anim_graphs\player_melee.animgraph (Cyberpunk 2077 2.5) with the
# per-attack Parried selection (patch_parried_combo.py). Offline; the game folder is only read (via the -RawGraph copy).
# Every step is checked: pinned vanilla SHA-256, unmodified JSON round trip = original bytes, patched graph inspect / diff /
# simulate (parried_graph_check.py), patched round trip (JSON identical, second deserialize byte-identical), pack ->
# unbundle byte check and archive listing. Output: <OutRoot>\<Run>\packed\utpp_melee_parried_graph.archive. Never installs.
# -CompareWith <older patched JSON> (optional, e.g. run3\work\patched\player_melee.animgraph.json): regression proof that the
# older states / entries are unchanged (parried_graph_check.py compare), written to compare.txt and listed in the manifest.
#
# RULE: after a game patch the vanilla graph changes -> SHA mismatch -> the build stops and an installed archive is STALE
# (it would replace the new game graph with the old one). Re-extract, review the Parried state, update $VanillaSha deliberately.
param(
  [string]$RawGraph = '',
  [string]$OutRoot = '',
  [string]$Run = ('run-' + (Get-Date -Format 'yyyyMMdd-HHmmss')),
  [string]$Python = '',
  [string]$CompareWith = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $RawGraph) { $RawGraph = Join-Path $root 'Build\work\melee-parried-graph-20261110\extract\content_basegame_1_engine\base\gameplay\anim_graphs\player_melee.animgraph' }
if (-not $OutRoot) { $OutRoot = Join-Path $root 'Build\work\melee-parried-graph-20261110' }
if (-not $Python) { $Python = Join-Path $root 'Tools\ai-motion\envs\kimodo\Scripts\python.exe' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$jsonSame = Join-Path $root 'Source\tools\anim\ragdoll\json_same.py'
# player_melee.animgraph of game 2.5 (content\basegame_1_engine.archive, the only copy in content + ep1; extracted 2026-11-10).
$VanillaSha = 'BFB4D7651E761B08FCE0FEA08767AF8F9A9D9321820F4A4DC13E412A7D445AB0'
$name = 'player_melee.animgraph'
$rel = 'base\gameplay\anim_graphs'
$archiveName = 'utpp_melee_parried_graph'

function Sha($p) { (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash }
function OutDir($p) { New-Item -ItemType Directory -Force $p | Out-Null; $p }   # WolvenKit refuses a missing -o directory
function Run-Checked([string]$what, [string]$exe, [string[]]$a) {
  # PowerShell 5.1 turns native stderr into ErrorRecords; judge only by the exit code, keep the output in the log.
  $ErrorActionPreference = 'Continue'
  $o = & $exe @a 2>&1
  $code = $LASTEXITCODE
  $ErrorActionPreference = 'Stop'
  $o | ForEach-Object { "$_" } | Add-Content -LiteralPath $script:log -Encoding utf8
  if ($code -ne 0) { throw "$what failed ($code)" }
  return $o
}

$out = Join-Path $OutRoot $Run
if (Test-Path $out) { throw "output exists, choose a new -Run: $out" }
New-Item -ItemType Directory -Force $out | Out-Null
$script:log = Join-Path $out 'build.log'
if (-not (Test-Path -LiteralPath $RawGraph)) { throw "raw graph missing: $RawGraph" }
if ((Sha $RawGraph) -ne $VanillaSha) { throw "GAME CHANGED: $RawGraph SHA256 $(Sha $RawGraph) != pinned 2.5 $VanillaSha" }

# 1. unmodified JSON round trip must give the original bytes (otherwise the JSON route loses data)
$w = Join-Path $out 'work'
Run-Checked 'serialize vanilla' $wkit @('convert', 'serialize', $RawGraph, '-o', (OutDir "$w\json")) | Out-Null
$vjson = Join-Path $w "json\$name.json"
Run-Checked 'deserialize vanilla' $wkit @('convert', 'deserialize', $vjson, '-o', (OutDir "$w\rt-orig")) | Out-Null
if ((Sha (Join-Path $w "rt-orig\$name")) -ne $VanillaSha) { throw 'unmodified JSON round trip is not byte-identical' }
"control round trip: byte-identical ($VanillaSha)" | Tee-Object -FilePath $script:log -Append

# 2. patch + offline checks on the patched JSON
$pjson = Join-Path (OutDir "$w\patched") "$name.json"
Run-Checked 'patch' $Python @('-I', (Join-Path $tools 'patch_parried_combo.py'), $vjson, $pjson, '--report', (Join-Path $out 'patch_report.json')) | Out-File (Join-Path $out 'patch.txt') -Encoding utf8
Run-Checked 'inspect' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'inspect', $pjson) | Out-File (Join-Path $out 'inspect.txt') -Encoding utf8
Run-Checked 'diff' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'diff', $vjson, $pjson) | Out-File (Join-Path $out 'diff.txt') -Encoding utf8
Run-Checked 'simulate' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'simulate', $pjson) | Out-File (Join-Path $out 'simulate.txt') -Encoding utf8
$checks = @('control round trip byte-identical', 'inspect PASS', 'diff PASS', 'simulate PASS')
if ($CompareWith) {
  if (-not (Test-Path -LiteralPath $CompareWith)) { throw "compare reference missing: $CompareWith" }
  Run-Checked 'compare' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'compare', $CompareWith, $pjson) | Out-File (Join-Path $out 'compare.txt') -Encoding utf8
  $checks += "compare PASS vs $CompareWith"
}

# 3. CR2W + round trips of the patched graph
$stage = OutDir (Join-Path $out "$archiveName\$rel")
Run-Checked 'deserialize patched' $wkit @('convert', 'deserialize', $pjson, '-o', $stage) | Out-Null
$staged = Join-Path $stage $name
Run-Checked 're-serialize patched' $wkit @('convert', 'serialize', $staged, '-o', (OutDir "$w\rt-patched")) | Out-Null
Run-Checked 'patched JSON round trip' $Python @('-I', $jsonSame, $pjson, (Join-Path $w "rt-patched\$name.json")) | Out-Null
Run-Checked 'second deserialize' $wkit @('convert', 'deserialize', (Join-Path $w "rt-patched\$name.json"), '-o', (OutDir "$w\rt-patched-bin")) | Out-Null
if ((Sha $staged) -ne (Sha (Join-Path $w "rt-patched-bin\$name"))) { throw 'patched graph is not byte-stable over a second round trip' }
# the re-serialized patched graph must still pass the checks (proves the binary, not only the patcher JSON)
Run-Checked 'inspect re-serialized' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'inspect', (Join-Path $w "rt-patched\$name.json")) | Out-File (Join-Path $out 'inspect_reserialized.txt') -Encoding utf8
Run-Checked 'simulate re-serialized' $Python @('-I', (Join-Path $tools 'parried_graph_check.py'), 'simulate', (Join-Path $w "rt-patched\$name.json")) | Out-File (Join-Path $out 'simulate_reserialized.txt') -Encoding utf8

# 4. pack -> unbundle -> byte check, listing
Run-Checked 'pack' $wkit @('pack', (Join-Path $out $archiveName), '-o', (OutDir "$out\packed")) | Out-Null
$arch = Join-Path $out "packed\$archiveName.archive"
Run-Checked 'unbundle' $wkit @('unbundle', $arch, '-o', (OutDir "$out\unpacked")) | Out-Null
$files = @(Get-ChildItem "$out\unpacked" -Recurse -File)
if ($files.Count -ne 1) { throw "unbundle file count $($files.Count) != 1" }
if ((Sha (Join-Path $out "unpacked\$rel\$name")) -ne (Sha $staged)) { throw 'unbundled graph differs from the staged graph' }
$listing = @(Run-Checked 'list' $wkit @('archive', $arch, '-l') | ForEach-Object { "$_" } | Where-Object { $_ -match '^base\\' })
if (($listing -join '|') -ne "$rel\$name") { throw "archive listing differs: $($listing -join ', ')" }

$manifest = [ordered]@{
  created = (Get-Date -Format s); vanillaGraph = $RawGraph; vanillaSha256 = $VanillaSha
  stagedGraphSha256 = (Sha $staged); stagedGraphBytes = (Get-Item $staged).Length
  archive = $arch; archiveSha256 = (Sha $arch); archiveBytes = (Get-Item $arch).Length; archiveListing = $listing
  checks = $checks + @('patched JSON round trip identical', 'second deserialize byte-identical', 're-serialized inspect/simulate PASS',
                       'unbundle byte-identical, 1 file')
  installed = $false; runtimeVerified = $false
}
$manifest | ConvertTo-Json -Depth 4 | Out-File (Join-Path $out 'manifest.json') -Encoding utf8
'{0}  {1}' -f (Sha $arch), $arch
"PACK_OK OUT=$out"

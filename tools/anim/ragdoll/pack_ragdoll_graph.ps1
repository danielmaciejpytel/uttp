# Build utpp_ragdoll_graph.archive: player_base.animgraph with the RagdollControl node (insert_ragdoll_control.py, R1j) and
# player_locomotion.animgraph with the ANY -> tpp_securitycam_hacking transition guarded by a WrapperValue condition on
# utpp_foot_tpp (must not be set; disable_securitycam_entry.py keeps the vanilla feature name and isEnabled).
# Inputs are the 2.5 game graphs unbundled into -RawRoot (read only). Every step is checked: unmodified round trip = original
# bytes, the patched files re-serialize to the expected JSON, and the packed archive unbundles to the staged bytes.
# Output: <OutRoot>\<Run>\packed\utpp_ragdoll_graph.archive. Nothing is installed.
param(
  [string]$RawRoot = '',
  [string]$OutRoot = '',
  [string]$Run = ('run-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $RawRoot) { $RawRoot = Join-Path $root 'Build\work\diag-20261103\crouch\raw25' }
if (-not $OutRoot) { $OutRoot = Join-Path $root 'Build\work\ragdoll-graph' }
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
function OutDir($p) { New-Item -ItemType Directory -Force $p | Out-Null; $p }   # WolvenKit refuses a missing -o directory
function Sha($p) { (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash }
$out = Join-Path $OutRoot $Run
if (Test-Path $out) { throw "output exists, choose a new -Run: $out" }
$rel = 'base\gameplay\anim_graphs'
$stage = Join-Path $out "utpp_ragdoll_graph\$rel"
New-Item -ItemType Directory -Force $stage | Out-Null
$patches = @(
  @{ name = 'player_base.animgraph'; tool = 'insert_ragdoll_control.py' },
  @{ name = 'player_locomotion.animgraph'; tool = 'disable_securitycam_entry.py' }
)
foreach ($p in $patches) {
  $raw = Join-Path $RawRoot "$rel\$($p.name)"
  if (-not (Test-Path $raw)) { throw "raw graph missing: $raw" }
  $w = Join-Path $out ('work\' + $p.name)
  & $wkit convert serialize $raw -o (OutDir "$w\json") | Out-Null; Check "serialize $($p.name)"
  $json = Join-Path $w "json\$($p.name).json"
  & $wkit convert deserialize $json -o (OutDir "$w\rt-orig") | Out-Null; Check "control deserialize $($p.name)"
  if ((Sha (Join-Path $w "rt-orig\$($p.name)")) -ne (Sha $raw)) {
    # player_base does not re-write byte-identically (R1j); then the re-written original must serialize to the same JSON
    & $wkit convert serialize (Join-Path $w "rt-orig\$($p.name)") -o (OutDir "$w\rt-orig-json") | Out-Null; Check "control re-serialize $($p.name)"
    python -I (Join-Path $tools 'json_same.py') $json (Join-Path $w "rt-orig-json\$($p.name).json")
    Check "unmodified round trip of $($p.name) (bytes differ, JSON must match)"
    "$($p.name): unmodified round trip differs in bytes, JSON identical"
  }
  $pj = Join-Path $w "patched\$($p.name).json"
  New-Item -ItemType Directory -Force (Split-Path $pj) | Out-Null
  python -I (Join-Path $tools $p.tool) $json $pj; Check "patch $($p.name)"
  & $wkit convert deserialize $pj -o $stage | Out-Null; Check "deserialize $($p.name)"
  & $wkit convert serialize (Join-Path $stage $p.name) -o (OutDir "$w\rt-patched") | Out-Null; Check "re-serialize $($p.name)"
  python -I (Join-Path $tools 'json_same.py') $pj (Join-Path $w "rt-patched\$($p.name).json")
  Check "patched round trip $($p.name)"
  '{0}  raw={1}  staged={2}' -f $p.name, (Sha $raw), (Sha (Join-Path $stage $p.name))
}
& $wkit pack (Join-Path $out 'utpp_ragdoll_graph') -o (OutDir "$out\packed") | Out-Null; Check 'pack'
$arch = Join-Path $out 'packed\utpp_ragdoll_graph.archive'
& $wkit unbundle $arch -o (OutDir "$out\unpacked") | Out-Null; Check 'unbundle'
foreach ($p in $patches) {
  if ((Sha (Join-Path $stage $p.name)) -ne (Sha (Join-Path $out "unpacked\$rel\$($p.name)"))) { throw "unbundled $($p.name) differs" }
}
if (@(Get-ChildItem "$out\unpacked" -Recurse -File).Count -ne $patches.Count) { throw 'unbundle file count mismatch' }
'{0}  {1}' -f (Sha $arch), $arch
"PACK_OK OUT=$out"

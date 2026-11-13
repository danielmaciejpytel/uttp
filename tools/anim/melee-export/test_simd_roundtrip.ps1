# Regression test of the SIMD sign fix (2026-11-10): a real round trip through the WolvenKit importer of a clip with negative-w keys.
#   powershell -ExecutionPolicy Bypass -File test_simd_roundtrip.ps1 -Out <new dir under Build\work> [-Fixture <pma_knife__throw_start.npc1.npz>]
# Fixture: the re-solved Male knife clip pma_knife / throw_start (animAnimationBufferSimd, uncompressed; LeftUpLeg and arm_left_ik keys with w < 0).
# Before the fix the importer returned some of those keys with another rotation (LeftUpLeg 18 deg, arm_left_ik 38 deg, foot helper 24.6 cm).
#   A  run_melee_export.ps1 -Only throw_start (canonical w >= 0, the default): must pass the verifier (exit 0, 1 of 1 clip, local error < 0.15 deg).
#   B  the same with -NoSimdCanon (the old sign handling, negative control): must FAIL the verifier with a local rotation error > 1 deg.
#      A pass of A alone would not prove anything; B proves that this fixture still triggers the importer problem.
# Needs WolvenKit.CLI and WkitBridge (see run_melee_export.ps1); nothing outside <Out> is written.
param(
  [Parameter(Mandatory = $true)][string]$Out,
  [string]$Fixture = '',
  [string]$FixtureSha256 = 'E727A9A34BBD1CDE5068F1EF2BB9251934FBA419189D540789660C9CBE7C6F19'
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Fixture) { $Fixture = Join-Path $root 'Build\test-data\melee-export\pma_knife__throw_start.npc1.npz' }
$ErrorActionPreference = 'Stop'
$run = Join-Path $PSScriptRoot 'run_melee_export.ps1'
$Out = [IO.Path]::GetFullPath($Out)
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
if ((Get-FileHash $Fixture -Algorithm SHA256).Hash -ne $FixtureSha256) { throw "fixture sha256 differs from the known failing input: $Fixture" }
$in = Join-Path $Out 'fixture'
New-Item -ItemType Directory -Force $in | Out-Null
Copy-Item $Fixture $in
$info = [IO.Path]::ChangeExtension($Fixture, $null).TrimEnd('.') + '.info.json'
if (Test-Path $info) { Copy-Item $info $in }
$fail = 0
function Run([string]$name, [switch]$noCanon) {
  $a = @{ InputDir = $in; Rig = 'man'; Sets = 'pma_knife'; Out = (Join-Path $Out $name); Suffix = 'npc1'; Only = 'throw_start'; NoPack = $true }
  if ($noCanon) { $a.NoSimdCanon = $true }
  & $run @a *> (Join-Path $Out "$name.log")
  return $LASTEXITCODE
}
function MaxLocalDeg([string]$json) {
  if (!(Test-Path $json)) { return $null }
  $r = Get-Content $json -Raw | ConvertFrom-Json
  return ($r.clips.PSObject.Properties | ForEach-Object { $_.Value.local_r_deg.max } | Measure-Object -Maximum).Maximum
}
# A: canonical signs
$ea = Run 'A_canonical'
$da = MaxLocalDeg (Join-Path $Out 'A_canonical\reports\verify-pma_knife.json')
$okA = ($ea -eq 0) -and ($null -ne $da) -and ($da -lt 0.15)
"A (w >= 0): run exit $ea, verifier max local rotation error $da deg -> $(if ($okA) { 'PASS' } else { 'FAIL' })"
if (!$okA) { $fail++ }
# B: negative control
$eb = Run 'B_old_signs' -noCanon
$db = MaxLocalDeg (Join-Path $Out 'B_old_signs\attempt1\reports__verify-pma_knife.json')
if ($null -eq $db) { $db = MaxLocalDeg (Join-Path $Out 'B_old_signs\reports\verify-pma_knife.json') }
$okB = ($eb -ne 0) -and ($null -ne $db) -and ($db -gt 1.0)
"B (old signs, -NoSimdCanon): run exit $eb, verifier max local rotation error $db deg -> $(if ($okB) { 'PASS (fails as expected)' } else { 'FAIL (the fixture no longer shows the importer problem, or the verifier is blind)' })"
if (!$okB) { $fail++ }
"SIMD_ROUNDTRIP_TEST $(if ($fail) { 'FAIL' } else { 'OK' })"
exit $(if ($fail) { 1 } else { 0 })

# Pack-only build of utpp_vehicle_bike.archive with the motorcycle workspot override (diag 2026-10-28 c8b).
# The bike seat enter clips come from the OccupantSlots workspot base\vehicles\common\gameplay\vehicle_motorcycle_workspot.workspot
# (its own per-rig animset list), not from the player entity sets. make_bike_workspot.py builds a full override of that file with
# the 12 UTPP enter sets added to the two player rigs (gated by utpp_vehicle_tpp; priority 127 and 230) and checks that nothing
# else changed. This script reuses the 12 UTPP enter clips of an existing build_bike.ps1 run unchanged, keeps the extension
# component patch .ent files (make_bike_ext_patch.py) and the .xl (both inert with the reds default route 0: every entry there needs
# a utpp_bike_route_* variable), validates the .xl (check_bike_xl.py incl. selftest), packs, unbundles and compares the hashes of
# every staged file. Nothing is installed; the game folder is not touched.
# -Hash0: animations.hash = 0 on the two player rigs of the workspot (variant for a second test).
# -Release: release file set. Stages only base\vehicles\common\gameplay\vehicle_motorcycle_workspot.workspot and the 12
#   utpp\vehicle\utpp_tpp_p?a_sportbike*_enter_{l,r}.anims: no extension patch .ent files, no route .xl (the workspot override is a
#   plain path override and needs no ArchiveXL). Same workspot build + selftest and the same unbundle hash compare; additionally the
#   unpacked set must be exactly 13 files, and the workspot must equal the one of -TestedBuild (the build the in-game tests ran).
#   Defaults with -Release: -AnimsRun = the c9c build (its 12 clips are the validated c9c clips), -Out = release-<stamp> beside it.
#   Output with -Release: <Out>\packed\utpp_vehicle_bike.archive only.
# -VanillaBin: vanilla vehicle_motorcycle_workspot.workspot (sha256 is checked by make_bike_workspot.py); its JSON is serialized with
#   WolvenKit into <Out>\vanilla-json. Use it when the diag copy of the vanilla JSON/binary (make_bike_workspot.py DIAG) is gone;
#   with -Release it defaults to Build\work\vehicle-bike-archive\vanilla-ws-raw (unbundle the file from the game archives into it).
# Output: <Out>\packed\utpp_vehicle_bike.archive + utpp_vehicle_bike.archive.xl
param(
  [string]$AnimsRun = '',
  [string]$Out = '',
  [switch]$Hash0,
  [switch]$Release,
  [string]$TestedBuild = '',
  [string]$VanillaBin = '',
  [string]$ExitRun = ''   # opt-in (2026-11-06): run dir of build_bike_exit.ps1; its 12 utpp_tpp_p?a_sportbike*_exit_?.anims are packed and registered (make_bike_workspot.py --exit-sets); default off; with -Release the release set (no .ent patches / .xl) + the registered exit clips (workspot differs from -TestedBuild by design)
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $AnimsRun) { $AnimsRun = Join-Path $root 'Build\work\vehicle-bike-archive\run-20261028-074600' }
if (-not $Out) { $Out = Join-Path $root 'Build\work\vehicle-bike-archive\c8b-ws' }
if (-not $TestedBuild) { $TestedBuild = Join-Path $root 'Build\work\vehicle-bike-archive\c9c-ws' }
$ErrorActionPreference = 'Stop'
if ($Release) {
  if (-not $PSBoundParameters.ContainsKey('AnimsRun')) { $AnimsRun = $TestedBuild }
  if (-not $PSBoundParameters.ContainsKey('Out')) { $Out = Join-Path (Split-Path -Parent $TestedBuild) ('release-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
  if (-not $VanillaBin) {
    $vb = Join-Path (Split-Path -Parent $TestedBuild) 'vanilla-ws-raw\base\vehicles\common\gameplay\vehicle_motorcycle_workspot.workspot'
    if (Test-Path $vb) { $VanillaBin = $vb }
  }
}
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$srcDepot = Join-Path $AnimsRun 'utpp_vehicle_bike\utpp\vehicle'
$stage = Join-Path $Out 'utpp_vehicle_bike'
$depot = Join-Path $stage 'utpp\vehicle'
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
New-Item -ItemType Directory -Force $depot | Out-Null
$anims = Get-ChildItem $srcDepot -Filter '*.anims' -File
if ($anims.Count -ne 12) { throw "expected 12 .anims in $srcDepot, found $($anims.Count)" }
$anims | ForEach-Object { Copy-Item $_.FullName $depot }
$exitSets = ''
if ($ExitRun) {
  # only exit clips whose offline gate report (check-<clip>.json of build_bike_exit.ps1) says ok are packed: a clip that failed (or has no report) is never shipped as an orphan
  $exits = @(Get-ChildItem (Join-Path $ExitRun 'utpp_vehicle_bike\utpp\vehicle') -Filter 'utpp_tpp_p?a_sportbike*_exit_?.anims' -File | Where-Object {
    $cj = Join-Path $ExitRun ('check-' + $_.BaseName + '.json')
    (Test-Path $cj) -and ((Get-Content $cj -Raw | ConvertFrom-Json).ok -eq $true) })
  # per (bike, side): present (and passing) for BOTH player rigs -> registered on both ('1r'); for one rig only -> registered on that rig only ('w1r' = pwa Female V, 'm1r' = pma Male V; make_bike_workspot.py --exit-sets);
  # the other rig keeps the vanilla exit of that bike / side
  $tok = @()
  foreach ($b in @('', '2', '3')) { foreach ($c in @('l', 'r')) {
    $hw = @($exits | Where-Object { $_.Name -eq "utpp_tpp_pwa_sportbike${b}_exit_$c.anims" }).Count -gt 0
    $hm = @($exits | Where-Object { $_.Name -eq "utpp_tpp_pma_sportbike${b}_exit_$c.anims" }).Count -gt 0
    $n = $(if ($b -eq '') { '1' } else { $b })
    if ($hw -and $hm) { $tok += "$n$c" } elseif ($hw) { $tok += "w$n$c" } elseif ($hm) { $tok += "m$n$c" }
  } }
  if ($tok.Count -eq 0) { throw "no passing exit clip in $ExitRun" }
  $exitSets = $tok -join ','
  $exits | ForEach-Object { Copy-Item $_.FullName $depot }
  "EXIT SETS registered: $exitSets"
}
if (-not $Release) { python -I "$tools\make_bike_ext_patch.py" --wkit $stage --wkit-exe $wkit; Check 'ext patch' }
$wsArgs = @("$tools\make_bike_workspot.py", '--stage', $stage, '--anims-dir', $depot, '--wkit-exe', $wkit, '--selftest')
if ($VanillaBin) {
  $vj = Join-Path $Out 'vanilla-json'
  New-Item -ItemType Directory -Force $vj | Out-Null
  & $wkit convert serialize $VanillaBin -o $vj | Out-Null; Check 'vanilla serialize'
  $wsArgs += @('--vanilla-bin', $VanillaBin, '--vanilla-json', (Join-Path $vj 'vehicle_motorcycle_workspot.workspot.json'))
}
if ($Hash0) { $wsArgs += '--hash0' }
if ($ExitRun) { $wsArgs += @('--exit-sets', $exitSets) }
python -I @wsArgs; Check 'workspot'
$bikeXl = Join-Path $tools 'utpp_vehicle_bike.archive.xl'
$tppXl = Join-Path $tools '..\..\..\plugin\UTPP\archive\utpp_vehicle_tpp.archive.xl'
$animReds = Join-Path $tools '..\..\..\plugin\UTPP\scripts\utpp_vehicle_anim.reds'
$gameDump = Join-Path $root 'Build\work\compat-2.5\codeware-modded-250.reds'
if (-not $Release) {
python -I "$tools\check_bike_xl.py" $bikeXl --tpp-xl $tppXl --depot $depot --reds $animReds --game-dump $gameDump `
  --ext-json (Join-Path $tools 'patch-json') --vanilla-json (Join-Path $root 'Extracted\game\vehicle-anims-2.31\json') --selftest
Check 'xl validation'
}
& $wkit pack $stage -o "$Out\packed"; Check 'pack'
if (-not $Release) { Copy-Item $bikeXl "$Out\packed\" }
& $wkit unbundle (Join-Path $Out 'packed\utpp_vehicle_bike.archive') -o "$Out\unpacked" | Out-Null; Check 'unbundle'
$bad = 0
$staged = Get-ChildItem $stage -File -Recurse
Get-ChildItem "$Out\unpacked" -File -Recurse | ForEach-Object {
  $rel = $_.FullName.Substring((Resolve-Path "$Out\unpacked").Path.Length + 1)
  if (-not (Test-Path (Join-Path $stage $rel))) { $bad++; "EXTRA in archive: $rel" }
}
$staged | ForEach-Object {
  $rel = $_.FullName.Substring((Resolve-Path $stage).Path.Length + 1)
  $u = Join-Path "$Out\unpacked" $rel
  $a = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
  $o = (Get-FileHash (Join-Path $srcDepot $_.Name) -Algorithm SHA256 -ErrorAction SilentlyContinue).Hash
  $bb = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  if ($a -ne $bb) { $bad++ }
  '{0} packed=unpacked:{1} same-as-run:{2} {3}' -f $rel, ($a -eq $bb), $(if ($o) { $a -eq $o } else { 'new' }), $a
}
if ($bad) { throw "$bad file(s) differ after unbundle" }
if ($Release) {
  $un = @(Get-ChildItem "$Out\unpacked" -File -Recurse)
  $ws = @($un | Where-Object { $_.Name -eq 'vehicle_motorcycle_workspot.workspot' })
  $an = @($un | Where-Object { $_.Name -like 'utpp_tpp_p?a_sportbike*_enter_?.anims' })
  $nExit = if ($ExitRun) { @($un | Where-Object { $_.Name -like 'utpp_tpp_p?a_sportbike*_exit_?.anims' }).Count } else { 0 }   # -ExitRun with -Release (2026-11-06): release set + the registered exit clips
  if ($un.Count -ne (13 + $nExit) -or $ws.Count -ne 1 -or $an.Count -ne 12 -or ($ExitRun -and $nExit -ne @($exits).Count)) { throw "release set is not exactly 1 workspot + 12 enter .anims + exit .anims (files=$($un.Count) workspot=$($ws.Count) anims=$($an.Count) exits=$nExit)" }
  $tw = Get-ChildItem (Join-Path $TestedBuild 'utpp_vehicle_bike\base') -Filter 'vehicle_motorcycle_workspot.workspot' -File -Recurse | Select-Object -First 1
  if (-not $tw) { throw "tested build has no workspot: $TestedBuild" }
  $same = (Get-FileHash $ws[0].FullName -Algorithm SHA256).Hash -eq (Get-FileHash $tw.FullName -Algorithm SHA256).Hash
  "WORKSPOT equals tested build ($TestedBuild): $same"
  if (-not $same -and -not $ExitRun) { throw "release workspot differs from the tested build $TestedBuild" }   # with -ExitRun it differs by design (the exit sets are registered); make_bike_workspot.py verify covers the non-exit part vs vanilla
}
Get-ChildItem "$Out\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"PACK_OK OUT=$Out"

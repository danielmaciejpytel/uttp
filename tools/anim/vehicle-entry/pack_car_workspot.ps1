# Pack-only build of utpp_vehicle_car.archive: the UTPP third-person car enter clips of one or more build_car.ps1 runs + the override of
# base\vehicles\common\gameplay\vehicle_workspot.workspot (make_car_workspot.py, the OccupantSlots workspot of every player-drivable car;
# see its header for the lookup rule, priorities 127 / 230 and the vars). Validates the workspot (round trip, strip-to-vanilla, NPC rigs
# identical, mutation self-test), packs, unbundles the archive and compares the sha256 of every staged file. Nothing is installed; the
# game folder is not touched. No .xl: the archive only carries depot files (no entity / resource patch).
# Install note: utpp_vehicle_entry.archive + utpp_vehicle_entry.archive.xl (entity-level sport2 registration, priority 95) are obsolete -
# their clips never played; the install must REMOVE both (the new archive carries the sport2 clips under new names, utpp_car_*).
# Output: <Out>\packed\utpp_vehicle_car.archive
param(
  [Parameter(Mandatory)][string[]]$AnimsRun,
  [Parameter(Mandatory)][string]$Out
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
$ErrorActionPreference = 'Stop'
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
if (Test-Path $Out) { throw "output exists, choose a new -Out: $Out" }
$stage = Join-Path $Out 'utpp_vehicle_car'
$depot = Join-Path $stage 'utpp\vehicle'
New-Item -ItemType Directory -Force $depot | Out-Null
$srcOf = @{}
$AnimsRun = @($AnimsRun | ForEach-Object { $_ -split ',' })  # powershell -File passes a comma list as one string
foreach ($r in $AnimsRun) {
  $d = Join-Path $r 'utpp_vehicle_car\utpp\vehicle'
  $files = Get-ChildItem $d -Filter 'utpp_car_*.anims' -File
  if (!$files) { throw "no utpp_car_*.anims in $d" }
  foreach ($f in $files) {
    if ($srcOf.ContainsKey($f.Name)) { throw "duplicate clip $($f.Name) in $r and $($srcOf[$f.Name])" }
    $srcOf[$f.Name] = $f.FullName
    Copy-Item $f.FullName $depot
  }
}
# every front-passenger clip ends on frame 0 of the seated 'idle' clip (build_car.ps1 --end-pose <set>:idle): the matching utpp_car_p?a_<class>_passenger_r_idle
# set (make_passenger_idle.py) must be in the pack, or the game would settle from the clip end into a clip that does not exist for that set
foreach ($n in @($srcOf.Keys | Where-Object { $_ -match '^utpp_car_p[wm]a_[a-z0-9]+_passenger_r\.anims$' })) {
  $idle = $n -replace '_passenger_r\.anims$', '_passenger_r_idle.anims'
  if (-not $srcOf.ContainsKey($idle)) { throw "passenger idle set missing for $n (expected $idle; run make_passenger_idle.py and pass its run dir in -AnimsRun)" }
}
python -I "$tools\check_car_reds.py" (Join-Path $tools '..\..\..\plugin\UTPP\scripts\utpp_vehicle_anim.reds') --selftest; Check 'reds check'
python -I "$tools\make_car_workspot.py" --stage $stage --anims-dir $depot --wkit-exe $wkit --selftest; Check 'workspot'
& $wkit pack $stage -o "$Out\packed"; Check 'pack'
& $wkit unbundle (Join-Path $Out 'packed\utpp_vehicle_car.archive') -o "$Out\unpacked" | Out-Null; Check 'unbundle'
$bad = 0
Get-ChildItem "$Out\unpacked" -File -Recurse | ForEach-Object {
  $rel = $_.FullName.Substring((Resolve-Path "$Out\unpacked").Path.Length + 1)
  if (-not (Test-Path (Join-Path $stage $rel))) { $bad++; "EXTRA in archive: $rel" }
}
Get-ChildItem $stage -File -Recurse | ForEach-Object {
  $rel = $_.FullName.Substring((Resolve-Path $stage).Path.Length + 1)
  $u = Join-Path "$Out\unpacked" $rel
  $a = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
  $o = if ($srcOf.ContainsKey($_.Name)) { (Get-FileHash $srcOf[$_.Name] -Algorithm SHA256).Hash } else { $null }
  $bb = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  if ($a -ne $bb) { $bad++ }
  '{0} packed=unpacked:{1} same-as-run:{2} {3}' -f $rel, ($a -eq $bb), $(if ($o) { $a -eq $o } else { 'new' }), $a
}
if ($bad) { throw "$bad file(s) differ after unbundle" }
Get-ChildItem "$Out\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"PACK_OK OUT=$Out"

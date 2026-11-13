# Pack-only build of utpp_vehicle_bike.archive with the extension component patch (seat pop fix 2026-10-28, route 'ext').
# Reuses the 12 UTPP enter clips of an existing build_bike.ps1 run unchanged (no clip rebuild: same .anims bytes as the installed
# archive of that run), adds utpp\vehicle\utpp_bike_animsets_{wa,ma}.ent (make_bike_ext_patch.py), validates the .xl (animations
# and resource patch section; check_bike_xl.py incl. selftest), packs, unbundles and compares hashes. Nothing is installed; the game folder is not touched.
# Output: <Out>\packed\utpp_vehicle_bike.archive + utpp_vehicle_bike.archive.xl (animations + resource patch section).
param(
  [string]$AnimsRun = '',
  [string]$Out = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $AnimsRun) { $AnimsRun = Join-Path $root 'Build\work\vehicle-bike-archive\run-20261028-074600' }
if (-not $Out) { $Out = Join-Path $root 'Research\logs\diag-20261028-c8\impl-seatpop\pack' }
$ErrorActionPreference = 'Stop'
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
python -I "$tools\make_bike_ext_patch.py" --wkit $stage --wkit-exe $wkit; Check 'ext patch'
$bikeXl = Join-Path $tools 'utpp_vehicle_bike.archive.xl'
$tppXl = Join-Path $tools '..\..\..\plugin\UTPP\archive\utpp_vehicle_tpp.archive.xl'
$animReds = Join-Path $tools '..\..\..\plugin\UTPP\scripts\utpp_vehicle_anim.reds'
$gameDump = Join-Path $root 'Build\work\compat-2.5\codeware-modded-250.reds'
python -I "$tools\check_bike_xl.py" $bikeXl --tpp-xl $tppXl --depot $depot --reds $animReds --game-dump $gameDump `
  --ext-json (Join-Path $tools 'patch-json') --vanilla-json (Join-Path $root 'Extracted\game\vehicle-anims-2.31\json') --selftest
Check 'xl validation'
& $wkit pack $stage -o "$Out\packed"; Check 'pack'
Copy-Item $bikeXl "$Out\packed\"
& $wkit unbundle (Join-Path $Out 'packed\utpp_vehicle_bike.archive') -o "$Out\unpacked" | Out-Null; Check 'unbundle'
$bad = 0
Get-ChildItem $depot -File | ForEach-Object {
  $u = Join-Path "$Out\unpacked\utpp\vehicle" $_.Name
  $a = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
  $o = (Get-FileHash (Join-Path $srcDepot $_.Name) -Algorithm SHA256 -ErrorAction SilentlyContinue).Hash
  $bb = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  if ($a -ne $bb) { $bad++ }
  '{0} packed=unpacked:{1} same-as-run:{2}' -f $_.Name, ($a -eq $bb), $(if ($o) { $a -eq $o } else { 'new' })
}
if ($bad) { throw "$bad file(s) differ after unbundle" }
Get-ChildItem "$Out\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"PACK_OK OUT=$Out"

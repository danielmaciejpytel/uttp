# Pack-only build of the LOCAL TEST archive utpp_ragdoll_poc.archive (+ .archive.xl) for the UTPP ragdoll PoC 'R1'. Not for release.
# Contents (exactly 4 files): the two full-replacement player skeleton rigs with the NPC ragdoll bodies
# (make_ragdoll_rig.py) and the two ArchiveXL patch entities utpp\ragdoll\utpp_ragdoll_{wa,ma}.ent (make_ragdoll_ent_patch.py).
# Nothing is installed; the game folder is only READ (npc_base.ent is unbundled from a game archive into the work dir).
# Output: <OutRoot>\<Run>\packed\utpp_ragdoll_poc.archive + utpp_ragdoll_poc.archive.xl; work files in <Run>\work, stage in <Run>\utpp_ragdoll_poc,
# unbundled copy in <Run>\unpacked. Refuses to reuse an existing run dir.
param(
  [string]$OutRoot = '',
  [string]$Run = ('run-' + (Get-Date -Format 'yyyyMMdd-HHmmss')),
  [switch]$APose,   # also synthesize the player rigs' A-pose (ragdoll joint frames), see make_ragdoll_rig.py synth_apose
  [string[]]$Limit = @()   # angle range overrides, make_ragdoll_rig.py --limit BODY:FIELD=a,b (e.g. LeftForeArm:SwingAnglesY=40,80)
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $OutRoot) { $OutRoot = Join-Path $root 'Build\work\ragdoll-poc' }
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
function Check($what) { if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" } }
$out = Join-Path $OutRoot $Run
if (Test-Path $out) { throw "output exists, choose a new -Run: $out" }
$stage = Join-Path $out 'utpp_ragdoll_poc'
$work = Join-Path $out 'work'
New-Item -ItemType Directory -Force $stage | Out-Null
New-Item -ItemType Directory -Force $work | Out-Null

$rigArgs = @('--stage', $stage, '--work', (Join-Path $work 'rig'), '--wkit-exe', $wkit)
if ($APose) { $rigArgs += '--apose' }
foreach ($l in $Limit) { $rigArgs += @('--limit', $l) }
python -I "$tools\make_ragdoll_rig.py" @rigArgs; Check 'rig build'
python -I "$tools\make_ragdoll_ent_patch.py" --stage $stage --work (Join-Path $work 'ent') --wkit-exe $wkit; Check 'ent patch'

$expected = @(
  'base\characters\entities\player\player_woman_skeleton.rig',
  'base\characters\entities\player\player_man_skeleton.rig',
  'utpp\ragdoll\utpp_ragdoll_wa.ent',
  'utpp\ragdoll\utpp_ragdoll_ma.ent'
)
$staged = @(Get-ChildItem $stage -Recurse -File | ForEach-Object { $_.FullName.Substring($stage.Length + 1) })
if ($staged.Count -ne 4) { throw "stage must hold exactly 4 files, found $($staged.Count): $($staged -join ', ')" }
foreach ($e in $expected) { if ($staged -notcontains $e) { throw "stage file missing: $e" } }

# .xl sanity: every patch resource it names is packed, every target is one of the four player entities
$xl = Join-Path $tools 'utpp_ragdoll_poc.archive.xl'
$xlText = Get-Content $xl -Raw
foreach ($e in $expected | Where-Object { $_ -like '*.ent' }) { if ($xlText -notmatch [regex]::Escape($e)) { throw ".xl does not name $e" } }
foreach ($t in 'base\characters\entities\player\player_wa_fpp.ent','ep1\characters\entities\player\player_wa_fpp_ep1.ent',
               'base\characters\entities\player\player_ma_fpp.ent','ep1\characters\entities\player\player_ma_fpp_ep1.ent') {
  if ($xlText -notmatch [regex]::Escape($t)) { throw ".xl does not name target $t" }
}

& $wkit pack $stage -o "$out\packed"; Check 'pack'
Copy-Item $xl (Join-Path $out 'packed\utpp_ragdoll_poc.archive.xl')
$arch = Join-Path $out 'packed\utpp_ragdoll_poc.archive'
if (-not (Test-Path $arch)) { throw "pack produced no $arch" }
& $wkit unbundle $arch -o "$out\unpacked" | Out-Null; Check 'unbundle'

$unpackedFiles = @(Get-ChildItem "$out\unpacked" -Recurse -File)
if ($unpackedFiles.Count -ne 4) { throw "unbundle returned $($unpackedFiles.Count) files, expected 4" }
$bad = 0
foreach ($e in $expected) {
  $a = (Get-FileHash (Join-Path $stage $e) -Algorithm SHA256).Hash
  $u = Join-Path "$out\unpacked" $e
  $b = if (Test-Path $u) { (Get-FileHash $u -Algorithm SHA256).Hash } else { 'MISSING' }
  if ($a -ne $b) { $bad++ }
  '{0}  staged=unpacked:{1}  {2}' -f $a, ($a -eq $b), $e
}
if ($bad) { throw "$bad file(s) differ after unbundle" }
Get-ChildItem "$out\packed" -File | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName }
"PACK_OK OUT=$out"

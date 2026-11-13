# TPP melee export for the NPC-based one-handed blunt / baton clips (2026-11-09): runs Source\tools\anim\melee-export\run_melee_export.ps1
# (NOT modified) through a patched copy written to Build\work\melee-npc-blunt-20261109\_tmp\run_melee_export_ext.ps1:
#   1 the set list also accepts pwa_baton / pma_baton;
#   2 $tools points at Source\tools\anim\melee-export (the original tools);
#   3 build_melee_set.py / patch_melee_json.py / verify_melee_set.py run through npc_blunt_export.py (baton added to melee_export_lib lists);
#   4 -ExtraStaged <dir>: the *.anims of that dir are copied byte for byte into the staging folder before the manifest / pack (used for the
#     installed fists sets: the archive keeps utpp_pwa_fists / utpp_pma_fists unchanged next to the new sets; logged with SHA-256).
# Every replacement must match exactly once (else the script stops).  Parameters are passed through to run_melee_export.ps1.
#   powershell -ExecutionPolicy Bypass -File npc_blunt_export.ps1 -Input <clips dir> -Rig woman|man|both -Sets <a,b> -Out <new dir under Build\work>
#              [-Suffix npc1] [-Vanilla <frozen dir>] [-Registration <json>] [-ExtraStaged <dir>] [-Identity] [-NoPack]
param(
  [Parameter(Mandatory = $true)][Alias('Input')][string]$InputDir,
  [ValidateSet('woman', 'man', 'both')][string]$Rig = 'both',
  [Parameter(Mandatory = $true)][string]$Sets,
  [Parameter(Mandatory = $true)][string]$Out,
  [string]$Suffix = 'npc1',
  [string]$Vanilla = '',
  [string]$Registration = '',
  [string]$ExtraStaged = '',
  [switch]$Identity,
  [switch]$NoPack
)
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Vanilla) { $Vanilla = Join-Path $root 'Build\work\melee-npc-blunt-20261109\vanilla-frozen' }
if (-not $Registration) { $Registration = Join-Path $root 'Build\work\melee-npc-blunt-20261109\evidence\registration.json' }
$ErrorActionPreference = 'Stop'
$me = Join-Path $root 'Source\tools\anim\melee-export'
$orig = Join-Path $me 'run_melee_export.ps1'
$shim = Join-Path $root 'Source\tools\melee\npc_blunt_export.py'
$dst = Join-Path $root 'Build\work\melee-npc-blunt-20261109\_tmp\run_melee_export_ext.ps1'
$s = [IO.File]::ReadAllText($orig)
function Sub([string]$old, [string]$new) {
  $n = ([regex]::Matches($script:s, [regex]::Escape($old))).Count
  if ($n -ne 1) { throw "patch: '$old' found $n times" }
  $script:s = $script:s.Replace($old, $new)
}
Sub "'pwa_two_handed_blunt', 'pma_two_handed_blunt')" "'pwa_two_handed_blunt', 'pma_two_handed_blunt', 'pwa_baton', 'pma_baton')"
Sub '$tools = Split-Path -Parent $MyInvocation.MyCommand.Path' ("`$tools = '" + $me + "'")
Sub '''-I'', "$tools\build_melee_set.py"' ('''-I'', ''' + $shim + ''', "$tools\build_melee_set.py"')
Sub '& $py -I "$tools\patch_melee_json.py"' ('& $py -I ''' + $shim + ''' "$tools\patch_melee_json.py"')
Sub '& $py -I "$tools\verify_melee_set.py"' ('& $py -I ''' + $shim + ''' "$tools\verify_melee_set.py"')
Sub '  [switch]$NoPack' "  [switch]`$NoPack,`r`n  [string]`$ExtraStaged = ''"
Sub '$staged = @(Get-ChildItem $stage -File)' (@'
if ($ExtraStaged) {
  Get-ChildItem -LiteralPath $ExtraStaged -Filter *.anims -File | ForEach-Object {
    $t = Join-Path $stage $_.Name
    if (Test-Path -LiteralPath $t) { throw "extra staged file collides with a built set: $($_.Name)" }
    Copy-Item -LiteralPath $_.FullName $t
    Log "EXTRA STAGED (byte copy, not rebuilt): $($_.Name) sha256 $((Get-FileHash $t -Algorithm SHA256).Hash) from $($_.FullName)" }
}
$staged = @(Get-ChildItem $stage -File)
'@)
[void][IO.Directory]::CreateDirectory((Split-Path -Parent $dst))   # _tmp is deleted when a round is closed
[IO.File]::WriteAllText($dst, $s)
"patched copy: $dst  (original sha256 $((Get-FileHash $orig -Algorithm SHA256).Hash))"
$a = @{ InputDir = $InputDir; Rig = $Rig; Sets = $Sets; Out = $Out; Suffix = $Suffix; Vanilla = $Vanilla; Registration = $Registration }
if ($ExtraStaged) { $a.ExtraStaged = $ExtraStaged }
if ($Identity) { $a.Identity = $true }
if ($NoPack) { $a.NoPack = $true }
& $dst @a
exit $LASTEXITCODE

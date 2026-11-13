# Second working copy for parallel work: Build\wt\<Name>\ looks like the project root (junctions to the shared folders) and holds
# its own checkout in Build\wt\<Name>\Source on branch work/<Name>, so every relative path of the build, tests and tools resolves as in
# the main copy. Own build folder and log: set UTPP_SRC / UTPP_BUILD / UTPP_LOG (printed below) before Build\build-main.cmd.
# Remove after merging: -Remove (deletes the junctions first, never their targets).
# -ExtraFiles: names of files from the project root to copy into the working copy (default: none).
param([Parameter(Mandatory = $true)][ValidatePattern('^[a-z0-9_]+$')][string]$Name, [switch]$Remove, [string[]]$ExtraFiles = @())
$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path (Split-Path $PSScriptRoot))
$wt = Join-Path $root "Build\wt\$Name"
$src = Join-Path $root 'Source'
$top = 'Data', 'Tools', 'Research', 'Extracted', 'Animations', 'Media'
$sub = 'UTPP', 'test-data', 'stages', 'work'
function Remove-Junction([string]$p) {
    if (Test-Path -LiteralPath $p) {
        $i = Get-Item -LiteralPath $p -Force
        if ($i.Attributes -band [IO.FileAttributes]::ReparsePoint) { [IO.Directory]::Delete($p, $false) } else { throw "not a junction: $p" }
    }
}
if ($Remove) {
    foreach ($t in $top) { Remove-Junction (Join-Path $wt $t) }
    foreach ($s in $sub) { Remove-Junction (Join-Path $wt "Build\$s") }
    if (Test-Path -LiteralPath (Join-Path $wt 'Source')) { git -C $src worktree remove --force (Join-Path $wt 'Source') }
    foreach ($d in 'Build', 'out') { $p = Join-Path $wt $d; if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -Recurse -Force } }
    if ((Test-Path -LiteralPath $wt) -and -not (Get-ChildItem -LiteralPath $wt -Force)) { Remove-Item -LiteralPath $wt -Force }
    git -C $src worktree prune
    "removed $wt (branch work/$Name is kept until merged or deleted by hand)"
    return
}
New-Item -ItemType Directory -Force -Path (Join-Path $wt 'Build') | Out-Null
foreach ($t in $top) {
    $l = Join-Path $wt $t
    if (-not (Test-Path -LiteralPath $l)) { cmd /c mklink /J "$l" "$(Join-Path $root $t)" | Out-Null }
}
foreach ($s in $sub) {
    $l = Join-Path $wt "Build\$s"
    if (-not (Test-Path -LiteralPath $l)) { cmd /c mklink /J "$l" "$(Join-Path $root "Build\$s")" | Out-Null }
}
foreach ($f in $ExtraFiles) { Copy-Item -LiteralPath (Join-Path $root $f) -Destination (Join-Path $wt $f) -Force }
if (-not (Test-Path -LiteralPath (Join-Path $wt 'Source'))) { git -C $src worktree add (Join-Path $wt 'Source') -b "work/$Name" | Out-Null }
"working copy : $wt\Source  (branch work/$Name)"
"UTPP_SRC     = $wt\Source\plugin\UTPP"
"UTPP_BUILD   = $wt\out"
"UTPP_LOG     = $root\Research\logs\build\build-$Name.log"

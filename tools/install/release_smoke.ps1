param(
    [ValidateSet('Prepare', 'Restore')][string]$Mode = 'Prepare',
    [switch]$Apply,
    [string]$Zip = '',
    [string]$ReleaseManifest = '',
    [string]$BackupDir = ''
)
# Clean smoke test of a release zip. Prepare: move the whole developer UTPP footprint out of the game folder into
# Backups\<date>\pre-release-smoke-<time> (refuses hardlinked files: Vortex), extract the release zip, verify every hash
# against the release manifest. Restore -BackupDir <dir>: remove exactly the release files (only if their hash still matches),
# move the developer footprint back. Without -Apply both modes only print what they would do.
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
$game = (Get-UtppGameDir).TrimEnd('\')
if (-not $Zip) { $Zip = Join-Path $root 'Build\work\release-0.1.0\UTPP-0.1.0.zip' }
if (-not $ReleaseManifest) { $ReleaseManifest = Join-Path $root 'Research\inventory\utpp-release-0.1.0.json' }
if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Cyberpunk 2077 is running - close the game first' }

function Get-Footprint {
    $items = @()
    foreach ($d in 'red4ext\plugins\UTPP', 'r6\scripts\UTPP') { if (Test-Path (Join-Path $game $d)) { $items += $d } }
    Get-ChildItem (Join-Path $game 'bin\x64\plugins\cyber_engine_tweaks\mods') -Directory -Filter 'utpp_*' -ErrorAction SilentlyContinue |
        ForEach-Object { $items += "bin\x64\plugins\cyber_engine_tweaks\mods\$($_.Name)" }
    Get-ChildItem (Join-Path $game 'archive\pc\mod') -File -Filter 'utpp_*' -ErrorAction SilentlyContinue |
        ForEach-Object { $items += "archive\pc\mod\$($_.Name)" }
    Get-ChildItem (Join-Path $game 'r6\input') -File -Filter 'utpp_*.xml' -ErrorAction SilentlyContinue |
        ForEach-Object { $items += "r6\input\$($_.Name)" }
    return $items
}
function Assert-NoHardlinks([string]$path) {
    $files = if (Test-Path $path -PathType Container) { Get-ChildItem $path -Recurse -File } else { Get-Item $path }
    foreach ($f in $files) {
        $n = @(fsutil hardlink list $f.FullName).Count
        if ($n -gt 1) { throw "hardlinked file (Vortex?): $($f.FullName) - aborting, nothing moved" }
    }
}
$rel = Get-Content $ReleaseManifest -Raw | ConvertFrom-Json
$relFiles = @($rel.files)

if ($Mode -eq 'Prepare') {
    $fp = Get-Footprint
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $bk = Join-Path $root ("Backups\" + (Get-Date -Format 'yyyy-MM-dd') + "\pre-release-smoke-$stamp")
    Write-Output "footprint ($($fp.Count) entries):"; $fp | ForEach-Object { Write-Output "  $_" }
    Write-Output "backup dir: $bk"
    Write-Output "release files to extract: $($relFiles.Count) from $Zip"
    if (!$Apply) { Write-Output 'PREVIEW_OK (run again with -Apply)'; exit 0 }
    foreach ($e in $fp) { Assert-NoHardlinks (Join-Path $game $e) }
    New-Item -ItemType Directory -Force $bk | Out-Null
    $moved = @()
    foreach ($e in $fp) {
        $dst = Join-Path $bk $e
        New-Item -ItemType Directory -Force (Split-Path $dst -Parent) | Out-Null
        Move-Item -LiteralPath (Join-Path $game $e) -Destination $dst
        $moved += $e
    }
    $moved | Set-Content -Encoding utf8 (Join-Path $bk 'moved-entries.txt')
    Expand-Archive -LiteralPath $Zip -DestinationPath $game -Force:$false
    $bad = 0
    foreach ($f in $relFiles) {
        $p = Join-Path $game ($f.path -replace '/', '\')
        if (!(Test-Path $p) -or (Get-FileHash $p -Algorithm SHA256).Hash -ne $f.sha256.ToUpper()) { $bad++; Write-Output "HASH MISMATCH: $($f.path)" }
    }
    if ($bad) { throw "$bad release files do not match the manifest" }
    Write-Output "PREPARE_OK moved=$($moved.Count) extracted=$($relFiles.Count) backup=$bk"
    Write-Output "restore later: powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Mode Restore -BackupDir `"$bk`" -Apply"
}
else {
    if (!$BackupDir -or !(Test-Path (Join-Path $BackupDir 'moved-entries.txt'))) { throw 'Restore needs -BackupDir of a Prepare run' }
    $moved = Get-Content (Join-Path $BackupDir 'moved-entries.txt')
    Write-Output "will remove $($relFiles.Count) release files and move back $($moved.Count) entries from $BackupDir"
    if (!$Apply) { Write-Output 'PREVIEW_OK (run again with -Apply)'; exit 0 }
    foreach ($f in $relFiles) {
        $p = Join-Path $game ($f.path -replace '/', '\')
        if (Test-Path $p) {
            if ((Get-FileHash $p -Algorithm SHA256).Hash -ne $f.sha256.ToUpper()) { throw "changed release file, not removing: $p" }
            Remove-Item -LiteralPath $p
        }
    }
    foreach ($e in $moved) {
        $dst = Join-Path $game $e
        if (Test-Path $dst) {
            # created by the game during the smoke test (logs, CET db.sqlite3, settings): keep them inside the backup dir
            $side = Join-Path $BackupDir ("smoke-leftovers\" + $e)
            New-Item -ItemType Directory -Force (Split-Path $side -Parent) | Out-Null
            Move-Item -LiteralPath $dst -Destination $side
        }
        New-Item -ItemType Directory -Force (Split-Path $dst -Parent) | Out-Null
        Move-Item -LiteralPath (Join-Path $BackupDir $e) -Destination $dst
    }
    Write-Output "RESTORE_OK entries=$($moved.Count)"
}

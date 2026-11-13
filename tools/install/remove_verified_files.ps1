<#
.SYNOPSIS
Removes listed UTPP leftovers from the game folder: files with a known SHA-256, runtime files without a pin (both backed up first) and directories that end up empty.
.DESCRIPTION
Counterpart of install_verified_stage.ps1 for removals.
Manifest: Research\release\<ManifestName>, JSON
  { "files": [ { "path": "...", "sha256": "..." } ], "runtimeFiles": [ "..." ], "emptyDirs": [ "..." ] }
paths relative to the game folder.
  files         hash-pinned files; a different hash stops the run (the file is not what the manifest was made for).
  runtimeFiles  files the game, CET or the DLL rewrite (db.sqlite3, logs, settings): no hash, but each is backed up and compared with its backup.
  emptyDirs     directories that must be empty AFTER the listed files are removed; a file that is not in the manifest keeps the run from starting.
Without -Apply only checks (PREFLIGHT_OK). With -Apply: game must be closed; every file is copied to
Backups\YYYY-MM-DD\<ManifestBase>-remove-<stamp> and the copy verified before Remove-Item (removing the directory entry keeps any
Vortex hardlink target intact). A report goes to Research\inventory.
Run it BEFORE install_verified_stage.ps1 (the install skips a file whose hash already matches, so removing afterwards would delete a
shipped file that is also listed here) and before the game is started (the game rewrites runtime files; a hash-pinned file never changes).
Allowed paths: red4ext\plugins\UTPP\*, r6\scripts\UTPP\*, CET mods utpp_* (runtime files: db.sqlite3, *.log, utpp_settings.txt,
utpp_modsettings_sync.txt of a mod other than the probe mods), the old-layout folders v2, v2\recoil, db and CET mods\utpp_v2, the
test-install folder helper_candidate (with man and woman; replaced by skin_helpers, 2026-11-12), and the six named empty probe directories
of the 2.5 port.
#>
param(
    [switch]$Apply,
    [Parameter(Mandatory = $true)][string]$ManifestName
)
$ErrorActionPreference = 'Stop'
if ($ManifestName -notmatch '^[A-Za-z0-9_-]+\.json$') { throw 'Unsafe manifest name.' }
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\')
# Removal manifests live in Research\release (unlike install_verified_stage.ps1, which reads Research\inventory).
$manifest = Get-Content -Raw -LiteralPath (Join-Path $root ('Research\release\' + $ManifestName)) | ConvertFrom-Json
if (!(Test-Path -LiteralPath (Join-Path $game 'bin\x64\Cyberpunk2077.exe'))) { throw 'Game executable missing.' }
if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Close the game before removal.' }
$fileRx = '^(red4ext\\plugins\\UTPP\\|r6\\scripts\\UTPP\\|bin\\x64\\plugins\\cyber_engine_tweaks\\mods\\utpp_)'
$runtimeRx = '^bin\\x64\\plugins\\cyber_engine_tweaks\\mods\\utpp_(?!probe\\|reload_probe\\)[A-Za-z0-9_]+\\(db\.sqlite3|[A-Za-z0-9_]+\.log|utpp_settings\.txt|utpp_modsettings_sync\.txt)$'
$dirRx = '^(red4ext\\plugins\\(ApiProbe|CodewareTestDriver|EngineCallbackProbe|HandlerProbe)|r6\\scripts\\(CodewareCompat25Test|UTPPCompat25EngineProbe)|red4ext\\plugins\\UTPP\\(v2\\recoil|v2|db|helper_candidate\\man|helper_candidate\\woman|helper_candidate)|bin\\x64\\plugins\\cyber_engine_tweaks\\mods\\utpp_v2)$'
function Resolve-GamePath([string]$relative) {
    if ([IO.Path]::IsPathRooted($relative) -or $relative -match '(^|[\\/])\.\.([\\/]|$)') { throw "Unsafe path: $relative" }
    $full = [IO.Path]::GetFullPath((Join-Path $game $relative))
    if (!$full.StartsWith($game + '\')) { throw "Path escaped the game root: $relative" }
    return $full
}
$files = @(foreach ($f in @($manifest.files)) {
    if (!$f) { continue }
    if ($f.path -notmatch $fileRx) { throw "Unapproved file: $($f.path)" }
    $t = Resolve-GamePath $f.path
    $present = Test-Path -LiteralPath $t -PathType Leaf
    if ($present) {
        if ((Get-Item -LiteralPath $t).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse target: $($f.path)" }
        if ((Get-FileHash -LiteralPath $t -Algorithm SHA256).Hash -ne $f.sha256) { throw "Hash mismatch (file changed): $($f.path)" }
    }
    [pscustomobject]@{ path = $f.path; target = $t; sha256 = $f.sha256; present = $present; runtime = $false }
})
$runtime = @(foreach ($p in @($manifest.runtimeFiles)) {
    if (!$p) { continue }
    if ($p -notmatch $runtimeRx) { throw "Unapproved runtime file: $p" }
    $t = Resolve-GamePath $p
    $present = Test-Path -LiteralPath $t -PathType Leaf
    if ($present -and ((Get-Item -LiteralPath $t).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw "Reparse target: $p" }
    [pscustomobject]@{ path = $p; target = $t; sha256 = $(if ($present) { (Get-FileHash -LiteralPath $t -Algorithm SHA256).Hash } else { $null }); present = $present; runtime = $true }
})
$all = @($files) + @($runtime)
$removeSet = @{}
foreach ($x in $all) { $removeSet[$x.target.ToLowerInvariant()] = $true }
$dirEntries = @(foreach ($d in @($manifest.emptyDirs)) {
    if (!$d) { continue }
    if ($d -notmatch $dirRx) { throw "Unapproved directory: $d" }
    [pscustomobject]@{ path = $d; target = (Resolve-GamePath $d) }
})
$dirSet = @{}
foreach ($d in $dirEntries) { $dirSet[$d.target.ToLowerInvariant()] = $true }
$dirs = @(foreach ($d in $dirEntries) {
    $present = Test-Path -LiteralPath $d.target -PathType Container
    if ($present) {
        # everything inside must be a file of this manifest or a directory of this manifest: otherwise the directory would not be empty afterwards
        foreach ($item in @(Get-ChildItem -LiteralPath $d.target -Recurse -Force)) {
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse point inside $($d.path): $($item.FullName)" }
            $key = $item.FullName.ToLowerInvariant()
            if ($item.PSIsContainer) { if (!$dirSet.ContainsKey($key)) { throw "Directory $($d.path) holds a folder that is not in the manifest: $($item.FullName)" } }
            elseif (!$removeSet.ContainsKey($key)) { throw "Directory $($d.path) holds a file that is not in the manifest: $($item.FullName)" }
        }
    }
    [pscustomobject]@{ path = $d.path; target = $d.target; present = $present }
}) | Sort-Object { $_.target.Length } -Descending
if (!$Apply) {
    Write-Output ("PREFLIGHT_OK files={0} (present {1}) runtimeFiles={2} (present {3}) dirs={4} (present {5}) noWritesToGame=true" -f $files.Count, @($files | Where-Object present).Count, $runtime.Count, @($runtime | Where-Object present).Count, $dirs.Count, @($dirs | Where-Object present).Count)
    exit 0
}
$now = Get-Date
$stamp = $now.ToString('yyyyMMdd-HHmmss')
$base = [IO.Path]::GetFileNameWithoutExtension($ManifestName)
$backup = Join-Path $root ("Backups\" + $now.ToString('yyyy-MM-dd') + "\$base-remove-$stamp")
if (Test-Path -LiteralPath $backup) { throw 'Backup already exists.' }
New-Item -ItemType Directory -Path $backup | Out-Null
foreach ($f in $all | Where-Object present) {
    $saved = Join-Path $backup $f.path
    New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($saved)) | Out-Null
    Copy-Item -LiteralPath $f.target -Destination $saved
    if ((Get-FileHash -LiteralPath $saved).Hash -ne $f.sha256) { throw "Backup mismatch: $($f.path)" }
}
$reportPath = Join-Path $root "Research\inventory\utpp-remove-$base-$stamp.json"
$report = [ordered]@{ startedAt = [DateTimeOffset]::UtcNow.ToString('o'); completedAt = $null; game = $game; manifest = $ManifestName; backup = $backup; state = 'backed-up'
    files = @($all | Select-Object path, sha256, present, runtime); emptyDirs = @($dirs | Select-Object path, present) }
$report | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $reportPath
foreach ($f in $all | Where-Object present) {
    if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Game started during removal; stop.' }
    if ((Get-FileHash -LiteralPath $f.target).Hash -ne $f.sha256) { throw "Target changed since preflight: $($f.path)" }
    Remove-Item -LiteralPath $f.target
    if (Test-Path -LiteralPath $f.target) { throw "Still present: $($f.path)" }
}
foreach ($d in $dirs | Where-Object present) {
    if (@(Get-ChildItem -LiteralPath $d.target -Force).Count -ne 0) { throw "Directory no longer empty: $($d.path)" }
    Remove-Item -LiteralPath $d.target
    if (Test-Path -LiteralPath $d.target) { throw "Still present: $($d.path)" }
}
$report.state = 'removed-verified'
$report.completedAt = [DateTimeOffset]::UtcNow.ToString('o')
$report | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $reportPath
Write-Output "REMOVE_OK files=$(@($all | Where-Object present).Count) dirs=$(@($dirs | Where-Object present).Count) report=$reportPath backup=$backup"

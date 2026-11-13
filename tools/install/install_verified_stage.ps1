param(
    [switch]$Apply,
    # Defaults: the current release stage (Build\stages\utpp-release) and its manifest (Research\inventory\utpp-release-010-game250.json).
    [string]$StageName = 'utpp-release',
    [string]$ManifestName = 'utpp-release-010-game250.json'
)
$ErrorActionPreference = 'Stop'
if ($StageName -notmatch '^[A-Za-z0-9_-]+$' -or $ManifestName -notmatch '^[A-Za-z0-9_-]+\.json$') { throw 'Unsafe stage or manifest name.' }
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\')
$stage = Join-Path $root ('Build\stages\' + $StageName)   # stages live in Build\stages since 2026-11-05
if (!(Test-Path -LiteralPath $stage)) { $stage = Join-Path $root ('Build\' + $StageName) }   # fallback: pre-2026-11-05 layout (Build\<stage>)
$manifest = Get-Content -Raw -LiteralPath (Join-Path $root ('Research\inventory\' + $ManifestName)) | ConvertFrom-Json
if (!(Test-Path -LiteralPath (Join-Path $game 'bin\x64\Cyberpunk2077.exe'))) { throw 'Game executable missing.' }
if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Close the game before installation.' }
foreach ($dependency in @('bin\x64\plugins\cyber_engine_tweaks.asi','red4ext\plugins\Codeware\Codeware.dll','red4ext\plugins\ArchiveXL\ArchiveXL.dll','engine\tools\scc.exe')) {
    if (!(Test-Path -LiteralPath (Join-Path $game $dependency))) { throw "Missing runtime dependency: $dependency" }
}
$entries = foreach ($file in $manifest.files) {
    $relative = $file.path
    # MotionDriver resolves recoil beside its database, under the DLL data root (red4ext\plugins\UTPP\data).
    # The legacy db\v2 location silently disables the authored moving-shot path.
    if ($relative -match '^red4ext\\plugins\\UTPP\\db\\v2\\recoil\\') {
        throw "Recoil is outside the runtime v2 data root: $relative (use red4ext\plugins\UTPP\data\recoil)."
    }
    if ([IO.Path]::IsPathRooted($relative) -or $relative -match '(^|[\\/])\.\.([\\/]|$)') { throw "Unsafe path: $relative" }
    if ($relative -notmatch '^(red4ext\\plugins\\UTPP\\|r6\\scripts\\UTPP\\|r6\\input\\utpp_shoulder\.xml$|bin\\x64\\plugins\\cyber_engine_tweaks\\mods\\utpp_|archive\\pc\\mod\\utpp_camera\.archive|archive\\pc\\mod\\utpp_female_tpp_nails_plane\.archive$|archive\\pc\\mod\\utpp_nue_reload\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_foot_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_vehicle_entry\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_bike\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_car\.archive$|archive\\pc\\mod\\utpp_weapon_magazine_fix\.archive$|archive\\pc\\mod\\utpp_melee\.archive$|archive\\pc\\mod\\utpp_melee_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_ragdoll_poc\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_ragdoll_graph\.archive$|archive\\pc\\mod\\utpp_melee_parried_graph\.archive$)') { throw "Unapproved target: $relative" }
    $source = [IO.Path]::GetFullPath((Join-Path $stage $relative))
    $target = [IO.Path]::GetFullPath((Join-Path $game $relative))
    if (!$source.StartsWith($stage + '\') -or !$target.StartsWith($game + '\')) { throw 'Path escaped its root.' }
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $file.sha256) { throw "Stage hash mismatch: $relative" }
    if (Test-Path -LiteralPath $target) {
        if ((Get-Item -LiteralPath $target).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse target: $relative" }
    }
    [pscustomobject]@{ path=$relative; source=$source; target=$target; sha256=$file.sha256; beforeSha256=$(if (Test-Path -LiteralPath $target) { (Get-FileHash -LiteralPath $target).Hash } else { $null }) }
}
if (!$Apply) { Write-Output "PREFLIGHT_OK files=$(@($entries).Count) noWritesToGame=true"; exit 0 }
# Unique backup; back up every existing target, including all databases, before any game mutation.
$localNow = Get-Date
$stamp = $localNow.ToString('yyyyMMdd-HHmmss')
$backup = Join-Path $root ("Backups\" + $localNow.ToString('yyyy-MM-dd') + "\$StageName-install-$stamp")
if (Test-Path -LiteralPath $backup) { throw 'Backup already exists.' }
New-Item -ItemType Directory -Path $backup | Out-Null
foreach ($entry in $entries) {
    if ($entry.beforeSha256) {
        $saved = Join-Path $backup $entry.path
        New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($saved)) | Out-Null
        Copy-Item -LiteralPath $entry.target -Destination $saved
        if ((Get-FileHash -LiteralPath $saved).Hash -ne $entry.beforeSha256) { throw "Backup mismatch: $($entry.path)" }
    }
}
$reportPath = Join-Path $root "Research\inventory\utpp-install-$StageName-$stamp.json"
$report = [ordered]@{ startedAt=[DateTimeOffset]::UtcNow.ToString('o'); completedAt=$null; sourceCommit=(& git -C (Join-Path $root 'Source') rev-parse HEAD); game=$game; stage=$stage; stageManifest=$ManifestName; backup=$backup; localPlaceholdersOnly=$true; state='backed-up'; files=@($entries | Select-Object path,sha256,beforeSha256) }
$report | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $reportPath
foreach ($entry in $entries) {
    if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Game started during installation; stop.' }
    $current = if (Test-Path -LiteralPath $entry.target) { (Get-FileHash -LiteralPath $entry.target).Hash } else { $null }
    if ($current -ne $entry.beforeSha256) { throw "Target changed since preflight: $($entry.path)" }
    if ($current -ne $entry.sha256) {
        New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($entry.target)) | Out-Null
        # Removing the directory entry first preserves Vortex's hardlinked original.
        if (Test-Path -LiteralPath $entry.target) { Remove-Item -LiteralPath $entry.target }
        Copy-Item -LiteralPath $entry.source -Destination $entry.target
    }
    if ((Get-FileHash -LiteralPath $entry.target).Hash -ne $entry.sha256) { throw "Installed hash mismatch: $($entry.path)" }
}
$report.state='installed-hashes-verified'
$report.completedAt=[DateTimeOffset]::UtcNow.ToString('o')
$report | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $reportPath
Write-Output "INSTALL_OK files=$(@($entries).Count) manifest=$reportPath backup=$backup"

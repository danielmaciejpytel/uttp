# Remove ONE UTPP archive (and its .archive.xl) from the game's archive\pc\mod, with a verified backup.
# Default is a DRY RUN: it only reports what would happen. Use -Apply only after reviewing the dry run.
#
#   disable_utpp_archive.ps1 -ArchiveName utpp_nue_reload            # dry run
#   disable_utpp_archive.ps1 -ArchiveName utpp_nue_reload -Apply     # backup + remove
#
# Refuses when: the game is running; the name is not a plain utpp_* base name; a target is a reparse point or has
# more than one hard link (Vortex-managed copy: remove it through Vortex instead, never through a hardlink); the
# backup copy hash differs. Writes a manifest to Research\inventory\. Restore = copy the backup files back.
param(
    [Parameter(Mandatory = $true)][string]$ArchiveName,
    [switch]$Apply
)
$ErrorActionPreference = 'Stop'
if ($ArchiveName -notmatch '^utpp_[A-Za-z0-9_]+$') { throw "Unsafe archive base name: $ArchiveName (expected utpp_<name>, no extension)" }
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\')
$modDir = Join-Path $game 'archive\pc\mod'
if (!(Test-Path -LiteralPath (Join-Path $game 'bin\x64\Cyberpunk2077.exe'))) { throw 'Game executable missing.' }
if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Close the game first.' }

if (-not ('UtppLinkCount2' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class UtppLinkCount2 {
    [StructLayout(LayoutKind.Sequential)]
    struct BY_HANDLE_FILE_INFORMATION {
        public uint Attr; public uint CLo, CHi, ALo, AHi, WLo, WHi; public uint Vol; public uint SizeHi; public uint SizeLo;
        public uint Links; public uint IdxHi; public uint IdxLo;
    }
    [DllImport("kernel32.dll", SetLastError = true)]
    static extern bool GetFileInformationByHandle(SafeFileHandle h, out BY_HANDLE_FILE_INFORMATION info);
    public static uint Get(string path) {
        using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete)) {
            BY_HANDLE_FILE_INFORMATION info;
            if (!GetFileInformationByHandle(fs.SafeFileHandle, out info)) throw new IOException("GetFileInformationByHandle failed: " + Marshal.GetLastWin32Error());
            return info.Links;
        }
    }
}
'@
}

$entries = @()
foreach ($leaf in @("$ArchiveName.archive", "$ArchiveName.archive.xl")) {
    $target = [IO.Path]::GetFullPath((Join-Path $modDir $leaf))
    if (!$target.StartsWith($modDir + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Target escaped archive\pc\mod.' }
    if (!(Test-Path -LiteralPath $target)) { continue }
    $item = Get-Item -LiteralPath $target
    if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse point, refusing: $leaf" }
    $links = [UtppLinkCount2]::Get($target)
    if ($links -gt 1) { throw "Hardlinked file (links=$links, Vortex-managed?), refusing: $leaf" }
    $entries += [pscustomobject]@{ path = "archive\pc\mod\$leaf"; target = $target; sha256 = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash; bytes = $item.Length; links = $links }
}
if (!$entries.Count) { throw "Nothing to disable: $ArchiveName(.archive/.xl) not found in archive\pc\mod" }
if (!$Apply) {
    $entries | ForEach-Object { Write-Output ("DRY_RUN would back up and remove {0} sha256={1} bytes={2} links={3}" -f $_.path, $_.sha256, $_.bytes, $_.links) }
    Write-Output "DRY_RUN_OK files=$($entries.Count) noWritesToGame=true"
    exit 0
}

$now = Get-Date
$stamp = $now.ToString('yyyyMMdd-HHmmss')
$backup = Join-Path $root ('Backups\' + $now.ToString('yyyy-MM-dd') + "\$ArchiveName-disabled-$stamp")
if (Test-Path -LiteralPath $backup) { throw 'Backup folder already exists.' }
# Back up everything first; nothing is removed unless every backup hash matches.
foreach ($e in $entries) {
    $saved = Join-Path $backup $e.path
    New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($saved)) | Out-Null
    Copy-Item -LiteralPath $e.target -Destination $saved
    if ((Get-FileHash -LiteralPath $saved -Algorithm SHA256).Hash -ne $e.sha256) { throw "Backup mismatch: $($e.path)" }
}
$manifest = Join-Path $root "Research\inventory\utpp-archive-disabled-$ArchiveName-$stamp.json"
$report = [ordered]@{ time = $now.ToString('o'); archive = $ArchiveName; game = $game; backup = $backup; state = 'backed-up'
    files = @($entries | Select-Object path, sha256, bytes, links) }
$report | ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 -LiteralPath $manifest
foreach ($e in $entries) {
    if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Game started during removal; stop.' }
    if ((Get-FileHash -LiteralPath $e.target -Algorithm SHA256).Hash -ne $e.sha256) { throw "Target changed since backup: $($e.path)" }
    Remove-Item -LiteralPath $e.target
    if (Test-Path -LiteralPath $e.target) { throw "Still present: $($e.path)" }
}
$report.state = 'disabled-backup-hashes-verified'
$report | ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 -LiteralPath $manifest
Write-Output "DISABLED_OK archive=$ArchiveName files=$($entries.Count) backup=$backup manifest=$manifest"

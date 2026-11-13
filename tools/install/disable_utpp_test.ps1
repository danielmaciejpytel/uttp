$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
$game = (Get-UtppGameDir).TrimEnd('\')
if (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue) { throw 'Close the game first' }
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$backup = Join-Path $root "Backups\2026-10-23\UTPP-disabled-$stamp"
$targets = @()
foreach ($rel in @('red4ext\plugins\UTPP','r6\scripts\UTPP')) {
    $p = Join-Path $game $rel
    if (Test-Path -LiteralPath $p) { $targets += Get-Item -LiteralPath $p }
}
$targets += @(Get-ChildItem "$game\bin\x64\plugins\cyber_engine_tweaks\mods" -Directory -Filter 'utpp_*')
$targets += @(Get-ChildItem "$game\archive\pc\mod" -File -Filter 'utpp_*')
$targets += @(Get-ChildItem "$game\r6\input" -File -Filter 'utpp_*.xml' -ErrorAction SilentlyContinue)
$targets += @(Get-ChildItem "$game\r6\tweaks" -File -Recurse -ErrorAction SilentlyContinue | Where-Object {$_.Name -like 'utpp*'})
if (!$targets.Count) { throw 'No active UTPP targets found' }
$files = @($targets | ForEach-Object { if ($_.PSIsContainer) { Get-ChildItem $_.FullName -Recurse -File } else { $_ } })
$entries = @()
foreach ($f in $files) {
    $relative = $f.FullName.Substring($game.Length + 1)
    $dst = Join-Path $backup $relative
    New-Item -ItemType Directory -Force (Split-Path $dst -Parent) | Out-Null
    $hash = (Get-FileHash -LiteralPath $f.FullName -Algorithm SHA256).Hash
    Copy-Item -LiteralPath $f.FullName -Destination $dst
    if ((Get-FileHash -LiteralPath $dst -Algorithm SHA256).Hash -ne $hash) { throw "Backup mismatch: $relative" }
    $entries += @{path=$relative; sha256=$hash; bytes=$f.Length}
}
# Back up all contents before unlinking anything; removal never writes through hardlinks.
foreach ($target in $targets) {
    $resolved = [IO.Path]::GetFullPath($target.FullName)
    if (!$resolved.StartsWith($game + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Target outside game root' }
    Remove-Item -LiteralPath $resolved -Recurse -Force
    if (Test-Path -LiteralPath $resolved) { throw "Still present: $resolved" }
}
$manifest = Join-Path $root "Research\inventory\utpp-disabled-$stamp.json"
@{time=(Get-Date -Format o); backup=$backup; status='disabled-backup-hashes-verified'; targets=@($targets | ForEach-Object {$_.FullName.Substring($game.Length+1)}); files=$entries} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $manifest -Encoding utf8
Write-Output "DISABLED_OK targets=$($targets.Count) files=$($entries.Count) backup=$backup manifest=$manifest"

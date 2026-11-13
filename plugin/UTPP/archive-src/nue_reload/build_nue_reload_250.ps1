# Rebuild utpp_nue_reload.archive on the Cyberpunk 2077 2.5 base graphs (offline; game folder is READ ONLY).
#
# Why: the archive overrides base\gameplay\anim_graphs\player_ranged.animgraph and weapon.animgraph. The copies
# installed until 2026-10-23 were the 2.31 graphs + UTPP gates; on 2.5 the game sends RightReload / EndRightReload /
# RightReloadData / RightShoot / RightCycleRound ... and the stale 2.31 graphs wait for Reload / EndReload / ReloadData,
# so no reload (body or magazine) and part of the weapon graph never runs.
#
# What: re-apply the unchanged UTPP delta (bool gates around the 6 EMPTY + 3 NORMAL native SkSpeedAnim leaves,
# features UTPPNueReload / UTPPNueReloadAim / UTPPNueReload2H) onto the 2.5 retail graphs. The delta never touched
# events, so the 2.5 Right*/Left* events and features are kept as shipped. Right hand only: the gate publishes the
# features only on the weapon in AttachmentSlots.WeaponRight (utpp_reload_gate.reds); the shared 2.5 weapon-graph
# reload states keep the native leaf for any weapon without the feature (left/secondary weapon).
#
# Steps: extract 2.5 graphs (read-only) -> hash gate -> NueGraphPatch body/magazine -> stage raw tree with the
# unchanged custom .anims -> WolvenKit pack -> unbundle + byte/path checks -> object round-trip + JSON round-trip ->
# manifests. Never installs. Re-run deletes and recreates only Build\stages\nue-reload-250\{raw,archive,check,reports,extract}.
param(
    [string]$AnimsDir = 'Extracted\game\anim250\installed_nue_reload\utpp\reload',
    [switch]$SkipJsonRoundTrip,
    # Defaults reproduce the female-only stage exactly as before. The male variant (2026-11-06) is selected with:
    #   -StageName nue-reload-male-20261106 -XlSource <combined xl> -XlSha256 <pin> -MaleAnims <.anims> -MaleAnimsSha256 <pin>
    #   -InventoryPath Research\inventory\nue-reload-male-20261106.json
    [string]$StageName = 'nue-reload-250',
    [string]$XlSource = 'Source\plugin\UTPP\archive\utpp_nue_reload.archive.xl',
    [string]$XlSha256 = '',
    [string]$MaleAnims = '',
    [string]$MaleAnimsSha256 = '',
    [string]$InventoryPath = 'Research\inventory\utpp-nue-reload-250-stage-20261023.json'
)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..\..\..'))
. (Join-Path $PSScriptRoot '..\..\..\..\tools\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\', '/')
$cli = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root "Build\stages\$StageName"
$extract = Join-Path $root 'Extracted\game\anim250'
$toolProj = Join-Path $PSScriptRoot 'tools\NueGraphPatch\NueGraphPatch.csproj'
# Directory.Build.props always builds the patcher into nue-reload-250\toolbuild, whatever the stage name is.
$toolExe = Join-Path $root 'Build\stages\nue-reload-250\toolbuild\bin\Release\net10.0\NueGraphPatch.exe'
$xlSource = Join-Path $root $XlSource
$inventoryOut = Join-Path $root $InventoryPath

# Known inputs (2026-10-23, game patch 2.5). A mismatch of the game exe or of the freshly extracted base graphs
# means the game was patched: the build ABORTS (the archive would otherwise ship stale graphs again, as 2.31 -> 2.5).
# After reviewing the new graphs (delta still applicable, see README.txt) update these values deliberately.
$expected = [ordered]@{
    gameProductVersion = '2.5'
    gameExe         = 'D36E74C47592E9C656AE250078CB40B2BE63C782AF5BFBEFF2280D43E653FAB4'
    vanillaBody25   = 'FCB4BB85300D8534FC683E5C7041A441CA69B62EA548717E7024770971DCD461'
    vanillaWeapon25 = 'A81FBCCEA06F8A3CF3A3C9A8FBC9D70EACF11238774135E8351326F3A84372A3'
    # 2.5 routes the player entity graph 'ranged' to player_dualwield.animgraph (2.31: player_ranged.animgraph). Without
    # this override the authored body reload never plays (2026-10-25 diagnosis: s0 = native Nue clip in 28/28 reloads).
    vanillaDualwield25 = '2DFBF7B61D2ACF506C17B8DB56D79C376E1C127B052750673AB7DF0392576BE5'
    reloadAnims     = 'EFEB52AF243CF21D6F9587CD9A1A6C018EC80678FE8164BEE407E22A92600CBE'
    magazineAnims   = '2380061EAE3CA5F412B40C0E8C39D8E6097076651D773F4127CB605DD43C2015'
    xl              = '1921E2F718F9B33EEA58F459A81AB28D157D9A3B59498AE543F86402D41F22E5'
}

function Assert-UtppUnderRoot([string]$Path) {
    $full = [IO.Path]::GetFullPath($Path)
    if (!$full.StartsWith($root + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Path escapes project root: $full" }
    return $full
}
function Get-UtppSha([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash }
function Assert-UtppHash([string]$Path, [string]$Want) {
    $have = Get-UtppSha $Path
    if ($have -ne $Want) { throw "SHA256 mismatch $Path : $have (expected $Want)" }
}
function Invoke-UtppChecked([string]$Exe, [string[]]$Arguments) {
    # Windows PowerShell 5.1 turns native stderr lines into ErrorRecords; with Stop that would throw on the first
    # warning even when the tool succeeds. Judge success only by the exit code.
    $ErrorActionPreference = 'Continue'
    $log = & $Exe @Arguments 2>&1
    $code = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    $log | ForEach-Object { "$_" } | Add-Content -LiteralPath (Join-Path $out 'reports\build.log') -Encoding utf8
    if ($code -ne 0) { throw "Command failed ($code): $Exe $($Arguments -join ' ')" }
    return $log
}

foreach ($p in @($out, $extract, $xlSource, $inventoryOut)) { [void](Assert-UtppUnderRoot $p) }
if ($MaleAnims) { $MaleAnims = Assert-UtppUnderRoot (Join-Path $root $MaleAnims); if (!$MaleAnimsSha256) { throw 'MaleAnims requires MaleAnimsSha256' } }
$AnimsDir = Assert-UtppUnderRoot (Join-Path $root $AnimsDir)
if (!(Test-Path -LiteralPath $cli)) { throw "Missing WolvenKit CLI: $cli" }

# Clean only this script's own outputs.
foreach ($sub in 'raw', 'archive', 'check', 'reports', 'extract') {
    $p = Join-Path $out $sub
    if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -Recurse -Force }
}
New-Item -ItemType Directory -Force -Path (Join-Path $out 'reports') | Out-Null

# 1. Game version gate, then ALWAYS a fresh read-only extraction of the base graphs into a clean
#    Build\stages\nue-reload-250\extract (never reuse an earlier copy). Every game archive under archive\pc (content, ep1,
#    and any patch folder a future update may add) is searched; every copy found is extracted separately and all
#    copies of one graph must be byte-identical, otherwise the build stops (no guessing of load order).
$exeItem = Get-Item -LiteralPath (Join-Path $game 'bin\x64\Cyberpunk2077.exe')
$exeSha = Get-UtppSha $exeItem.FullName
if ($exeItem.VersionInfo.ProductVersion -ne $expected.gameProductVersion -or $exeSha -ne $expected.gameExe) {
    throw ("GAME CHANGED: Cyberpunk2077.exe ProductVersion={0} SHA256={1}; this build is pinned to {2} / {3}. " +
        "Base graphs may have changed: review them and update `$expected deliberately; the installed archive must be treated as STALE.") -f
        $exeItem.VersionInfo.ProductVersion, $exeSha, $expected.gameProductVersion, $expected.gameExe
}
$freshRoot = Join-Path $out 'extract'
$wanted = [ordered]@{ 'player_ranged.animgraph' = $expected.vanillaBody25; 'player_dualwield.animgraph' = $expected.vanillaDualwield25; 'weapon.animgraph' = $expected.vanillaWeapon25 }
$regex = '^base\\gameplay\\anim_graphs\\(player_ranged|player_dualwield|weapon)\.animgraph$'
$sources = [ordered]@{}
foreach ($name in $wanted.Keys) { $sources[$name] = @() }
$gameArchives = @(Get-ChildItem -LiteralPath (Join-Path $game 'archive\pc') -Recurse -File -Filter '*.archive' |
    Where-Object { $_.FullName -notmatch '\\archive\\pc\\mod\\' } | Sort-Object FullName)
if (!$gameArchives.Count) { throw 'No game archives found under archive\pc.' }
foreach ($a in $gameArchives) {
    $hits = @(Invoke-UtppChecked $cli @('archive', $a.FullName, '-l', '-r', $regex) | ForEach-Object { "$_" } | Where-Object { $_ -match $regex })
    if (!$hits.Count) { continue }
    $dest = Join-Path $freshRoot ($a.Directory.Name + '_' + $a.BaseName)
    [void](Invoke-UtppChecked $cli @('unbundle', $a.FullName, '-o', $dest, '-r', $regex))
    foreach ($hit in $hits) {
        $file = Join-Path $dest $hit
        if (!(Test-Path -LiteralPath $file)) { throw "Listed but not extracted: $hit from $($a.FullName)" }
        $leaf = [IO.Path]::GetFileName($hit)
        $sources[$leaf] += [pscustomobject]@{ archive = $a.FullName.Substring($game.Length + 1); file = $file; sha256 = (Get-UtppSha $file) }
    }
}
foreach ($name in $wanted.Keys) {
    $copies = @($sources[$name])
    if (!$copies.Count) { throw "Base graph $name not found in any game archive." }
    $distinct = @($copies | Select-Object -ExpandProperty sha256 -Unique)
    if ($distinct.Count -ne 1) { throw "Base graph $name differs between game archives ($(($copies | ForEach-Object { $_.archive + '=' + $_.sha256 }) -join ', ')); resolve load order manually." }
    if ($distinct[0] -ne $wanted[$name]) {
        throw "GAME CHANGED: fresh $name SHA256=$($distinct[0]) from $($copies[0].archive) != pinned $($wanted[$name]). The installed utpp_nue_reload archive is STALE; review the new graph before updating `$expected."
    }
}
$body25 = $sources['player_ranged.animgraph'][0].file
$weapon25 = $sources['weapon.animgraph'][0].file
$dual25 = $sources['player_dualwield.animgraph'][0].file
$graphSources = @($sources.Values | ForEach-Object { $_ } | Select-Object archive, sha256)
Assert-UtppHash (Join-Path $AnimsDir 'utpp_nue_tpp_reload.anims') $expected.reloadAnims
Assert-UtppHash (Join-Path $AnimsDir 'utpp_nue_tpp_magazine.anims') $expected.magazineAnims
if (!$XlSha256) { $XlSha256 = $expected.xl }
Assert-UtppHash $xlSource $XlSha256
if ($MaleAnims) { Assert-UtppHash $MaleAnims $MaleAnimsSha256 }

# 2. Patcher (net10, local WolvenKit assemblies).
$toolSources = @(Get-ChildItem -LiteralPath (Split-Path $toolProj -Parent) -File | Where-Object { $_.Extension -in '.cs', '.csproj', '.props' })
$needBuild = !(Test-Path -LiteralPath $toolExe)
if (!$needBuild) {
    $exeTime = (Get-Item -LiteralPath $toolExe).LastWriteTimeUtc
    $needBuild = @($toolSources | Where-Object { $_.LastWriteTimeUtc -gt $exeTime }).Count -gt 0
}
if ($needBuild) {
    $env:DOTNET_CLI_TELEMETRY_OPTOUT = '1'
    [void](Invoke-UtppChecked 'dotnet' @('build', $toolProj, '-c', 'Release'))
}
$env:UTPP_WK_DIR = Join-Path $root 'Tools\WolvenKit.CLI'
$raw = Join-Path $out 'raw\utpp_nue_reload'
$graphDir = Join-Path $raw 'base\gameplay\anim_graphs'
$animDir = Join-Path $raw 'utpp\reload'
New-Item -ItemType Directory -Force -Path $graphDir, $animDir | Out-Null
$rep = Join-Path $out 'reports'
[void](Invoke-UtppChecked $toolExe @('body', $body25, (Join-Path $graphDir 'player_ranged.animgraph'), (Join-Path $rep 'patch-body-250.json')))
[void](Invoke-UtppChecked $toolExe @('body', $dual25, (Join-Path $graphDir 'player_dualwield.animgraph'), (Join-Path $rep 'patch-body-dualwield-250.json')))
[void](Invoke-UtppChecked $toolExe @('magazine', $weapon25, (Join-Path $graphDir 'weapon.animgraph'), (Join-Path $rep 'patch-magazine-250.json')))
Copy-Item -LiteralPath (Join-Path $AnimsDir 'utpp_nue_tpp_reload.anims') -Destination $animDir
Copy-Item -LiteralPath (Join-Path $AnimsDir 'utpp_nue_tpp_magazine.anims') -Destination $animDir
if ($MaleAnims) { Copy-Item -LiteralPath $MaleAnims -Destination (Join-Path $animDir 'utpp_nue_tpp_reload_man.anims') }

# 3. Pack + .xl (content unchanged; ArchiveXL wiring verified on 2.5 by the 2026-10-23 ArchiveXL log).
$mod = Join-Path $out 'archive\pc\mod'
New-Item -ItemType Directory -Force -Path $mod | Out-Null
[void](Invoke-UtppChecked $cli @('pack', $raw, '-o', $mod))
$archive = Join-Path $mod 'utpp_nue_reload.archive'
if (!(Test-Path -LiteralPath $archive)) { throw "Pack produced no archive: $archive" }
Copy-Item -LiteralPath $xlSource -Destination ($archive + '.xl')

# 4. Unbundle: same 4 resource paths as the installed archive, byte-identical payloads.
$check = Join-Path $out 'check\unbundle'
[void](Invoke-UtppChecked $cli @('unbundle', $archive, '-o', $check))
$wantPaths = @('base\gameplay\anim_graphs\player_ranged.animgraph', 'base\gameplay\anim_graphs\player_dualwield.animgraph','base\gameplay\anim_graphs\weapon.animgraph',
    'utpp\reload\utpp_nue_tpp_magazine.anims', 'utpp\reload\utpp_nue_tpp_reload.anims')
if ($MaleAnims) { $wantPaths += 'utpp\reload\utpp_nue_tpp_reload_man.anims' }
$gotPaths = @(Get-ChildItem -LiteralPath $check -Recurse -File | ForEach-Object { $_.FullName.Substring($check.Length + 1) } | Sort-Object)
if (($gotPaths -join '|') -ne (($wantPaths | Sort-Object) -join '|')) { throw "Resource path list differs: $($gotPaths -join ', ')" }
foreach ($rel in $wantPaths) { Assert-UtppHash (Join-Path $check $rel) (Get-UtppSha (Join-Path $raw $rel)) }
$listing = @(Invoke-UtppChecked $cli @('archive', $archive, '-l') | ForEach-Object { "$_" } | Where-Object { $_ -match '^(base|utpp)\\' } | Sort-Object)
if (($listing -join '|') -ne (($wantPaths | Sort-Object) -join '|')) { throw 'archive -l listing differs from expected resource paths' }

# 5. Object round-trip of the packed graphs (re-read -> re-write must be byte-identical, 0 differences).
foreach ($name in 'player_ranged', 'player_dualwield', 'weapon') {
    $src = Join-Path $check "base\gameplay\anim_graphs\$name.animgraph"
    [void](Invoke-UtppChecked $toolExe @('roundtrip', $src, (Join-Path $out "check\roundtrip-$name.animgraph"), (Join-Path $rep "roundtrip-$name.json"), '--expect-sha', (Get-UtppSha $src)))
}

# 6. WolvenKit JSON round-trip (serialize -> deserialize) + deep object comparison against the packed graph.
if (!$SkipJsonRoundTrip) {
    $jsonDir = Join-Path $out 'check\json'
    $backDir = Join-Path $out 'check\json-back'
    New-Item -ItemType Directory -Force -Path $jsonDir, $backDir | Out-Null
    foreach ($name in 'player_ranged', 'player_dualwield', 'weapon') {
        $src = Join-Path $check "base\gameplay\anim_graphs\$name.animgraph"
        [void](Invoke-UtppChecked $cli @('convert', 'serialize', $src, '-o', $jsonDir))
        [void](Invoke-UtppChecked $cli @('convert', 'deserialize', (Join-Path $jsonDir "$name.animgraph.json"), '-o', $backDir))
        $back = Join-Path $backDir "$name.animgraph"
        if (!(Test-Path -LiteralPath $back)) { throw "JSON deserialize produced no file for $name" }
        [void](Invoke-UtppChecked $toolExe @('compare', $src, $back, (Join-Path $rep "json-roundtrip-$name.json")))
    }
}

# 7. Manifests (stage manifest for install_verified_stage.ps1 + build record).
$files = @(
    [ordered]@{ path = 'archive\pc\mod\utpp_nue_reload.archive'; sha256 = (Get-UtppSha $archive) },
    [ordered]@{ path = 'archive\pc\mod\utpp_nue_reload.archive.xl'; sha256 = (Get-UtppSha ($archive + '.xl')) }
)
$payload = foreach ($rel in $wantPaths) { [ordered]@{ path = $rel; sha256 = (Get-UtppSha (Join-Path $raw $rel)); bytes = (Get-Item -LiteralPath (Join-Path $raw $rel)).Length } }
$record = [ordered]@{
    createdAt = [DateTimeOffset]::Now.ToString('o'); game = [ordered]@{ productVersion = $exeItem.VersionInfo.ProductVersion; fileVersion = $exeItem.VersionInfo.FileVersion; exeSha256 = $exeSha }; inputs = $expected; graphSources = $graphSources
    files = $files; payload = @($payload); runtimeVerified = $false; installed = $false
    note = ('Offline build only. Install with Source\tools\install\install_verified_stage.ps1 -StageName ' + $StageName + ' -ManifestName ' + (Split-Path $inventoryOut -Leaf) + ' (dry run first, -Apply only with the game closed).')
}
$record | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath (Join-Path $out 'manifest.json')
New-Item -ItemType Directory -Force -Path (Split-Path $inventoryOut -Parent) | Out-Null
if ($MaleAnims) {
    [ordered]@{ createdAt = $record.createdAt; stage = ('Build\stages\' + $StageName); files = $files; payload = @($payload); runtimeVerified = $false
        note = 'Female + male authored Nue reload archive (one archive + one combined .xl, replaces the female-only utpp_nue_reload). Gates stay female-only until the male gate opening (reds/CET/DLL).'; sourceCommit = 'uncommitted' } |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 -LiteralPath $inventoryOut
} else {
    [ordered]@{ createdAt = $record.createdAt; stage = ('Build\stages\' + $StageName); files = $files; runtimeVerified = $false } |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 -LiteralPath $inventoryOut
}
Write-Output "BUILD_OK archive=$archive sha256=$($files[0].sha256)"
Write-Output 'Runtime (reload animation, magazine, ArchiveXL merge, gate) is NOT verified; nothing was installed.'

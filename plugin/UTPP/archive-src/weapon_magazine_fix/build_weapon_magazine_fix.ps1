# Build utpp_weapon_magazine_fix.archive on the Cyberpunk 2077 2.5 game files (offline; game folder is READ ONLY).
#
# Why: during a reload the weapon animation parks the old/spare magazine away from the gun and hides it through the
# rig track showMagazine / showMagazineReload (Left/Right on the Palica). The magazine mesh is bound to that track
# (entSkinnedMeshComponent.visibilityAnimationParam), but several decal meshes of the same magazine are not
# (visibilityAnimationParam = None), so in third person the decal shell (e.g. the Sidewinder flag) floats next to V.
#
# What: set visibilityAnimationParam "None" -> the parameter of the animated component (Magazine / MagazineReload)
# the decal is skinned to, which the sibling magazine mesh already uses, on exactly the components pinned in
# targets.json, in every serialized copy (appearance components + compiledData chunks). Nothing else changes.
#
# Which files: on 2.5 the player weapon is built from the weapon ROOT appearance resource (e.g.
# nokota_sidewinder\w_rifle_assault__nokota_sidewinder.app, appearance divided_ws for Items.mq008_golden_knuckledusters),
# which carries its own inline copies of the magazine components. The first build (bd417a1, 2026-10-27) patched only
# the magazine part .app/.ent files, so the player weapon still had "None" and the flag kept floating (recording
# 2026-10-28 00-20-31). The weapon root .app files are now pinned too; the part files stay patched for consistency.
# v3 (2026-10-28): files whose content\ and ep1\ copies differ (Kyubi, Kyubi Myers, Kenshin, Nova) are patched from the
# ep1 copy, pinned as winner only when it is a strict superset of the content copy (see gen_targets.py): the mod
# archive overrides the depot path, so one override replaces whichever copy the game would load and keeps every content
# component. Phantom Liberty weapons (Metel, Rasetsu) are included after checking their ep1\animations\weapon reload sets.
#
# Steps: exe gate -> fresh read-only extraction from every game archive (the set of copies and every copy's SHA256 must
# match targets.json; the pinned winner copy is patched) -> WolvenKit JSON -> control round-trip (vanilla JSON -> CR2W -> JSON must be identical) -> python patch ->
# CR2W -> JSON round-trip diff (only the pinned leaves changed) -> pack -> unbundle path/byte check -> manifests.
# Never installs. Re-run deletes and recreates only Build\stages\weapon-decal-fix\{extract,vanilla,control,patched,raw,archive,check,reports}.
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..\..\..'))
. (Join-Path $PSScriptRoot '..\..\..\..\tools\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\', '/')
$cli = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Build\stages\weapon-decal-fix'
$targetsPath = Join-Path $PSScriptRoot 'targets.json'
$patcher = Join-Path $PSScriptRoot 'patch_magazine_visibility.py'
$archiveName = 'utpp_weapon_magazine_fix.archive'
# 20261027 = bd417a1 (part files), 20261028 = 3978d36 (installed 2026-10-28 01:13, archive 20FD2672...), -v3 = this build.
$manifestName = 'utpp-weapon-magazine-fix-stage-20261028-v3.json'

# Known inputs (2026-10-28, game patch 2.5). A different exe or a different freshly extracted file means the game was
# patched: the build ABORTS. Re-derive targets.json (scan_mag_visibility.py -> check_show_tracks.py -> gen_targets.py) and review it before re-pinning.
$expected = [ordered]@{
    gameProductVersion = '2.5'
    gameExe = 'D36E74C47592E9C656AE250078CB40B2BE63C782AF5BFBEFF2280D43E653FAB4'
    targets = 'E6D1E296D35386D0118A387E4AC84C82B64F133E6BE6EB95C1AAF02098AE3B5A'
    patcher = '8FE2F5CF61BB2A813348CE7F411532A96C22ABAA02E652818C542F785B593569'
    # SHA256 over the sorted "depot path=SHA256" lines of the 75 packed CR2W files. The payload is byte-reproducible;
    # the .archive container itself is not (WolvenKit pack writes per-run index metadata), so the archive SHA is
    # recorded in the manifest but the payload is what is pinned.
    payload = 'D5C105B68BA11CC63EE1D13484BCC5E626FA5A497C9C636E02C89E4E40BD70CB'
}

function Assert-UtppUnderRoot([string]$Path) {
    $full = [IO.Path]::GetFullPath($Path)
    if (!$full.StartsWith($root + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Path escapes project root: $full" }
    return $full
}
function Get-UtppSha([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash }
function Invoke-UtppChecked([string]$Exe, [string[]]$Arguments) {
    # Windows PowerShell 5.1 turns native stderr lines into ErrorRecords; judge success only by the exit code.
    $ErrorActionPreference = 'Continue'
    $log = & $Exe @Arguments 2>&1
    $code = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    $log | ForEach-Object { "$_" } | Add-Content -LiteralPath (Join-Path $out 'reports\build.log') -Encoding utf8
    if ($code -ne 0) { throw "Command failed ($code): $Exe $($Arguments -join ' ')" }
    return $log
}
function Get-UtppJsonFiles([string]$Dir) {
    @(Get-ChildItem -LiteralPath $Dir -Recurse -File -Filter '*.json' | ForEach-Object { $_.FullName.Substring($Dir.Length + 1) } | Sort-Object)
}

[void](Assert-UtppUnderRoot $out)
if (!(Test-Path -LiteralPath $cli)) { throw "Missing WolvenKit CLI: $cli" }
$python = (Get-Command python -ErrorAction Stop).Source
foreach ($sub in 'extract', 'vanilla', 'control', 'patched', 'raw', 'archive', 'check', 'reports') {
    $p = Join-Path $out $sub
    if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -Recurse -Force }
}
New-Item -ItemType Directory -Force -Path (Join-Path $out 'reports') | Out-Null

# 1. Game version gate + pinned build inputs.
$exeItem = Get-Item -LiteralPath (Join-Path $game 'bin\x64\Cyberpunk2077.exe')
$exeSha = Get-UtppSha $exeItem.FullName
if ($exeItem.VersionInfo.ProductVersion -ne $expected.gameProductVersion -or $exeSha -ne $expected.gameExe) {
    throw ("GAME CHANGED: Cyberpunk2077.exe ProductVersion={0} SHA256={1}; this build is pinned to {2} / {3}. Re-run scan_mag_visibility.py -> " +
        "check_show_tracks.py -> gen_targets.py, review, then re-pin targets.json; the installed archive must be treated as STALE.") -f $exeItem.VersionInfo.ProductVersion, $exeSha, $expected.gameProductVersion, $expected.gameExe
}
if ((Get-UtppSha $targetsPath) -ne $expected.targets) { throw "targets.json changed: review it and update the pinned hash deliberately." }
if ((Get-UtppSha $patcher) -ne $expected.patcher) { throw "patch_magazine_visibility.py changed: review it and update the pinned hash deliberately." }
$targets = Get-Content -Raw -LiteralPath $targetsPath | ConvertFrom-Json
$wantPaths = @($targets.files | ForEach-Object { $_.path } | Sort-Object)
if (!$wantPaths.Count) { throw 'targets.json lists no files.' }

# 2. Fresh read-only extraction from every game archive under archive\pc (mods excluded). The copies found (archive +
#    SHA256) must equal the reviewed set pinned in targets.json (copies[]), so a game patch to ANY copy aborts; the
#    pinned winner copy (archive/sha256) is the one patched.
$regex = '^(' + (($wantPaths | ForEach-Object { [regex]::Escape($_) }) -join '|') + ')$'
$copies = @{}
foreach ($p in $wantPaths) { $copies[$p] = @() }
$gameArchives = @(Get-ChildItem -LiteralPath (Join-Path $game 'archive\pc') -Recurse -File -Filter '*.archive' |
    Where-Object { $_.FullName -notmatch '\\archive\\pc\\mod\\' } | Sort-Object FullName)
if (!$gameArchives.Count) { throw 'No game archives found under archive\pc.' }
foreach ($a in $gameArchives) {
    $hits = @(Invoke-UtppChecked $cli @('archive', $a.FullName, '-l', '-r', $regex) | ForEach-Object { "$_" } | Where-Object { $_ -match $regex })
    if (!$hits.Count) { continue }
    $dest = Join-Path $out ('extract\' + $a.Directory.Name + '_' + $a.BaseName)
    [void](Invoke-UtppChecked $cli @('unbundle', $a.FullName, '-o', $dest, '-r', $regex))
    foreach ($hit in $hits) {
        $file = Join-Path $dest $hit
        if (!(Test-Path -LiteralPath $file)) { throw "Listed but not extracted: $hit from $($a.FullName)" }
        $copies[$hit] += [pscustomobject]@{ archive = $a.FullName.Substring($game.Length + 1); file = $file; sha256 = (Get-UtppSha $file) }
    }
}
$vanilla = Join-Path $out 'vanilla'
$sources = foreach ($t in $targets.files) {
    $c = @($copies[$t.path])
    if (!$c.Count) { throw "Target not found in any game archive: $($t.path)" }
    if (!$t.copies) { throw "targets.json entry without copies[]: $($t.path). Re-generate it with gen_targets.py." }
    $got = (@($c | ForEach-Object { $_.archive + '=' + $_.sha256 }) | Sort-Object) -join ', '
    $pin = (@($t.copies | ForEach-Object { $_.archive + '=' + $_.sha256 }) | Sort-Object) -join ', '
    if ($got -ne $pin) { throw "GAME CHANGED: $($t.path) copies [$got] != pinned [$pin]. Re-derive targets.json (scan -> tracks -> gen_targets.py)." }
    $win = @($c | Where-Object { $_.archive -eq $t.archive })
    if ($win.Count -ne 1 -or $win[0].sha256 -ne $t.sha256) { throw "Pinned winner $($t.archive)=$($t.sha256) not found for $($t.path)." }
    $dst = Join-Path $vanilla $t.path
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Copy-Item -LiteralPath $win[0].file -Destination $dst
    [ordered]@{ path = $t.path; archives = @($c | ForEach-Object { $_.archive }); winner = $t.archive; sha256 = $t.sha256; winnerReason = $t.winnerReason }
}

# 3. Vanilla JSON (in place) + control round-trip: vanilla JSON -> CR2W -> JSON must be identical (WolvenKit re-encodes
#    the binary, so bytes may differ; the object content may not). Without this the patched diff would be meaningless.
[void](Invoke-UtppChecked $cli @('convert', 'serialize', $vanilla, '-r', '\.(app|ent)$'))
$control = Join-Path $out 'control'
foreach ($rel in (Get-UtppJsonFiles $vanilla)) {
    $dst = Join-Path $control $rel
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Copy-Item -LiteralPath (Join-Path $vanilla $rel) -Destination $dst
}
[void](Invoke-UtppChecked $cli @('convert', 'deserialize', $control, '-r', '\.json$'))
[void](Invoke-UtppChecked $cli @('convert', 'serialize', $control, '-r', '\.(app|ent)$'))
# A "verify" run against an unpatched round-trip must find zero changed leaves; reuse the verifier with a zero-hit spec.
$zeroSpec = Join-Path $out 'reports\control-spec.json'
$zero = [ordered]@{ files = @($targets.files | ForEach-Object { [ordered]@{ path = $_.path; expectedHits = 0; components = @($_.components) } }) }
$zero | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $zeroSpec
[void](Invoke-UtppChecked $python @('-I', $patcher, 'verify', $zeroSpec, $vanilla, $control, (Join-Path $out 'reports\control-roundtrip.json')))

# 4. Patch JSON -> CR2W.
$patched = Join-Path $out 'patched'
[void](Invoke-UtppChecked $python @('-I', $patcher, 'patch', $targetsPath, $vanilla, $patched, (Join-Path $out 'reports\patch.json')))
[void](Invoke-UtppChecked $cli @('convert', 'deserialize', $patched, '-r', '\.json$'))
$raw = Join-Path $out 'raw\utpp_weapon_magazine_fix'
foreach ($p in $wantPaths) {
    $bin = Join-Path $patched $p
    if (!(Test-Path -LiteralPath $bin)) { throw "Deserialize produced no file: $p" }
    $dst = Join-Path $raw $p
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Copy-Item -LiteralPath $bin -Destination $dst
}

# 5. Pack, unbundle, compare resource paths and bytes.
$mod = Join-Path $out 'archive\pc\mod'
New-Item -ItemType Directory -Force -Path $mod | Out-Null
[void](Invoke-UtppChecked $cli @('pack', $raw, '-o', $mod))
$archive = Join-Path $mod $archiveName
if (!(Test-Path -LiteralPath $archive)) { throw "Pack produced no archive: $archive" }
$check = Join-Path $out 'check\unbundle'
[void](Invoke-UtppChecked $cli @('unbundle', $archive, '-o', $check))
$gotPaths = @(Get-ChildItem -LiteralPath $check -Recurse -File | ForEach-Object { $_.FullName.Substring($check.Length + 1) } | Sort-Object)
if (($gotPaths -join '|') -ne ($wantPaths -join '|')) { throw "Resource path list differs: $($gotPaths -join ', ')" }
foreach ($p in $wantPaths) {
    if ((Get-UtppSha (Join-Path $check $p)) -ne (Get-UtppSha (Join-Path $raw $p))) { throw "Unbundled bytes differ: $p" }
}
$listing = @(Invoke-UtppChecked $cli @('archive', $archive, '-l') | ForEach-Object { "$_" } | Where-Object { $_ -match '^(base|ep1)\\' } | Sort-Object)
if (($listing -join '|') -ne ($wantPaths -join '|')) { throw 'archive -l listing differs from expected resource paths' }

# 6. Packed CR2W -> JSON; only the pinned visibilityAnimationParam leaves may differ from vanilla, exactly expectedHits each.
$rtJson = Join-Path $out 'check\json'
foreach ($p in $wantPaths) {
    $dst = Join-Path $rtJson $p
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Copy-Item -LiteralPath (Join-Path $check $p) -Destination $dst
}
[void](Invoke-UtppChecked $cli @('convert', 'serialize', $rtJson, '-r', '\.(app|ent)$'))
$verify = @(Invoke-UtppChecked $python @('-I', $patcher, 'verify', $targetsPath, $vanilla, $rtJson, (Join-Path $out 'reports\verify-roundtrip.json')))
$totalHits = ($targets.files | Measure-Object -Property expectedHits -Sum).Sum
if (!($verify | Where-Object { "$_" -eq "VERIFY OK files=$($wantPaths.Count) changed=$totalHits" })) { throw "Verify summary mismatch: $($verify -join ' ')" }

# 7. Manifests (stage manifest for install_verified_stage.ps1 + build record).
$files = @([ordered]@{ path = "archive\pc\mod\$archiveName"; sha256 = (Get-UtppSha $archive) })
$payload = foreach ($p in $wantPaths) { [ordered]@{ path = $p; sha256 = (Get-UtppSha (Join-Path $raw $p)); bytes = (Get-Item -LiteralPath (Join-Path $raw $p)).Length } }
$payloadLines = ($payload | Sort-Object { $_.path } | ForEach-Object { "$($_.path)=$($_.sha256)" }) -join "`n"
$payloadSha = (([Security.Cryptography.SHA256]::Create().ComputeHash([Text.Encoding]::UTF8.GetBytes($payloadLines))) | ForEach-Object { $_.ToString('X2') }) -join ''
if ($payloadSha -ne $expected.payload) { throw "Payload SHA256 $payloadSha != pinned $($expected.payload): patched content changed; review reports\ and re-pin deliberately." }
$record = [ordered]@{
    createdAt = [DateTimeOffset]::Now.ToString('o'); game = [ordered]@{ productVersion = $exeItem.VersionInfo.ProductVersion; fileVersion = $exeItem.VersionInfo.FileVersion; exeSha256 = $exeSha }
    inputs = $expected; field = 'visibilityAnimationParam'; changedLeaves = $totalHits; sources = @($sources); excluded = @($targets.excluded)
    files = $files; payload = @($payload); runtimeVerified = $false; installed = $false
    note = "Offline build only. Install with Source\tools\install\install_verified_stage.ps1 -StageName weapon-decal-fix -ManifestName $manifestName (dry run first, -Apply only with the game closed)."
}
$record | ConvertTo-Json -Depth 8 | Set-Content -Encoding utf8 -LiteralPath (Join-Path $out 'manifest.json')
[ordered]@{ createdAt = $record.createdAt; stage = 'Build\stages\weapon-decal-fix'; files = $files; runtimeVerified = $false } |
    ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 -LiteralPath (Join-Path $root "Research\inventory\$manifestName")
Write-Output "BUILD_OK archive=$archive sha256=$($files[0].sha256) files=$($wantPaths.Count) changedLeaves=$totalHits"
Write-Output 'Runtime (decal hidden with the magazine during reload, no other visual change) is NOT verified; nothing was installed.'

<#
.SYNOPSIS
  Test stage of the NPC-based third-person fists (UTPP melee, npc1 pack): UTPP.dll, the melee scripts + the CET core module,
  utpp_melee.archive + the composed utpp_melee_tpp.archive.xl, optionally the run-start lean motion matching databases.

.DESCRIPTION
  Writes Build\stages\<StageName>\ (game-relative layout; real copies, never hardlinks; SHA-256 of every copy verified) and the
  manifest Research\inventory\<StageName>.json in the format install_verified_stage.ps1 reads (files[].path / sha256 / bytes).
  Never writes to the game folder and never runs the installer. -DryRun runs every check and prints the plan; it writes nothing.

  Staged (the files whose function differs from the installed melee pilot; other runtime files are only reported):
    red4ext\plugins\UTPP\UTPP.dll                               Build\UTPP\package (= Build\UTPP\UTPP.dll)
    r6\scripts\UTPP\utpp_melee.reds, utpp_body.reds, utpp_modsettings.reds          Build\UTPP\package (= Source\plugin\UTPP\scripts)
    bin\x64\plugins\cyber_engine_tweaks\mods\utpp_core\init.lua Build\UTPP\package (= Source\plugin\UTPP\cet)
    archive\pc\mod\utpp_melee.archive                           -Archive (default <PackDir>\packed\utpp_melee.archive)
    archive\pc\mod\utpp_melee_tpp.archive.xl                    -Xl (default <PackDir>\packed\utpp_melee_tpp.archive.xl)
    -ArchiveOnly: only the two archive files (no DLL / scripts / CET; the script / state-table checks below still read the package scripts)
    -WithRunStartLean: the 4 files of Build\stages\runstart-lean-20261109 (v1 db + .airz, v2 woman + man), all or none
    archive\pc\mod\utpp_melee_parried_graph.archive             -GraphArchive (optional; the per-strike parried graph, see "graph" below)
  The stage REPLACES the installed utpp_melee.archive / .xl: sets that are
  not in the new archive (e.g. the male fists of a female-only pack) go back to the vanilla clips.
  The parried graph archive is a NEW file (not in the game): it replaces base\gameplay\anim_graphs\player_melee.animgraph through the
  archive load order. Uninstall = Source\tools\install\disable_utpp_archive.ps1 -ArchiveName utpp_melee_parried_graph (backup + remove).

  Checks (any FAIL = nothing written, exit 1):
    pack      run manifest (<PackDir>\manifest.json) has 'archive' with ok = true; the archive's SHA-256 / size equal the manifest;
              every set of the .xl was read back equal from the archive (archive.unbundle_compare); every entry gates on
              utpp_foot_tpp
    coverage  every anim variable of UTPPMeleeFullBodyCoveredName (staged utpp_melee.reds) has an .xl entry - otherwise that weapon
              takes the full body under its vanilla first-person clip (static legs): FAIL (-AllowCoverageMismatch: WARN);
              a variable registered for one rig only: FAIL too (the script has no rig check; -AllowCoverageMismatch: WARN)
    states    EVERY line of the UTPP_STATES_TABLE (utpp_melee.reds; a line the parser does not understand = FAIL) against the
              coverage of the pack; the sets of a line are the .xl entries gated on its weapon variable
              (-Coverage, default <PackDir>\coverage.json, else <PackDir>\coverage_fists.txt; for a .txt the .json of the same name
              is preferred when present): a state bit whose clips stay vanilla = FAIL (ground slam clips under JumpAttack 18: WARN,
              state not confirmed); states whose clips are all third-person but not in the mask are listed (INFO)
    graph     a line with the Parried bit (32) whose sets hold per-strike parried clips (parried_(left|right)_<suffix>; plain
              parried_left / parried_right do not count: the fists have only those) REQUIRES -GraphArchive with the pinned SHA-256
              (D5786BB1..., run5 of Build\work\melee-parried-graph-20261110): else FAIL (-AllowCoverageMismatch does not
              relax this). Clip names come from the coverage.json (redone + fallback clips) and <PackDir>\reports\build-<set>.json.
              A given graph archive is verified (SHA-256 = the pin, run manifest archiveSha256 / vanillaSha256 BFB4D765... /
              listing = player_melee.animgraph only); staged without need = WARN. Without the graph keep the Parried bit off
              (knife 3668160, katana 3669440); with it knife 3668192 (+32), katana 3669472.
    package   staged scripts / CET equal Source (stale package = FAIL: run Build\build-main.cmd); package DLL = Build\UTPP\UTPP.dll
    dbs       (-WithRunStartLean) every file matches Research\inventory\runstart-lean-20261109.json
    targets   every staged path passes the target allow-list of install_verified_stage.ps1 (copied below; keep in sync)

  Usage (from the project root):
    powershell -NoProfile -ExecutionPolicy Bypass -File Source\tools\install\stage_melee_npc.ps1 -DryRun
    powershell -NoProfile -ExecutionPolicy Bypass -File Source\tools\install\stage_melee_npc.ps1 [-Xl <composed .xl>] [-WithRunStartLean]
  then (lead, game closed): install_verified_stage.ps1 -StageName <name> -ManifestName <name>.json   (preflight), + -Apply
#>
param(
    [string]$PackDir = 'Build\work\melee-export-20261109\runs\npc1-fists',
    [string]$Archive = '',
    [string]$Xl = '',
    [string]$Coverage = '',
    [switch]$WithRunStartLean,
    [switch]$AllowCoverageMismatch,
    [switch]$ArchiveOnly,         # stage only the two archive files (no DLL / scripts / CET): the package may hold unrelated, untested changes
    [string]$GraphArchive = '',   # optional: utpp_melee_parried_graph.archive (required when a states line has Parried + per-strike parried clips)
    [string]$GraphSha256 = 'D5786BB159CEC4AD8A334B15B6762785A5B587E1DF0664A79DE91F0CFDC87449',   # expected SHA-256; must equal the pin below
    [string]$MeleeReds = '',      # script tests only: read the state table / coverage lists from this file instead of Build\UTPP\package
    [switch]$DryRun,
    [string]$StageName = '',
    [string]$StageRoot = '',      # default <project>\Build\stages (install_verified_stage.ps1 reads only there; other roots = script tests)
    [string]$InventoryDir = '',   # default <project>\Research\inventory (install_verified_stage.ps1 reads the manifest there)
    [string]$Note = ''
)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$game = (Get-UtppGameDir).TrimEnd('\')
$vortex = Get-UtppLocalPath -Key 'mod_manager_dir'
if (-not $vortex) { throw 'The mod manager folder is not configured: set UTPP_MOD_MANAGER_DIR or mod_manager_dir in local_paths.json (the stage must never be written there).' }
$vortex = $vortex.TrimEnd('\')
# target allow-list of install_verified_stage.ps1 (line 26), copied verbatim
$allowRx = '^(red4ext\\plugins\\UTPP\\|red4ext\\plugins\\UTPPTestHost\\UTPPTestHost\.dll$|r6\\scripts\\UTPP\\|r6\\input\\utpp_shoulder\.xml$|bin\\x64\\plugins\\cyber_engine_tweaks\\mods\\utpp_|archive\\pc\\mod\\utpp_camera\.archive|archive\\pc\\mod\\utpp_female_tpp_nails_plane\.archive$|archive\\pc\\mod\\utpp_nue_reload\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_foot_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_vehicle_entry\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_bike\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_vehicle_car\.archive$|archive\\pc\\mod\\utpp_weapon_magazine_fix\.archive$|archive\\pc\\mod\\utpp_melee\.archive$|archive\\pc\\mod\\utpp_melee_tpp\.archive\.xl$|archive\\pc\\mod\\utpp_ragdoll_poc\.archive(?:\.xl)?$|archive\\pc\\mod\\utpp_ragdoll_graph\.archive$|archive\\pc\\mod\\utpp_melee_parried_graph\.archive$)'
$graphRel = 'archive\pc\mod\utpp_melee_parried_graph.archive'
# pinned: Build\work\melee-parried-graph-20261110\run5 (patched copy of the 2.5 player_melee.animgraph; keep in sync with make_release.py)
$graphPinSha = 'D5786BB159CEC4AD8A334B15B6762785A5B587E1DF0664A79DE91F0CFDC87449'
$graphVanillaSha = 'BFB4D7651E761B08FCE0FEA08767AF8F9A9D9321820F4A4DC13E412A7D445AB0'
$graphListing = 'base\gameplay\anim_graphs\player_melee.animgraph'

$script:fails = New-Object System.Collections.Generic.List[string]
$script:warns = New-Object System.Collections.Generic.List[string]
function Fail([string]$m) { $script:fails.Add($m); Write-Output "FAIL  $m" }
function Warn([string]$m) { $script:warns.Add($m); Write-Output "WARN  $m" }
function Info([string]$m) { Write-Output "INFO  $m" }
function Sha([string]$p) { return (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash }
function FullPath([string]$p) {
    if ([IO.Path]::IsPathRooted($p)) { return [IO.Path]::GetFullPath($p) }
    return [IO.Path]::GetFullPath((Join-Path $root $p))
}
function Prop($o, [string]$n) {
    if ($null -eq $o) { return $null }
    $p = $o.PSObject.Properties[$n]
    if ($null -eq $p) { return $null }
    return $p.Value
}
function RigOf([string]$setFile) { if ($setFile -match '^utpp_pma_') { return 'man' } return 'woman' }
function ReadJson([string]$p) { return (Get-Content -Raw -Encoding UTF8 -LiteralPath $p | ConvertFrom-Json) }

# ------------------------------------------------------------------ names and paths
if (-not $StageName) { $StageName = 'melee-npc1-' + (Get-Date).ToString('yyyyMMdd-HHmm') }
if ($StageName -notmatch '^[A-Za-z0-9_-]+$') { throw "Unsafe stage name: $StageName" }
$manifestName = $StageName + '.json'
$stageRootPath = if ($StageRoot) { FullPath $StageRoot } else { Join-Path $root 'Build\stages' }
$invPath = if ($InventoryDir) { FullPath $InventoryDir } else { Join-Path $root 'Research\inventory' }
$stage = Join-Path $stageRootPath $StageName
$manPath = Join-Path $invPath $manifestName
foreach ($p in @($stage, $invPath)) {
    foreach ($bad in @($game, $vortex)) {
        if ($p.StartsWith($bad, [StringComparison]::OrdinalIgnoreCase)) { throw "Refusing to write under $bad" }
    }
}
$pack = FullPath $PackDir
$Archive = if ($Archive) { FullPath $Archive } else { Join-Path $pack 'packed\utpp_melee.archive' }
$Xl = if ($Xl) { FullPath $Xl } else { Join-Path $pack 'packed\utpp_melee_tpp.archive.xl' }
if ($Coverage) { $Coverage = FullPath $Coverage }
else {
    $Coverage = Join-Path $pack 'coverage.json'   # coverage.py format: sets.p?a_fists redone_clips / not_built_clips / ...
    if (!(Test-Path -LiteralPath $Coverage)) { $Coverage = Join-Path $pack 'coverage_fists.txt' }
}
$pkg = Join-Path $root 'Build\UTPP\package'
$src = Join-Path $root 'Source\plugin\UTPP'
Write-Output "stage_melee_npc: stage $stage$(if ($DryRun) { ' (dry run: nothing is written)' })"
Write-Output "  pack $pack"
Write-Output "  archive $Archive"
Write-Output "  xl $Xl"
Write-Output "  coverage $Coverage"
if ($GraphArchive) { $GraphArchive = FullPath $GraphArchive; Write-Output "  graph archive $GraphArchive" }
if ($MeleeReds) { $MeleeReds = FullPath $MeleeReds; Warn "-MeleeReds ${MeleeReds}: the state table and coverage lists are read from this file (script tests), not from the staged package" }
if ($StageRoot) { Warn "-StageRoot ${stageRootPath}: install_verified_stage.ps1 reads stages from Build\stages only" }
if ($InventoryDir) { Warn "-InventoryDir ${invPath}: install_verified_stage.ps1 reads manifests from Research\inventory only" }

$entries = New-Object System.Collections.Generic.List[object]
function AddEntry([string]$rel, [string]$source, [string]$sha) {
    $entries.Add([pscustomobject]@{ path = $rel; source = $source; sha256 = $sha; bytes = (Get-Item -LiteralPath $source).Length })
}

# ------------------------------------------------------------------ package: DLL, scripts, CET (built by Build\build-main.cmd)
if ($ArchiveOnly) { Info '-ArchiveOnly: DLL, scripts and CET are not staged (the installed ones stay)' }
else {
$pkgDll = Join-Path $pkg 'red4ext\plugins\UTPP\UTPP.dll'
$buildDll = Join-Path $root 'Build\UTPP\UTPP.dll'
if (!(Test-Path -LiteralPath $pkgDll) -or !(Test-Path -LiteralPath $buildDll)) {
    Fail "DLL missing: $pkgDll / $buildDll (run Build\build-main.cmd)"
} else {
    $h = Sha $pkgDll
    if ($h -ne (Sha $buildDll)) { Fail "package DLL differs from Build\UTPP\UTPP.dll (stale package: run Build\build-main.cmd)" }
    else { AddEntry 'red4ext\plugins\UTPP\UTPP.dll' $pkgDll $h; Info "UTPP.dll SHA-256 $h" }
}
$code = @(
    @('r6\scripts\UTPP\utpp_melee.reds', 'scripts\utpp_melee.reds'),
    @('r6\scripts\UTPP\utpp_body.reds', 'scripts\utpp_body.reds'),
    @('r6\scripts\UTPP\utpp_modsettings.reds', 'scripts\utpp_modsettings.reds'),
    @('bin\x64\plugins\cyber_engine_tweaks\mods\utpp_core\init.lua', 'cet\utpp_core\init.lua')
)
foreach ($c in $code) {
    $p = Join-Path $pkg $c[0]
    $s = Join-Path $src $c[1]
    if (!(Test-Path -LiteralPath $p)) { Fail "$($c[0]) is not in the package (run Build\build-main.cmd)"; continue }
    $h = Sha $p
    if (!(Test-Path -LiteralPath $s) -or $h -ne (Sha $s)) { Fail "$($c[0]): package copy differs from Source\plugin\UTPP\$($c[1]) (stale package: run Build\build-main.cmd)"; continue }
    AddEntry $c[0] $p $h
}
}

# ------------------------------------------------------------------ the staged melee script: coverage list, guard list, state table
$covered = @(); $guardCovered = @(); $states = @{}
$meleeReds = if ($MeleeReds) { $MeleeReds } else { Join-Path $pkg 'r6\scripts\UTPP\utpp_melee.reds' }
if (Test-Path -LiteralPath $meleeReds) {
    $reds = Get-Content -Raw -LiteralPath $meleeReds
    $m = [regex]::Match($reds, '(?s)func\s+UTPPMeleeFullBodyCoveredName\(n:\s*CName\)\s*->\s*Bool\s*\{(.*?)\n\}')
    if ($m.Success) { $covered = @([regex]::Matches($m.Groups[1].Value, 'n"([A-Za-z0-9_]+)"') | ForEach-Object { $_.Groups[1].Value }) }
    else { Fail 'UTPPMeleeFullBodyCoveredName not found in utpp_melee.reds' }
    $m = [regex]::Match($reds, '(?s)func\s+UTPPMeleeGuardCoveredName\(n:\s*CName\)\s*->\s*Bool\s*\{(.*?)\n\}')
    if ($m.Success) { $guardCovered = @([regex]::Matches($m.Groups[1].Value, 'n"([A-Za-z0-9_]+)"') | ForEach-Object { $_.Groups[1].Value }) }
    $m = [regex]::Match($reds, '(?s)// UTPP_STATES_TABLE begin(.*?)// UTPP_STATES_TABLE end')
    if ($m.Success) {
        # every line between the markers: a line that is neither blank, a comment nor a state line = FAIL (it would be skipped silently)
        foreach ($ln in ($m.Groups[1].Value -split "`r?`n")) {
            $tl = $ln.Trim()
            if (-not $tl -or $tl.StartsWith('//')) { continue }
            $t = [regex]::Match($tl, '^if\s+Equals\(n,\s*n"([A-Za-z0-9_]+)"\)\s*\{\s*return\s+(\d+);\s*\}$')
            if (-not $t.Success) { Fail "UTPP_STATES_TABLE line not understood by this script (parse = if Equals(n, n`"<var>`") { return <mask>; }): $tl"; continue }
            if ($states.ContainsKey($t.Groups[1].Value)) { Fail "UTPP_STATES_TABLE lists $($t.Groups[1].Value) twice"; continue }
            $states[$t.Groups[1].Value] = [int64]$t.Groups[2].Value
        }
    } else { Fail 'UTPP_STATES_TABLE markers not found in utpp_melee.reds' }
    Info ("script covers (full body): " + ($covered -join ', ') + "; guard: " + ($guardCovered -join ', '))
    Info ("state table: " + (($states.GetEnumerator() | Sort-Object Name | ForEach-Object { '{0}=0x{1:X}' -f $_.Name, $_.Value }) -join ', '))
}

# ------------------------------------------------------------------ pack: run manifest, archive, .xl
$packedOk = @{}
$runManPath = Join-Path $pack 'manifest.json'
if (!(Test-Path -LiteralPath $pack)) { Fail "pack folder missing: $pack (the npc1 export has not finished)" }
elseif (!(Test-Path -LiteralPath $runManPath)) { Fail "run manifest missing: $runManPath" }
else {
    $a = Prop (ReadJson $runManPath) 'archive'
    if ($null -eq $a) { Fail "run manifest has no 'archive' (not packed)" }
    else {
        if ((Prop $a 'ok') -ne $true) { Fail "archive.ok = $(Prop $a 'ok') (mismatches $(Prop $a 'mismatches'))" }
        foreach ($c in @(Prop $a 'unbundle_compare')) { if ($c -and (Prop $c 'packed_equals_unpacked') -eq $true) { $packedOk[[string](Prop $c 'file')] = $true } }
        if (!(Test-Path -LiteralPath $Archive)) { Fail "archive missing: $Archive" }
        else {
            $h = Sha $Archive
            $want = [string](Prop $a 'sha256')
            if (-not $want -or $h -ne $want) { Fail "archive SHA-256 $h differs from the run manifest ($want)" }
            elseif ((Prop $a 'size') -and (Get-Item -LiteralPath $Archive).Length -ne [int64](Prop $a 'size')) { Fail 'archive size differs from the run manifest' }
            else { AddEntry 'archive\pc\mod\utpp_melee.archive' $Archive $h; Info ("archive sets read back equal: " + (($packedOk.Keys | Sort-Object) -join ', ')) }
        }
    }
}
$xlEntries = New-Object System.Collections.Generic.List[object]
if (!(Test-Path -LiteralPath $Xl)) { Fail ".xl missing: $Xl (compose it with Source\tools\anim\melee-export\make_melee_xl.py)" }
else {
    $lines = @(Get-Content -LiteralPath $Xl -Encoding UTF8)
    if (@($lines | Where-Object { $_ -match '^animations:\s*$' }).Count -ne 1) { Fail ".xl has no single 'animations:' section" }
    $cur = $null
    foreach ($l in $lines) {
        if ($l -match '^\s*-\s*entity:\s*$') {
            if ($cur) { $xlEntries.Add($cur) }
            $cur = [pscustomobject]@{ set = ''; vars = @(); priority = $null }
            continue
        }
        if (-not $cur) { continue }
        if ($l -match '^\s+set:\s*(\S+)\s*$') { $cur.set = $Matches[1] }
        elseif ($l -match '^\s+vars:\s*\[(.*)\]\s*$') { $cur.vars = @($Matches[1] -split ',' | ForEach-Object { $_.Trim() } | Where-Object { $_ }) }
        elseif ($l -match '^\s+priority:\s*(-?\d+)\s*$') { $cur.priority = [int]$Matches[1] }
    }
    if ($cur) { $xlEntries.Add($cur) }
    if ($xlEntries.Count -eq 0) { Fail '.xl has no entries' }
    foreach ($e in $xlEntries) {
        $f = Split-Path -Leaf $e.set
        if (-not $packedOk.ContainsKey($f)) { Fail ".xl set $($e.set) is not read back equal from the staged archive" }
        if ($e.vars -notcontains 'utpp_foot_tpp') { Fail ".xl entry $($e.set) does not gate on utpp_foot_tpp (it would replace the first-person clips too)" }
        # the default set has no variable in vanilla: without a weapon variable the entry would give its clips to every melee weapon
        if ($f -match '^utpp_(pma_)?default_melee_weapon\.anims$' -and @($e.vars | Where-Object { $_ -match '^w_' }).Count -eq 0) {
            $msg = ".xl entry $($e.set) has no weapon variable (w_*): the default-set clips would replace those of every melee weapon"
            if ($AllowCoverageMismatch) { Warn $msg } else { Fail ($msg + ' - gate it on the weapon variable (registration_evidence / make_melee_xl) or stage with -AllowCoverageMismatch') }
        }
    }
    AddEntry 'archive\pc\mod\utpp_melee_tpp.archive.xl' $Xl (Sha $Xl)
    Info (".xl entries: " + (($xlEntries | ForEach-Object { '{0} [{1}] {2}' -f (Split-Path -Leaf $_.set), ($_.vars -join ','), $_.priority }) -join '; '))
}

# ------------------------------------------------------------------ coverage list of the script vs the .xl
$femaleOnly = $false
foreach ($v in $covered) {
    $rigs = @($xlEntries | Where-Object { $_.vars -contains $v } | ForEach-Object { RigOf (Split-Path -Leaf $_.set) } | Sort-Object -Unique)
    if ($rigs.Count -eq 0) {
        $msg = "anim variable $v is covered by UTPPMeleeFullBodyCoveredName but the .xl registers no set for it: that weapon takes the full body under its vanilla first-person clip (static legs)"
        if ($AllowCoverageMismatch) { Warn $msg } else { Fail ($msg + ' - narrow the list in utpp_melee.reds or stage with -AllowCoverageMismatch') }
    } elseif ($rigs.Count -eq 1) {
        if ($rigs[0] -eq 'woman' -and $v -eq 'w_melee_fists') { $femaleOnly = $true }
        # The script gates the full body on the anim variable only (no rig check), so the other body would take the full body
        # under its vanilla first-person clips (static legs): the same defect as no rig at all.
        $msg = "anim variable $v is registered for the $($rigs[0]) rig only: the other body keeps the vanilla first-person clips under the full body (static legs)"
        if ($AllowCoverageMismatch) { Warn $msg } else { Fail ($msg + ' - stage a pack with both rigs (runs\npc1-fists) or stage with -AllowCoverageMismatch') }
    }
}

# ------------------------------------------------------------------ fists state mask vs the coverage of the pack
$stateName = @{ 5 = 'Parried'; 6 = 'Hold'; 7 = 'ChargedHold'; 8 = 'Block'; 10 = 'Deflect'; 11 = 'ComboAttack'; 12 = 'FinalAttack'
    13 = 'StrongAttack'; 14 = 'SafeAttack'; 15 = 'BlockAttack'; 16 = 'SprintAttack'; 17 = 'CrouchAttack'; 18 = 'JumpAttack'; 19 = 'ThrowAttack' }
# state -> clip family by vanilla clip name (heuristic; states without a known family are not checked)
$family = @{ 5 = '^parried_'; 6 = '^hold_'; 7 = '^hold_'; 8 = '^block(_start|_end)?$'; 10 = '^block_deflect_'; 11 = '^combo_attack_'
    12 = '^final_attack_'; 13 = '^heavy_attack_'; 14 = '^safe_attack'; 15 = '^block_attack'; 16 = '^sprint_attack'; 17 = '^crouch_attack'
    18 = '^jump_attack'; 19 = '^throw_' }
# clips whose state is not confirmed (the ground slam may play in JumpAttack 18): vanilla under a mask bit = WARN, not FAIL
$familyMaybe = @{ 18 = '^melee_groundslam_' }
$fistsMask = if ($states.ContainsKey('w_melee_fists')) { $states['w_melee_fists'] } else { $null }
# every line of the table: its sets = the .xl entries gated on the line's weapon variable (utpp_<set>.anims -> p?a_<weapon>)
$lineVars = @($states.Keys | Sort-Object)
$weaponSets = @{}
foreach ($v in $lineVars) {
    $weaponSets[$v] = @($xlEntries | Where-Object { $_.vars -contains $v } | ForEach-Object { (Split-Path -Leaf $_.set) -replace '^utpp_', '' -replace '\.anims$', '' } | Where-Object { $_ -match '^p[wm]a_[a-z0-9_]+$' } | Sort-Object -Unique)
}
$neededSets = @($weaponSets.Values | ForEach-Object { $_ } | Sort-Object -Unique)
$linesWhat = if ($lineVars.Count -eq 1 -and $lineVars[0] -eq 'w_melee_fists') { 'fists' } else { 'weapon' }
$covJson = [IO.Path]::ChangeExtension($Coverage, '.json')
$covSets = @{}   # set -> @{ tpp = names or $null; vanilla = names }
$covAll = $null  # coverage.json 'sets' (all sets), for the fists-family fallback clips below
if (Test-Path -LiteralPath $covJson) {
    $cj = Prop (ReadJson $covJson) 'sets'
    $covAll = $cj
    foreach ($s in $neededSets) {
        $d = Prop $cj $s
        if ($null -eq $d) { continue }
        $van = @()
        foreach ($k in @('not_built_clips', 'rejected_clips', 'no_input_clips')) { $o = Prop $d $k; if ($o) { $van += @($o.PSObject.Properties | ForEach-Object { $_.Name }) } }
        $van += @(Prop $d 'failed_clips')
        $tpp = @(Prop $d 'redone_clips') + @(Prop $d 'fallback_clips')
        $covSets[$s] = @{ tpp = @($tpp | Where-Object { $_ }); vanilla = @($van | Where-Object { $_ } | Sort-Object -Unique) }
    }
    Info "coverage: $covJson ($(($covSets.Keys | Sort-Object) -join ', '))"
} elseif (Test-Path -LiteralPath $Coverage) {
    $txt = @(Get-Content -LiteralPath $Coverage -Encoding UTF8)
    $iDll = -1; for ($i = 0; $i -lt $txt.Count; $i++) { if ($txt[$i] -match '^FOR THE DLL:') { $iDll = $i; break } }
    if ($iDll -lt 0) { Warn "coverage format not recognised ($Coverage): $linesWhat states not checked - compare the state table by hand" }
    else {
        $section = 'dll'
        for ($i = $iDll + 1; $i -lt $txt.Count; $i++) {
            $l = $txt[$i]
            if ($l -match '^REJECTED') { $section = 'rej'; continue }
            if ($l.Trim() -eq '') { if ($section -eq 'rej') { break } else { continue } }
            if ($l -notmatch '^\s') { break }   # the next section of the report
            if ($l -match '^\s+(p[wm]a_[a-z0-9_]+)\s+(.*)$' -and $neededSets -contains $Matches[1]) {
                $s = $Matches[1]; $rest = $Matches[2]
                if (-not $covSets.ContainsKey($s)) { $covSets[$s] = @{ tpp = $null; vanilla = @() } }
                if ($section -eq 'dll') { $names = @($rest -split ',' | ForEach-Object { ($_ -replace '\(.*?\)', '').Trim() } | Where-Object { $_ }) }
                else { $names = @(($rest.Trim() -split '\s+')[0]) }
                $covSets[$s].vanilla += $names
            }
        }
        Info "coverage: $Coverage (text; vanilla clips only: $(($covSets.Keys | Sort-Object) -join ', '))"
    }
} else { Warn "coverage missing ($Coverage): $linesWhat states not checked against the pack" }
foreach ($var in $lineVars) {
    $varMask = $states[$var]
    if ($weaponSets[$var].Count -eq 0) {
        $msg = "UTPP_STATES_TABLE has a line for $var but the .xl registers no set for it: the staged pack gives that weapon no third-person clips under its state mask"
        if ($AllowCoverageMismatch) { Warn $msg } else { Fail ($msg + ' - drop the line or stage the pack of that weapon (or stage with -AllowCoverageMismatch)') }
        continue
    }
    foreach ($s in $weaponSets[$var]) {
        if (-not $covSets.ContainsKey($s)) { if ($covSets.Count) { Warn "coverage has no $s" }; continue }
        $cv = $covSets[$s]
        foreach ($b in ($family.Keys | Sort-Object)) {
            $inMask = ($varMask -band ([int64]1 -shl $b)) -ne 0
            $van = @($cv.vanilla | Where-Object { $_ -match $family[$b] })
            $hasTpp = $null -ne $cv.tpp   # the text coverage lists only the vanilla clips
            $tp = @()
            if ($hasTpp) { $tp = @($cv.tpp | Where-Object { $_ -match $family[$b] }) }
            if ($inMask -and $van.Count) { Fail "$var mask has $($stateName[$b]) ($b) but $s keeps $($van -join ', ') vanilla: the full body would show the first-person legs" }
            elseif ($inMask -and $hasTpp -and $tp.Count -eq 0) { Warn "$var mask has $($stateName[$b]) ($b) but $s has no third-person clip of it" }
            elseif (-not $inMask -and $hasTpp -and $tp.Count -and -not $van.Count) {
                Info ("candidate for the $var line: {0} ({1}, +{2}) - {3} has third-person {4}" -f $stateName[$b], $b, ([int64]1 -shl $b), $s, ($tp -join ', '))
            }
            if ($inMask -and $familyMaybe.ContainsKey($b)) {
                $vm = @($cv.vanilla | Where-Object { $_ -match $familyMaybe[$b] })
                if ($vm.Count) { Warn "$var mask has $($stateName[$b]) ($b) and $s keeps $($vm -join ', ') vanilla: if the game plays them in that state (not confirmed), the legs follow the first-person clip on the ground - check in game" }
            }
        }
        foreach ($g in $guardCovered) {
            if ($g -ne $var) { continue }
            if ($cv.vanilla -contains 'melee_idlepose') { Warn "$s keeps melee_idlepose vanilla: the guard (melee.guard) would show the first-person pose - keep it off" }
            elseif ($null -ne $cv.tpp -and $cv.tpp -contains 'melee_idlepose') { Info "$s has a third-person melee_idlepose (guard pose)" }
        }
    }
}

# fists-family fallback clips (default_melee_weapon: jump / slide / body slam; Gorilla Arms one_punch) against the same w_melee_fists line:
# a third-person clip whose state is not in the mask is only shown through the upper-body mask (the legs stay motion matching)
if ($null -ne $fistsMask -and $null -ne $covAll) {
    $fallbackFam = @(
        @{ rx = '^fall_attack';   bit = 18; what = 'JumpAttack' },
        @{ rx = '^slide_attack';  bit = 17; what = 'CrouchAttack' },
        @{ rx = '^one_punch_';    bit = 13; what = 'StrongAttack' }
    )
    foreach ($s in @('default_melee_weapon', 'pma_default_melee_weapon', 'pwa_strongarms_melee_fists', 'pma_strongarms_melee_fists')) {
        $d = Prop $covAll $s
        if ($null -eq $d) { continue }
        $redone = @(Prop $d 'redone_clips' | Where-Object { $_ })
        if ($redone.Count -eq 0) { continue }
        Info ("fists-family clips of ${s}: " + ($redone -join ', '))
        foreach ($ff in $fallbackFam) {
            $hit = @($redone | Where-Object { $_ -match $ff.rx })
            if ($hit.Count -eq 0) { continue }
            if (($fistsMask -band ([int64]1 -shl $ff.bit)) -eq 0) { Warn "$s has third-person $($hit -join ', ') but the w_melee_fists mask lacks $($ff.what) ($($ff.bit)): only the upper-body mask shows them (legs stay motion matching)" }
            else { Info "$s $($hit -join ', '): $($ff.what) ($($ff.bit)) is in the w_melee_fists mask" }
        }
    }
}

# ------------------------------------------------------------------ parried graph (player_melee.animgraph replacement)
# A state line with the Parried bit (1 -shl 5 = 32) whose sets hold per-strike parried clips (parried_<left|right>_<suffix>, not the plain
# parried_left / parried_right) is only meaningful with the patched graph (vanilla picks the clip by hit_direction only): the graph archive is
# then REQUIRED. -AllowCoverageMismatch does not relax this.
function ClipNamesOf([string]$set) {
    $names = @(); $known = $false
    if ($covSets.ContainsKey($set) -and $null -ne $covSets[$set].tpp) { $names += @($covSets[$set].tpp); $known = $true }
    $rp = Join-Path $pack "reports\build-$set.json"
    if (Test-Path -LiteralPath $rp) {
        $cl = Prop (ReadJson $rp) 'clips'
        if ($null -ne $cl) { $names += @($cl.PSObject.Properties | ForEach-Object { $_.Name }); $known = $true }
    }
    return @{ names = @($names | Where-Object { $_ } | Sort-Object -Unique); known = $known }
}
$graphNeeds = New-Object System.Collections.Generic.List[string]
foreach ($var in $lineVars) {
    if (($states[$var] -band ([int64]1 -shl 5)) -eq 0) { continue }
    foreach ($s in $weaponSets[$var]) {
        $cn = ClipNamesOf $s
        if (-not $cn.known) {
            $msg = "$var has the Parried bit (32) but the clip names of $s are unknown (no third-person clip list in the coverage and no $pack\reports\build-$s.json): cannot tell whether it needs the parried graph"
            if ($var -eq 'w_melee_fists') { Warn $msg } else { Fail $msg }   # fists: only the generic parried_left / parried_right exist
            continue
        }
        $per = @($cn.names | Where-Object { $_ -match '^parried_(left|right)_.+' })
        if ($per.Count) { $graphNeeds.Add(("{0}/{1}: {2}{3}" -f $var, $s, (($per | Select-Object -First 3) -join ', '), $(if ($per.Count -gt 3) { ", ... ($($per.Count) per-strike clips)" } else { '' }))) }
    }
}
if ($GraphArchive) {
    if ($GraphSha256 -ne $graphPinSha) { Fail "-GraphSha256 $GraphSha256 is not the pinned graph SHA-256 $graphPinSha (the graph is pinned to the 2.5 vanilla player_melee.animgraph; change the pin deliberately here and in make_release.py)" }
    elseif (!(Test-Path -LiteralPath $GraphArchive)) { Fail "graph archive missing: $GraphArchive" }
    else {
        $gh = Sha $GraphArchive
        if ($gh -ne $GraphSha256) { Fail "graph archive SHA-256 $gh differs from the pinned $GraphSha256" }
        else {
            $gOk = $true
            $gm = Join-Path (Split-Path -Parent (Split-Path -Parent $GraphArchive)) 'manifest.json'   # <run>\packed\<archive> -> <run>\manifest.json
            if (Test-Path -LiteralPath $gm) {
                $gj = ReadJson $gm
                if ([string](Prop $gj 'archiveSha256') -ne $gh) { Fail "graph run manifest ${gm}: archiveSha256 $(Prop $gj 'archiveSha256') differs from the archive"; $gOk = $false }
                if ([string](Prop $gj 'vanillaSha256') -ne $graphVanillaSha) { Fail "graph run manifest ${gm}: vanillaSha256 $(Prop $gj 'vanillaSha256') is not the pinned 2.5 vanilla graph $graphVanillaSha"; $gOk = $false }
                $gl = @(Prop $gj 'archiveListing')
                if ($gl.Count -ne 1 -or [string]$gl[0] -ne $graphListing) { Fail "graph run manifest ${gm}: archiveListing is not exactly $graphListing"; $gOk = $false }
            } else { Warn "graph run manifest not found ($gm): the vanilla graph pin and the archive listing were not cross-checked" }
            if ($gOk) {
                AddEntry $graphRel $GraphArchive $gh
                Info "graph archive SHA-256 $gh (pinned; patched copy of the 2.5 player_melee.animgraph, vanilla $graphVanillaSha)"
            }
        }
    }
}
if ($graphNeeds.Count -and -not $GraphArchive) {
    Fail ("the parried graph archive is required but not staged - Parried (32) is in the state mask and the pack has per-strike parried clips: " + ($graphNeeds -join '; ') +
        " - stage it with -GraphArchive <run>\packed\utpp_melee_parried_graph.archive (SHA-256 $graphPinSha), or keep the Parried bit off (knife 3668160, katana 3669440)")
} elseif ($GraphArchive -and -not $graphNeeds.Count) {
    Warn "graph archive staged but no state line needs it (no Parried bit with per-strike parried clips): it replaces the game's player_melee.animgraph for nothing - drop -GraphArchive unless intended"
} elseif ($graphNeeds.Count) { Info ("parried graph needed by: " + ($graphNeeds -join '; ')) }

# ------------------------------------------------------------------ optional: run-start lean databases (all four or none)
if ($WithRunStartLean) {
    $dbStage = Join-Path $root 'Build\stages\runstart-lean-20261109'
    $dbManPath = Join-Path $root 'Research\inventory\runstart-lean-20261109.json'
    if (!(Test-Path -LiteralPath $dbManPath)) { Fail "manifest missing: $dbManPath" }
    else {
        $dbFiles = @((ReadJson $dbManPath).files)
        if ($dbFiles.Count -ne 4) { Fail "runstart-lean manifest lists $($dbFiles.Count) files, expected 4" }
        foreach ($f in $dbFiles) {
            $p = Join-Path $dbStage $f.path
            if (!(Test-Path -LiteralPath $p)) { Fail "runstart-lean file missing: $p"; continue }
            $h = Sha $p
            if ($h -ne $f.sha256) { Fail "runstart-lean file $($f.path): SHA-256 $h differs from its manifest" ; continue }
            AddEntry $f.path $p $h
        }
    }
}

# ------------------------------------------------------------------ targets (installer allow-list) and the plan
$seen = @{}
foreach ($e in $entries) {
    if ([IO.Path]::IsPathRooted($e.path) -or $e.path -match '(^|[\\/])\.\.([\\/]|$)') { Fail "unsafe path $($e.path)" }
    if ($e.path -notmatch $allowRx) { Fail "target not accepted by install_verified_stage.ps1: $($e.path)" }
    if ($e.path -match '^red4ext\\plugins\\UTPP\\db\\v2\\recoil\\') { Fail "recoil outside the v2 data root: $($e.path)" }
    if ($seen.ContainsKey($e.path)) { Fail "duplicate target $($e.path)" }
    $seen[$e.path] = $true
}
# runtime files of the package that are not staged: report the ones that differ from the installed copy (read only)
if (Test-Path -LiteralPath $game) {
    $rt = @()
    if (Test-Path -LiteralPath (Join-Path $pkg 'r6\scripts\UTPP')) { $rt += @(Get-ChildItem -LiteralPath (Join-Path $pkg 'r6\scripts\UTPP') -Filter '*.reds' -File) }
    $cetPkg = Join-Path $pkg 'bin\x64\plugins\cyber_engine_tweaks\mods'
    if (Test-Path -LiteralPath $cetPkg) { $rt += @(Get-ChildItem -LiteralPath $cetPkg -Recurse -File -Filter '*.lua') }
    $other = @()
    foreach ($f in $rt) {
        $rel = $f.FullName.Substring($pkg.Length + 1)
        if ($seen.ContainsKey($rel)) { continue }
        $g = Join-Path $game $rel
        if (!(Test-Path -LiteralPath $g)) { $other += "$rel (not installed)" }
        elseif ((Sha $f.FullName) -ne (Sha $g)) { $other += $rel }
    }
    if ($other.Count) { Info ("not staged, differs from the installed copy (check that it is comment-only or stage it): " + ($other -join ', ')) }
    else { Info 'every other runtime file of the package equals the installed copy' }
}
Write-Output ''
Write-Output ('{0,-68} {1,11}  {2}' -f 'target (game-relative)', 'bytes', 'sha256 / source')
foreach ($e in $entries) { Write-Output ('{0,-68} {1,11}  {2}  {3}' -f $e.path, $e.bytes, $e.sha256, $e.source) }
Write-Output ''
if ($script:fails.Count) {
    Write-Output "$(if ($DryRun) { 'DRYRUN_FAIL' } else { 'STAGE_REFUSED' }) fails=$($script:fails.Count) warnings=$($script:warns.Count) (nothing written)"
    exit 1
}
if ($DryRun) {
    Write-Output "DRYRUN_OK files=$($entries.Count) warnings=$($script:warns.Count) stage=$stage manifest=$manPath (nothing written)"
    exit 0
}

# ------------------------------------------------------------------ write the stage (fresh folder, real copies) and the manifest
if (Test-Path -LiteralPath $stage) { throw "Stage already exists: $stage" }
if (Test-Path -LiteralPath $manPath) { throw "Manifest already exists: $manPath" }
New-Item -ItemType Directory -Path $stage | Out-Null
foreach ($e in $entries) {
    if ((Get-Item -LiteralPath $e.source).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse source: $($e.source)" }
    $dst = [IO.Path]::GetFullPath((Join-Path $stage $e.path))
    if (!$dst.StartsWith($stage + '\')) { throw "Path escaped the stage: $($e.path)" }
    New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($dst)) | Out-Null
    Copy-Item -LiteralPath $e.source -Destination $dst   # a new file, never a hardlink: later edits of the source cannot change the stage
    if ((Sha $dst) -ne $e.sha256) { throw "Copy hash mismatch: $($e.path)" }
    if ((Sha $e.source) -ne $e.sha256) { throw "Source changed while staging: $($e.source)" }
}
$codeNote = if ($ArchiveOnly) { 'ARCHIVE ONLY (no DLL / scripts / CET staged; the installed ones stay); ' } else { 'UTPP.dll + utpp_melee.reds / utpp_body.reds / utpp_modsettings.reds + CET utpp_core init.lua (Build\UTPP\package = Source); ' }
$noteText = "melee npc1 (NPC-based third-person fists): " + $codeNote +
    "utpp_melee.archive from $Archive; utpp_melee_tpp.archive.xl from $Xl (" +
    (($xlEntries | ForEach-Object { '{0} [{1}]' -f (Split-Path -Leaf $_.set), ($_.vars -join ',') }) -join '; ') + ")" +
    $(if ($WithRunStartLean) { '; + run-start lean databases (runstart-lean-20261109)' } else { '' }) +
    $(if ($GraphArchive) { "; + utpp_melee_parried_graph.archive from $GraphArchive (patched copy of base\gameplay\anim_graphs\player_melee.animgraph, pinned to the 2.5 vanilla graph $graphVanillaSha; remove with disable_utpp_archive.ps1 -ArchiveName utpp_melee_parried_graph)" } else { '' }) +
    $(if ($femaleOnly) { '; FEMALE-ONLY fists: the installed utpp_melee.archive is replaced, Male V fists go back to the vanilla first-person clips' } else { '' }) +
    $(if ($script:warns.Count) { '; warnings: ' + ($script:warns -join ' | ') } else { '' }) +
    $(if ($Note) { '; ' + $Note } else { '' })
$manifest = [ordered]@{
    createdAt = (Get-Date).ToString('yyyy-MM-dd')
    stagedAt = (Get-Date).ToString('o')
    tool = 'Source\tools\install\stage_melee_npc.ps1'
    files = @($entries | ForEach-Object { [ordered]@{ path = $_.path; sha256 = $_.sha256; bytes = $_.bytes } })
    note = $noteText
    runtimeVisualPass = $false
    sourceCommit = 'uncommitted'
}
New-Item -ItemType Directory -Force -Path $invPath | Out-Null
$manifest | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $manPath
# read back the way install_verified_stage.ps1 does (manifest files, allow-list, stage hashes)
$back = Get-Content -Raw -LiteralPath $manPath | ConvertFrom-Json
foreach ($f in $back.files) {
    if ($f.path -notmatch $allowRx) { throw "Read-back: target not accepted: $($f.path)" }
    if ((Sha (Join-Path $stage $f.path)) -ne $f.sha256) { throw "Read-back: stage hash mismatch: $($f.path)" }
}
Write-Output "STAGE_OK name=$StageName files=$(@($back.files).Count) warnings=$($script:warns.Count) stage=$stage manifest=$manPath"
Write-Output "Next (lead, game closed): powershell -NoProfile -ExecutionPolicy Bypass -File Source\tools\install\install_verified_stage.ps1 -StageName $StageName -ManifestName $manifestName   (preflight; add -Apply to install)"
exit 0

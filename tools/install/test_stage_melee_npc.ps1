<#
.SYNOPSIS
  Self-check of the parried-graph rule of stage_melee_npc.ps1 and of the installer allow-list. Offline: every stage run is -DryRun
  (nothing is written to the stage, the inventory, the game or Vortex); scratch files go to Build\work\melee-graph-rule-20261110\test-tmp.

.DESCRIPTION
  Usage (project root):  powershell -NoProfile -ExecutionPolicy Bypass -File Source\tools\install\test_stage_melee_npc.ps1
  Exit 0 = every check passed, 1 = a check failed, 3 = inputs missing (nothing checked).
  Inputs: the fists pack (Build\work\melee-export-20261109\runs\npc1-fists), the knife r4 pack (Build\work\melee-b1-knife-20261109\runs\b1-knife,
  archive SHA-256 7ED2D2B2...), the run5 graph archive (Build\work\melee-parried-graph-20261110\run5\packed). The knife line is NOT in
  the shipped utpp_melee.reds yet, so the knife cases read a scratch copy of the script (-MeleeReds) with a knife line.
#>
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
$stageScript = Join-Path $PSScriptRoot 'stage_melee_npc.ps1'
$installScript = Join-Path $PSScriptRoot 'install_verified_stage.ps1'
$fistsPack = Join-Path $root 'Build\work\melee-export-20261109\runs\npc1-fists'
$knifePack = Join-Path $root 'Build\work\melee-b1-knife-20261109\runs\b1-knife'
$graph = Join-Path $root 'Build\work\melee-parried-graph-20261110\run5\packed\utpp_melee_parried_graph.archive'
$graphSha = 'D5786BB159CEC4AD8A334B15B6762785A5B587E1DF0664A79DE91F0CFDC87449'
$knifeSha = '7ED2D2B2DE06CEA8935F6FABF00FCA8D7738A28F8800170D1E266DDA1458ACE7'
$srcReds = Join-Path $root 'Source\plugin\UTPP\scripts\utpp_melee.reds'
$tmp = Join-Path $root 'Build\work\melee-graph-rule-20261110\test-tmp'
$script:failed = 0
$script:passed = 0
function Check([bool]$ok, [string]$what) {
    if ($ok) { $script:passed++; Write-Output "PASS  $what" } else { $script:failed++; Write-Output "FAIL  $what" }
}
foreach ($need in @($fistsPack, $knifePack, $graph, $srcReds)) {
    if (!(Test-Path -LiteralPath $need)) { Write-Output "MISSING input: $need"; exit 3 }
}
if ((Get-FileHash -LiteralPath (Join-Path $knifePack 'packed\utpp_melee.archive') -Algorithm SHA256).Hash -ne $knifeSha) { Write-Output 'MISSING input: the knife pack is not r4 (archive SHA-256 differs)'; exit 3 }
if ((Get-FileHash -LiteralPath $graph -Algorithm SHA256).Hash -ne $graphSha) { Write-Output 'MISSING input: the graph archive is not the pinned run5 one'; exit 3 }

function RunStage([string[]]$more) {
    $a = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $stageScript, '-DryRun', '-StageName', 'zz-rule-test', '-ArchiveOnly') + $more
    $o = & powershell @a 2>&1 | Out-String
    return [pscustomobject]@{ rc = $LASTEXITCODE; out = $o }
}
$graphFail = 'the parried graph archive is required but not staged'

# ---- allow-list: installer and stage script carry the same regex, and it accepts the graph archive only by its exact name
function AllowRx([string]$file, [string]$pattern) {
    $m = [regex]::Match((Get-Content -Raw -LiteralPath $file), $pattern)
    if (!$m.Success) { throw "allow-list not found in $file" }
    return $m.Groups[1].Value
}
$rxInstall = AllowRx $installScript "-notmatch '(\^\(red4ext[^']*)'"
$rxStage = AllowRx $stageScript "\`$allowRx = '(\^\(red4ext[^']*)'"
Check ($rxInstall -ceq $rxStage) 'allow-list of install_verified_stage.ps1 equals the copy in stage_melee_npc.ps1 (byte for byte)'
Check ('archive\pc\mod\utpp_melee_parried_graph.archive' -match $rxInstall) 'allow-list accepts archive\pc\mod\utpp_melee_parried_graph.archive'
foreach ($bad in @('archive\pc\mod\utpp_melee_parried_graph.archive.xl', 'archive\pc\mod\utpp_melee_parried_graphX.archive', 'archive\pc\mod\other\utpp_melee_parried_graph.archive', 'archive\pc\mod\melee_parried_graph.archive')) {
    Check (-not ($bad -match $rxInstall)) "allow-list refuses $bad"
}
Check ('archive\pc\mod\utpp_melee.archive' -match $rxInstall) 'allow-list still accepts utpp_melee.archive'

# ---- scratch copies of utpp_melee.reds (one covered weapon each)
if (Test-Path -LiteralPath $tmp) { Remove-Item -LiteralPath $tmp -Recurse -Force }
New-Item -ItemType Directory -Path $tmp | Out-Null
$text = Get-Content -Raw -LiteralPath $srcReds
# Scratch copy of the shipped script that covers exactly one weapon variable: the coverage function lists only that
# variable and the state table holds only the given line(s), so the result does not depend on how many weapons the
# shipped script covers.
function ScratchReds([string]$name, [string]$variable, [string]$tableLines) {
    $t = [regex]::Replace($text, '(?s)(func\s+UTPPMeleeFullBodyCoveredName\(n:\s*CName\)\s*->\s*Bool\s*\{).*?(\n\})',
        { param($m) $m.Groups[1].Value + "`n  return Equals(n, n`"$variable`");" + $m.Groups[2].Value })
    $t = [regex]::Replace($t, '(?s)(// UTPP_STATES_TABLE begin).*?(// UTPP_STATES_TABLE end)',
        { param($m) $m.Groups[1].Value + "`n  " + $tableLines + "`n  " + $m.Groups[2].Value })
    $p = Join-Path $tmp $name
    Set-Content -LiteralPath $p -Value $t -Encoding UTF8
    return $p
}
function KnifeReds([string]$name, [string]$maskLine) { return ScratchReds $name 'w_melee_knife' $maskLine }
$fistsLine = [regex]::Match($text, 'if Equals\(n, n"w_melee_fists"\) \{ return \d+; \}').Value
$fistsReds = ScratchReds 'fists_only.reds' 'w_melee_fists' $fistsLine

# ---- fists-only stage: generic parried clips only: no graph needed, no graph mention
$r = RunStage @('-PackDir', $fistsPack, '-MeleeReds', $fistsReds)
Check ($r.rc -eq 0 -and $r.out -match 'DRYRUN_OK files=2 ') 'fists pack + fists-only script: DRYRUN_OK, 2 files (archive + xl)'
$findings = (($r.out -split "`n") | Where-Object { $_ -notmatch '^WARN\s+-MeleeReds' }) -join "`n"   # that line echoes the scratch path
Check ($findings -notmatch 'graph') 'fists-only run mentions no parried graph'
Check ($findings -notmatch 'parried graph') 'fists-only run has no parried-graph finding'
$r = RunStage @('-PackDir', $fistsPack, '-MeleeReds', $fistsReds, '-GraphArchive', $graph)
Check ($r.rc -eq 0 -and $r.out -match 'graph archive staged but no state line needs it' -and $r.out -match 'files=3 ') 'fists + graph given: WARN not needed, graph is the 3rd staged file'

# ---- knife cases: scratch copy with the knife line only
$knifeOn = KnifeReds 'knife_on.reds' 'if Equals(n, n"w_melee_knife") { return 3668192; }'
$knifeOff = KnifeReds 'knife_off.reds' 'if Equals(n, n"w_melee_knife") { return 3668160; }'
$badLine = KnifeReds 'knife_bad.reds' ('if Equals(n, n"w_melee_knife") { return 3668192; }' + "`n  if (Equals(n, n`"w_melee_katana`")) { return 3669472; }")
$twice = KnifeReds 'knife_twice.reds' ('if Equals(n, n"w_melee_knife") { return 3668192; }' + "`n  if Equals(n, n`"w_melee_knife`") { return 3668192; }")
Check ((Get-Content -Raw -LiteralPath $knifeOn) -match 'return 3668192;') 'scratch script holds the knife line (3668192)'
$kp = @('-PackDir', $knifePack)

$r = RunStage ($kp + @('-MeleeReds', $knifeOn))
Check ($r.rc -eq 1 -and $r.out -match [regex]::Escape($graphFail) -and $r.out -match 'w_melee_knife/pwa_knife: parried_left_') 'knife 3668192 without the graph: FAIL (graph required), names the per-strike clips'
Check ($r.out -match '3668160' -and $r.out -match '3669440') 'the FAIL message names the graph-less masks (knife 3668160, katana 3669440)'
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-AllowCoverageMismatch'))
Check ($r.rc -eq 1 -and $r.out -match [regex]::Escape($graphFail)) 'knife 3668192 without the graph + -AllowCoverageMismatch: still FAIL'
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-GraphArchive', $graph))
Check ($r.rc -eq 0 -and $r.out -notmatch [regex]::Escape($graphFail) -and $r.out -match 'DRYRUN_OK files=3 ' -and $r.out -match 'utpp_melee_parried_graph\.archive') 'knife 3668192 + pinned graph: DRYRUN_OK, graph staged as the 3rd file'
$r = RunStage ($kp + @('-MeleeReds', $knifeOff))
Check ($r.rc -eq 0 -and $r.out -notmatch 'parried graph' -and $r.out -notmatch [regex]::Escape($graphFail)) 'knife 3668160 (Parried off) without the graph: OK'
$r = RunStage ($kp + @('-MeleeReds', $knifeOff, '-GraphArchive', $graph))
Check ($r.rc -eq 0 -and $r.out -match 'graph archive staged but no state line needs it') 'knife 3668160 + graph: WARN, not needed'

# ---- the graph archive itself is verified
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-GraphArchive', $graph, '-GraphSha256', ('0' * 64)))
Check ($r.rc -eq 1 -and $r.out -match 'is not the pinned graph SHA-256') 'a non-pinned -GraphSha256 is refused'
$tampered = Join-Path $tmp 'tampered\packed'
New-Item -ItemType Directory -Path $tampered -Force | Out-Null
$bytes = [IO.File]::ReadAllBytes($graph); $bytes[$bytes.Length - 1] = $bytes[$bytes.Length - 1] -bxor 1
[IO.File]::WriteAllBytes((Join-Path $tampered 'utpp_melee_parried_graph.archive'), $bytes)
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-GraphArchive', (Join-Path $tampered 'utpp_melee_parried_graph.archive')))
Check ($r.rc -eq 1 -and $r.out -match 'graph archive SHA-256 .* differs from the pinned') 'a modified graph archive is refused'
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-GraphArchive', (Join-Path $tmp 'nope.archive')))
Check ($r.rc -eq 1 -and $r.out -match 'graph archive missing') 'a missing graph archive is refused'
# the pinned bytes copied next to a manifest that disagrees: cross-check (archiveSha256 / vanillaSha256 / listing)
$okDir = Join-Path $tmp 'okcopy\packed'
New-Item -ItemType Directory -Path $okDir -Force | Out-Null
Copy-Item -LiteralPath $graph -Destination (Join-Path $okDir 'utpp_melee_parried_graph.archive')
$man = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path (Split-Path -Parent (Split-Path -Parent $graph)) 'manifest.json') | ConvertFrom-Json
$man.vanillaSha256 = ('A' * 64)
$man | ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $tmp 'okcopy\manifest.json')
$r = RunStage ($kp + @('-MeleeReds', $knifeOn, '-GraphArchive', (Join-Path $okDir 'utpp_melee_parried_graph.archive')))
Check ($r.rc -eq 1 -and $r.out -match 'vanillaSha256') 'run manifest with another vanilla graph SHA-256 is refused'

# ---- state table parsing: every line is understood, or FAIL
$r = RunStage ($kp + @('-MeleeReds', $badLine))
Check ($r.rc -eq 1 -and $r.out -match 'UTPP_STATES_TABLE line not understood') 'a state line the parser cannot read = FAIL (not skipped silently)'
$r = RunStage ($kp + @('-MeleeReds', $twice))
Check ($r.rc -eq 1 -and $r.out -match 'lists w_melee_knife twice') 'a duplicate state line = FAIL'

Remove-Item -LiteralPath $tmp -Recurse -Force
Write-Output ''
Write-Output "test_stage_melee_npc: $($script:passed) passed, $($script:failed) failed"
if ($script:failed) { exit 1 }
exit 0

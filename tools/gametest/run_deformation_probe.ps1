<#
.SYNOPSIS
Run the existing 18-second armed deformation scenario in an ALREADY LOADED game.
.DESCRIPTION
No build, installation, game launch, MM/perspective/database or persisted setting
changes. Requires drawn firearm, TPP, clear target and default gamepad X reload.
Uses run.py's existing guarded CET console automation and persistent virtual pad.

run.py has no --cet-end option. This wrapper therefore uses a SECOND CET-only
run in finally to Stop/flush the deformation probe and disable the temporary feed
dump. If focus is lost or the game exits, guarded cleanup can fail; stop/flush
manually with GetMod("utpp_deform_probe").Stop(); Game.UTPPSetMMFeedDump(false).
The probe itself is capped at six four-second capture intervals. No gameplay
settings are restored/overwritten because none are changed here.

DryRun forwards --dry-run to BOTH invocations: no game input or ViGEm writes.
#>
[CmdletBinding()]
param(
    [switch]$DryRun,
    [switch]$ReloadPhaseOnly,
    [string]$Python = 'python',
    [string]$Out = '',
    [string]$Log = '',
    [string]$CetInputRel = ''
)

$ErrorActionPreference = 'Stop'
$null = Get-Command $Python -ErrorAction Stop
$probeRunner = Join-Path $PSScriptRoot 'run.py'
$probeScenarios = Join-Path $PSScriptRoot 'deformation_scenarios.json'
$probeRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
if (!$Out) {
    $Out = Join-Path $probeRoot ('Research/logs/gametest/deformprobe-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
}
$probeStart = 'local p=GetMod("utpp_deform_probe"); assert(p and p.Stop and p.Start,"utpp_deform_probe unavailable"); assert(p.Stop(),"probe stop failed"); p.MAX_CYCLES=6; p.INTERVAL=4; assert(p.Start(),"probe start failed")'
$probeStop = 'local p=GetMod("utpp_deform_probe"); assert(p and p.Stop,"utpp_deform_probe unavailable"); assert(p.Stop(),"probe flush failed")'
if ($ReloadPhaseOnly) {
    $probeStart = 'local p=GetMod("utpp_reload_probe"); assert(p and p.Start and p.Start(),"CET reload phase probe unavailable")'
    $probeStop = 'local p=GetMod("utpp_reload_probe"); assert(p and p.Stop and p.Stop(),"CET reload phase probe cleanup failed")'
}
$probeToken = Get-Date -Format 'yyyyMMdd-HHmmss-ffff'
$probeStartMarker = '[utpp_deformation_wrapper] START ' + $probeToken
$probeStopMarker = '[utpp_deformation_wrapper] STOP ' + $probeToken
$probeStart += '; Game.UTPPLogLine("' + $probeStartMarker + '")'
$probeStop += '; Game.UTPPLogLine("' + $probeStopMarker + '")'
function Test-ProbeMarker([string]$PlanPath, [string]$Marker) {
    if (!(Test-Path -LiteralPath $PlanPath)) { return $false }
    $probePlan = Get-Content -LiteralPath $PlanPath -Raw | ConvertFrom-Json
    if (!$probePlan.log -or !(Test-Path -LiteralPath $probePlan.log)) { return $false }
    # The retail logger keeps this file open for writing during the probe.
    $probeStream = [IO.File]::Open($probePlan.log, [IO.FileMode]::Open,
        [IO.FileAccess]::Read, [IO.FileShare]::ReadWrite)
    $probeReader = [IO.StreamReader]::new($probeStream)
    try { return $probeReader.ReadToEnd().Contains($Marker) }
    finally { $probeReader.Dispose() }
}
$probeCommon = @('--keep-pad', '--no-dismiss', '--wake', 'none')
if ($DryRun) { $probeCommon += '--dry-run' }
if ($Log) { $probeCommon += @('--log', $Log) }
if ($CetInputRel) { $probeCommon += @('--cet-input-rel', $CetInputRel) }
$probeFeedStart = if ($ReloadPhaseOnly) { '' } else { 'Game.UTPPSetMMFeedDump(false);;Game.UTPPSetMMFeedDump(true);;' }
$probeFeedStop = if ($ReloadPhaseOnly) { '' } else { ';;Game.UTPPSetMMFeedDump(false)' }
$probePreflightOut = Join-Path $Out 'preflight'
$probePreflightArguments = @($probeRunner, '--out', $probePreflightOut, '--cet',
    ($probeFeedStart + $probeStart)) + $probeCommon
$probeArguments = @($probeRunner, '--scenario-file', $probeScenarios, '--scenarios', 'armed_short',
    '--repeat', '1', '--out', $Out) + $probeCommon
$probeCleanupArguments = @($probeRunner, '--out', (Join-Path $Out 'teardown'), '--cet',
    ($probeStop + $probeFeedStop)) + $probeCommon
$probeExitCode = 1
$probeCleanupExitCode = 1
Write-Host '18 s: Ready -> Aim -> two shots -> Ready -> Reload -> neutral. The game must already be loaded.'
Write-Host 'A loaded weapon IN HAND and a clear line of fire are required. The script neither draws the weapon nor switches the camera.'
if ($ReloadPhaseOnly) { Write-Host 'Reload phase only: no bone dumps and no change to feed-dump. Does not start authored playback.' }
try {
    & $Python @probePreflightArguments
    if ($LASTEXITCODE -ne 0) { throw 'Probe preflight failed; no scenario input sent.' }
    if (!$DryRun -and !(Test-ProbeMarker (Join-Path $probePreflightOut 'plan.json') $probeStartMarker)) {
        throw 'Probe START not confirmed; no scenario input sent.'
    }
    & $Python @probeArguments
    $probeExitCode = $LASTEXITCODE
}
finally {
    # The persistent pad is neutralized by run.py even on failure. Cleanup
    # deliberately uses the same foreground guard and never bypasses it.
    & $Python @probeCleanupArguments
    $probeCleanupExitCode = $LASTEXITCODE
    if (!$DryRun -and !(Test-ProbeMarker (Join-Path $Out 'teardown/plan.json') $probeStopMarker)) {
        $probeCleanupExitCode = 2
    }
    if ($probeCleanupExitCode -ne 0) {
        Write-Warning 'The end of the diagnostics was not confirmed. After regaining focus run Stop() and UTPPSetMMFeedDump(false) in CET.'
    }
}
if ($probeExitCode -ne 0) { exit $probeExitCode }
exit $probeCleanupExitCode

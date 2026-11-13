param([int]$N = 10, [string]$Label = 'utpp_on', [int]$Hold = 85, [string]$LogDir = '')
# Loads the newest save N times (the game continues it by itself after launch) and counts crashes in the first $Hold seconds after
# the process appears. No input is sent. Result lines go to crash_loop_<label>.log in -LogDir (default: Research\logs\diag).
if (-not $LogDir) { $LogDir = Join-Path ([IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))) 'Research\logs\diag' }
$log = Join-Path $LogDir ("crash_loop_$Label.log")
$queue = Join-Path $env:LOCALAPPDATA 'REDEngine\ReportQueue'
"start $Label N=$N $(Get-Date -Format s)" | Out-File $log -Encoding utf8
$crashes = 0
for ($i = 1; $i -le $N; $i++) {
    Get-Process Cyberpunk2077, CrashReporter -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep 6
    $before = @(Get-ChildItem $queue -Directory -ErrorAction SilentlyContinue | ForEach-Object Name)
    Start-Process 'steam://rungameid/1091500'
    $t0 = Get-Date; $p = $null
    while (-not $p -and ((Get-Date) - $t0).TotalSeconds -lt 150) { Start-Sleep 2; $p = Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue }
    if (-not $p) { "run $i NO_PROCESS" | Out-File $log -Append -Encoding utf8; continue }
    $t1 = Get-Date; $state = 'alive'
    while (((Get-Date) - $t1).TotalSeconds -lt $Hold) {
        Start-Sleep 3
        $new = @(Get-ChildItem $queue -Directory -ErrorAction SilentlyContinue | ForEach-Object Name | Where-Object { $before -notcontains $_ })
        if ($new.Count -gt 0) { $state = "CRASH $($new[0]) at +$([int]((Get-Date) - $t1).TotalSeconds)s"; break }
        if (-not (Get-Process Cyberpunk2077 -ErrorAction SilentlyContinue)) { $state = "EXITED at +$([int]((Get-Date) - $t1).TotalSeconds)s"; break }
    }
    if ($state -ne 'alive') { $crashes++ }
    "run $i $state" | Out-File $log -Append -Encoding utf8
    Get-Process Cyberpunk2077, CrashReporter -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
}
"done $Label crashes=$crashes of $N $(Get-Date -Format s)" | Out-File $log -Append -Encoding utf8

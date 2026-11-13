# UTPP animation deformation survey (2026-11-09): read-only extraction of every animset the four 2.5 player entities register
# (player_wa_fpp, player_ma_fpp, player_wa_fpp_ep1, player_ma_fpp_ep1; list = player_entity_sets_unique.json) from the installed
# game archives (content + ep1; archive\pc\mod is NOT read, so the data are vanilla). Modelled on Source\tools\melee\extract_npc_melee.ps1.
#   -Stage raw : unbundle the .anims to Extracted\game\player-anims\raw (depot layout), provenance.txt (SHA-256, size, depot path),
#                missing-set check, serialize every .anims to json\ (clip lists, durations, rig, events).
#   -Stage glb : export the .anims listed in -List (depot paths relative to raw\, one per line) to glb\<file>.glb with root motion
#                (AnimationExportArgs__incRootMotion=true), at most -MaxProc concurrent detached WolvenKit processes, logs in logs\.
# Writes only under Extracted\game\player-anims\. The game folder is only read.
#   powershell -ExecutionPolicy Bypass -File extract_player_anims.ps1 -Stage raw
#   powershell -ExecutionPolicy Bypass -File extract_player_anims.ps1 -Stage glb -List <list.txt> [-MaxProc 6]
param(
  [ValidateSet('raw', 'glb')][string]$Stage = 'raw',
  [string]$Game = '',
  [string]$SetsJson = '',
  [string]$List = '',
  [int]$MaxProc = 6,
  [switch]$SkipUnbundle
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)))
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
if (-not $Game) { $Game = Get-UtppGameDir }
if (!(Test-Path (Join-Path $root 'Source\tools'))) { throw "project root not found from $PSScriptRoot" }
if (!$SetsJson) { $SetsJson = Join-Path $root 'Build\work\melee-export-20261109\evidence\player_entity_sets_unique.json' }
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\player-anims'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $glb = Join-Path $out 'glb'; $logs = Join-Path $out 'logs'
foreach ($d in @($raw, $json, $glb, $logs)) { New-Item -ItemType Directory -Force $d | Out-Null }

if ($Stage -eq 'raw') {
  $arch = @('content\basegame_4_animation', 'content\basegame_1_engine', 'content\basegame_4_gamedata', 'content\basegame_3_nightcity',
            'ep1\ep1_1_nightcity', 'ep1\ep1_2_gamedata')
  $doc = Get-Content $SetsJson -Raw | ConvertFrom-Json
  $paths = @{}
  foreach ($p in $doc.PSObject.Properties) { foreach ($r in $p.Value) { if ($r.set) { $paths[$r.set.ToLower()] = 1 } } }
  $alts = @($paths.Keys | Sort-Object | ForEach-Object { [regex]::Escape($_) + '$' })
  $re = '(' + ($alts -join ')|(') + ')'
  "sets registered: $($paths.Count)"
  if (!$SkipUnbundle) {
    foreach ($a in $arch) {
      $p = Join-Path (Join-Path $Game 'archive\pc') ($a + '.archive')
      if (!(Test-Path $p)) { "skip (missing) $p"; continue }
      "unbundle $a"
      & $wkit unbundle $p -o $raw -r $re | Out-File (Join-Path $logs ('unbundle_' + ($a -replace '\\', '_') + '.log')) -Encoding utf8
      if ($LASTEXITCODE -ne 0) { throw "unbundle $a failed $LASTEXITCODE" }
    }
  }
  $files = Get-ChildItem $raw -Recurse -File
  $files | ForEach-Object { '{0}  {1}  {2}' -f (Get-FileHash $_.FullName -Algorithm SHA256).Hash, $_.Length, $_.FullName.Substring($raw.Length + 1) } |
    Set-Content (Join-Path $out 'provenance.txt') -Encoding ascii
  "raw files: $($files.Count)"
  $missing = @($paths.Keys | Where-Object { !(Test-Path (Join-Path $raw $_)) })
  "missing registered sets: $($missing.Count)"; $missing | ForEach-Object { "  MISSING $_" }
  # serialize per raw directory into the mirrored json directory: 13 file names occur twice (female / male player_locomotion,
  # base / ep1 cyberware ...), a flat output would overwrite them.
  Get-ChildItem $json -File | Remove-Item
  $dirs = @($files | Where-Object { $_.Extension -eq '.anims' } | ForEach-Object { $_.DirectoryName } | Sort-Object -Unique)
  $bad = 0
  foreach ($d in $dirs) {
    $rel = $d.Substring($raw.Length + 1)
    $o = Join-Path $json $rel
    New-Item -ItemType Directory -Force $o | Out-Null
    & $wkit convert serialize $d -o $o | Out-File (Join-Path $logs ('serialize_' + ($rel -replace '\\', '_') + '.log')) -Encoding utf8
    if ($LASTEXITCODE -ne 0) { $bad++; "  serialize failed $rel" }
  }
  "serialize dirs: $($dirs.Count), failed $bad"
  "json files: $((Get-ChildItem $json -Recurse -File).Count)"
}
else {
  if (!$List -or !(Test-Path $List)) { throw 'glb stage needs -List <file with depot paths>' }
  $env:AnimationExportArgs__incRootMotion = 'true'
  $items = @(Get-Content $List | Where-Object { $_.Trim() } | ForEach-Object { $_.Trim() })
  "glb export: $($items.Count) sets, max $MaxProc processes"
  $running = @()
  $HungMinutes = 20
  function Stop-Hung($procs) {
    foreach ($p in @($procs | Where-Object { !$_.HasExited })) {
      if (((Get-Date) - $p.StartTime).TotalMinutes -gt $HungMinutes) { "  HUNG > $HungMinutes min, stopping export pid $($p.Id)"; Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
    }
  }
  foreach ($rel in $items) {
    $src = Join-Path $raw $rel
    if (!(Test-Path $src)) { "  MISSING raw $rel"; continue }
    $name = Split-Path $rel -Leaf
    $o = Join-Path $glb (Split-Path $rel -Parent)          # mirrored depot layout (13 file names occur twice)
    New-Item -ItemType Directory -Force $o | Out-Null
    if (Test-Path (Join-Path $o ($name + '.glb'))) { "  skip (exists) $rel"; continue }
    while (@($running | Where-Object { !$_.HasExited }).Count -ge $MaxProc) { Start-Sleep -Milliseconds 500; Stop-Hung $running }
    $tag = ($rel -replace '\\', '_')
    $a = @('export', ('"' + $src + '"'), '-o', ('"' + $o + '"'), '-gp', ('"' + $Game + '"'))
    $running += Start-Process -FilePath $wkit -ArgumentList $a -RedirectStandardOutput (Join-Path $logs ('export_' + $tag + '.log')) `
      -RedirectStandardError (Join-Path $logs ('export_' + $tag + '.err')) -PassThru -WindowStyle Hidden
  }
  # poll instead of Wait-Process: a WolvenKit export that throws (female player_locomotion: duplicate root-motion key) can stay
  # alive forever; Stop-Hung stops only our own export processes after $HungMinutes
  while (@($running | Where-Object { !$_.HasExited }).Count -gt 0) { Start-Sleep -Milliseconds 500; Stop-Hung $running }
  $made = @($items | Where-Object { Test-Path (Join-Path $glb ($_ + '.glb')) })
  "glb made: $($made.Count) / $($items.Count)"
  @($items | Where-Object { !(Test-Path (Join-Path $glb ($_ + '.glb'))) }) | ForEach-Object { "  NO GLB $_" }
}

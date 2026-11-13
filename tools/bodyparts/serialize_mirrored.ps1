# Step 2 (after extract_body_parts.ps1): serialize every raw file separately into the mirrored json directory (the tool flattens / recurses directories).
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path  # Source\tools\bodyparts -> project root
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root 'Extracted\game\body-parts-250'
$raw = Join-Path $out 'raw'; $json = Join-Path $out 'json'; $logs = Join-Path $out 'logs'
New-Item -ItemType Directory -Force $json | Out-Null
$files = @(Get-ChildItem $raw -Recurse -File)
"files: $($files.Count)"
$procs = @()
$i = 0
foreach ($f in $files) {
  $rel = $f.DirectoryName.Substring($raw.Length + 1)
  $o = Join-Path $json $rel
  New-Item -ItemType Directory -Force $o | Out-Null
  $i++
  $lg = Join-Path $logs ('ser_{0:D3}.log' -f $i)
  $procs += Start-Process -FilePath $wkit -ArgumentList @('convert', 'serialize', "`"$($f.FullName)`"", '-o', "`"$o`"") -NoNewWindow -PassThru -RedirectStandardOutput $lg
  while (@($procs | Where-Object { !$_.HasExited }).Count -ge 8) { Start-Sleep -Milliseconds 200 }
}
$procs | ForEach-Object { $_.WaitForExit() }
"json files: $((Get-ChildItem $json -Recurse -File -Filter *.json).Count)"

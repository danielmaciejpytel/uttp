# Rifle grip + stock 2026-11-10: read-only extraction of one weapon's meshes (default Militech Saratoga SMG) from the installed game
# archives (archive\pc\content + ep1; archive\pc\mod is NOT read) with WolvenKit CLI, then glb export without materials.
# Output: Extracted\game\weapon-mesh-<Name>-20261110\ (raw + glb) and logs in Build\work\rifle-grip-stock-20261110\mesh_logs.
# Writes nothing into the game folder.
param(
  [string]$Game = '',
  [string]$Name = 'saratoga',
  [string]$Regex = 'base\\weapons\\firearms\\smg\\militech_saratoga\\entities\\meshes\\.*\.mesh$'
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Game) { $Game = Get-UtppGameDir }
$wkit = Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe'
$out = Join-Path $root ("Extracted\game\weapon-mesh-$Name-20261110")
$logs = Join-Path $root 'Build\work\rifle-grip-stock-20261110\mesh_logs'
New-Item -ItemType Directory -Force -Path $out, $logs | Out-Null
$archives = Get-ChildItem -Path (Join-Path $Game 'archive\pc\content'), (Join-Path $Game 'archive\pc\ep1') -Filter '*.archive' -ErrorAction SilentlyContinue
foreach ($a in $archives) {
  & $wkit unbundle $a.FullName -o $out -r $Regex | Out-File (Join-Path $logs ('unbundle_' + $a.BaseName + '.log')) -Encoding utf8
}
$meshes = Get-ChildItem -Path $out -Recurse -Filter '*.mesh'
"meshes found: $($meshes.Count)"
foreach ($f in $meshes) {
  & $wkit export $f.FullName -o $f.DirectoryName -gp $Game | Out-File (Join-Path $logs ('export_' + $f.BaseName + '.log')) -Encoding utf8
}
Get-ChildItem -Path $out -Recurse -Filter '*.glb' | ForEach-Object { $_.FullName }

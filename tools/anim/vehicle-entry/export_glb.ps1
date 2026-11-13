# Generic: export .anims files to .anims.glb with WolvenKit.CLI (rig resolved from the game archives, read-only).
# -RootMotion binds AnimationExportArgs.incRootMotion through the environment.
param(
  [Parameter(Mandatory)][string[]]$Files,
  [Parameter(Mandatory)][string]$OutDir,
  [switch]$RootMotion,
  [string]$Game = ''
)
. (Join-Path $PSScriptRoot '..\..\utpp_paths.ps1')
$root = Get-UtppProjectRoot
if (-not $Game) { $Game = Get-UtppGameDir }
$ErrorActionPreference = 'Stop'
$full = [IO.Path]::GetFullPath($OutDir)
if (!$full.StartsWith($root + '\')) { throw 'OutDir outside the project' }
if (!(Test-Path $full)) { New-Item -ItemType Directory -Path $full | Out-Null }
$env:AnimationExportArgs__incRootMotion = $(if ($RootMotion) { 'true' } else { 'false' })
& (Join-Path $root 'Tools\WolvenKit.CLI\WolvenKit.CLI.exe') export @Files -o $full -gp $Game
exit $LASTEXITCODE

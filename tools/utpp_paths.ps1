# Paths of the project folder and of the game, without machine-specific paths in the sources.
# Usage in a script one level below tools\:   . (Join-Path $PSScriptRoot '..\utpp_paths.ps1')
#   Get-UtppProjectRoot      the folder that contains Source, Build, Data ... (environment variable UTPP_PROJECT_ROOT,
#                            else the nearest folder above this file that contains both "Source" and "Build")
#   Get-UtppGameDir          the Cyberpunk 2077 folder (environment variable UTPP_GAME_DIR, else "game_dir" in
#                            local_paths.json); throws when it is not configured unless -Optional is given
#   Get-UtppLocalPath <key>  any other machine-specific path: environment variable UTPP_<KEY>, else <key> in
#                            local_paths.json, else -Default
# local_paths.json is not tracked. It is read from the folder of this file, else from Source\tools of the nearest
# folder above it that has one (so a second working copy inside the project uses the same file);
# local_paths.example.json shows its keys.

$script:UtppPathsDir = $PSScriptRoot
$script:UtppLocalPaths = $null

function Get-UtppProjectRoot {
    if ($env:UTPP_PROJECT_ROOT) { return $env:UTPP_PROJECT_ROOT }
    $dir = $script:UtppPathsDir
    while ($true) {
        $parent = Split-Path -Parent $dir
        if (-not $parent -or $parent -eq $dir) { break }
        $dir = $parent
        if ((Test-Path -LiteralPath (Join-Path $dir 'Source') -PathType Container) -and
            (Test-Path -LiteralPath (Join-Path $dir 'Build') -PathType Container)) { return $dir }
    }
    return [IO.Path]::GetFullPath((Join-Path $script:UtppPathsDir '..\..'))
}

function Get-UtppLocalPath {
    param([Parameter(Mandatory = $true)][string]$Key, [string]$Default = '')
    $fromEnv = [Environment]::GetEnvironmentVariable('UTPP_' + $Key.ToUpperInvariant())
    if ($fromEnv) { return $fromEnv }
    if ($null -eq $script:UtppLocalPaths) {
        $script:UtppLocalPaths = New-Object PSObject
        $candidates = @((Join-Path $script:UtppPathsDir 'local_paths.json'))
        $dir = $script:UtppPathsDir
        while ($true) {   # every folder above this file that has Source\tools (nearest first)
            $parent = Split-Path -Parent $dir
            if (-not $parent -or $parent -eq $dir) { break }
            $dir = $parent
            $candidates += (Join-Path $dir 'Source\tools\local_paths.json')
        }
        foreach ($file in $candidates) {
            if (Test-Path -LiteralPath $file) {
                $script:UtppLocalPaths = Get-Content -Raw -LiteralPath $file | ConvertFrom-Json
                break
            }
        }
    }
    $value = $script:UtppLocalPaths.PSObject.Properties[$Key]
    if ($value -and $value.Value) { return [string]$value.Value }
    return $Default
}

function Get-UtppGameDir {
    param([switch]$Optional)
    $dir = Get-UtppLocalPath -Key 'game_dir'
    if (-not $dir -and -not $Optional) {
        throw ('The game folder is not configured: set the environment variable UTPP_GAME_DIR or "game_dir" in ' +
            (Join-Path $script:UtppPathsDir 'local_paths.json'))
    }
    return $dir
}

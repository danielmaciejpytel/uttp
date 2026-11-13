@echo off
rem Knee-fix variant of the source pack retarget (retarget_batch.py --knee-fix) into NEW folders; the current v2 inputs
rem out_sourcepack_{woman,man}[_rtz] are never written. Usage: run_retarget_knee.cmd woman|man [rtz]
rem Same inputs / options as run_retarget_{woman,man}.cmd full (and rtz), plus --knee-fix; tools from this worktree.
rem Logs: Research\logs\retarget-sourcepack-<rig>[-rtz]-knee.log.
setlocal
set "BLENDER=%ProgramFiles(x86)%\Steam\steamapps\common\Blender\blender.exe"
if defined UTPP_BLENDER set "BLENDER=%UTPP_BLENDER%"
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "TOOLS=%~dp0"
set "EXP=%ROOT%\Animations\sourcepack\als_export"
set "META=%ROOT%\Animations\retarget\meta_sourcepack"
set "RTZLIST=%ROOT%\Animations\retarget\out_sourcepack_woman_rtz\clips.txt"
if /i "%~1"=="woman" (
  set "CPBLEND=%ROOT%\Animations\retarget\cp_player_woman_skeleton.blend"
  set "RIG=%ROOT%\Extracted\game\json\vanilla\player_woman_skeleton.rig.json"
  set "BONEMAP=%TOOLS%bonemap_source_to_game_woman.json"
) else if /i "%~1"=="man" (
  set "CPBLEND=%ROOT%\Animations\retarget\cp_player_man_skeleton.blend"
  set "RIG=%ROOT%\Extracted\game\json\vanilla-man\player_man_skeleton.rig.json"
  set "BONEMAP=%TOOLS%bonemap_source_to_game_man.json"
) else (
  echo usage: %~nx0 woman^|man [rtz]
  exit /b 2
)
if /i "%~2"=="rtz" (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_%~1_rtz_knee"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-%~1-rtz-knee.log"
  set "EXTRA=--list "%RTZLIST%" --rtz"
) else (
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_%~1_knee"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-%~1-knee.log"
  set "EXTRA="
)
"%BLENDER%" --background --factory-startup --python "%TOOLS%retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --no-preview --cp-blend "%CPBLEND%" --rig-json "%RIG%" --bonemap "%BONEMAP%" --keep-all-curves --knee-fix %EXTRA% > "%LOG%" 2>&1
set RC=%ERRORLEVEL%
findstr /C:"UTPP_RETARGET_DONE" "%LOG%"
exit /b %RC%

@echo off
rem Retarget of the source pack slide export (Animations\sourcepack\slide_export) -> NEW dirs Animations\retarget\out_sourcepack_slide_{woman,man}[_relaxed].
rem Usage: run_retarget_slide.cmd woman|man [relaxed]   (Neutral = 16 clips, relaxed = the 24 Relaxed clips)
rem Same options as run_retarget_knee.cmd (bonemap/rig/--keep-all-curves/--knee-fix); no --rtz: slide root motion is Y-only, root Z constant.
rem Meta = Animations\retarget\meta_sourcepack_slide (export meta copy; contact_l/r come from the export, all 0 in slides = feet slide).
rem Logs: Research\logs\retarget-sourcepack-slide-<rig>[-relaxed].log.
setlocal
set "BLENDER=%ProgramFiles(x86)%\Steam\steamapps\common\Blender\blender.exe"
if defined UTPP_BLENDER set "BLENDER=%UTPP_BLENDER%"
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "TOOLS=%~dp0"
set "EXP=%ROOT%\Animations\sourcepack\slide_export"
set "META=%ROOT%\Animations\retarget\meta_sourcepack_slide"
if /i "%~1"=="woman" (
  set "CPBLEND=%ROOT%\Animations\retarget\cp_player_woman_skeleton.blend"
  set "RIG=%ROOT%\Extracted\game\json\vanilla\player_woman_skeleton.rig.json"
  set "BONEMAP=%TOOLS%bonemap_source_to_game_woman.json"
) else if /i "%~1"=="man" (
  set "CPBLEND=%ROOT%\Animations\retarget\cp_player_man_skeleton.blend"
  set "RIG=%ROOT%\Extracted\game\json\vanilla-man\player_man_skeleton.rig.json"
  set "BONEMAP=%TOOLS%bonemap_source_to_game_man.json"
) else (
  echo usage: %~nx0 woman^|man [relaxed]
  exit /b 2
)
if /i "%~2"=="relaxed" (
  set "LIST=%TOOLS%slide_clips_relaxed.txt"
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_slide_%~1_relaxed"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-slide-%~1-relaxed.log"
) else (
  set "LIST=%TOOLS%slide_clips_neutral.txt"
  set "OUT=%ROOT%\Animations\retarget\out_sourcepack_slide_%~1"
  set "LOG=%ROOT%\Research\logs\retarget-sourcepack-slide-%~1.log"
)
"%BLENDER%" --background --factory-startup --python "%TOOLS%retarget_batch.py" -- --root "%ROOT%" --fbx-dir "%EXP%\fbx" --meta-dir "%META%" --out-dir "%OUT%" --list "%LIST%" --no-preview --cp-blend "%CPBLEND%" --rig-json "%RIG%" --bonemap "%BONEMAP%" --keep-all-curves --knee-fix > "%LOG%" 2>&1
set RC=%ERRORLEVEL%
findstr /C:"UTPP_RETARGET_DONE" "%LOG%"
exit /b %RC%

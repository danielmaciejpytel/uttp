@echo off
rem Re-bake of the v2 DBs from the --knee-fix retarget (run_retarget_knee.cmd): the exact bake_args of
rem Data\utpp_motion_{woman,man}_v2.summary.json, only the clip dirs (--clips, --rtz-clips) point to the *_knee folders;
rem outputs Data\utpp_motion_{woman,man}_v2_knee.utppmm (+ .chooser.json, .summary.json). The original DBs are not written.
rem Usage: run_bake_knee.cmd woman|man   (baker = main checkout Source\tools\mmbake, unchanged)
setlocal
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
cd /d "%ROOT%"
if /i "%~1"=="woman" (
  python Source\tools\mmbake\mmbake_base.py --format v2 --clips Animations\retarget\out_sourcepack_woman_knee --rtz-clips Animations\retarget\out_sourcepack_woman_rtz_knee --rig Extracted\game\json\vanilla\player_woman_skeleton.rig.json --bonemap Source\tools\retarget\bonemap_source_to_game_woman.json --motion-src sourcepack_export --out Data\utpp_motion_woman_knee.utppmm --log Research\logs\mmbake-sourcepack-woman-v2-knee.log > Research\logs\mmbake-sourcepack-woman-v2-knee.stdout.log 2>&1
) else if /i "%~1"=="man" (
  python Source\tools\mmbake\mmbake_base.py --format v2 --clips Animations\retarget\out_sourcepack_man_knee --rtz-clips Animations\retarget\out_sourcepack_man_rtz_knee --sourcepack-only --rig Extracted\game\json\vanilla-man\player_man_skeleton.rig.json --bonemap Source\tools\retarget\bonemap_source_to_game_man.json --motion-src sourcepack_export --out Data\utpp_motion_man_knee.utppmm --log Research\logs\mmbake-sourcepack-man-v2-knee.log > Research\logs\mmbake-sourcepack-man-v2-knee.stdout.log 2>&1
) else (
  echo usage: %~nx0 woman^|man
  exit /b 2
)
exit /b %ERRORLEVEL%

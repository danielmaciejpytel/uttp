@echo off
rem Bake of the v2 bundles with the source pack slide data: the bake_args of Data\utpp_motion_{woman,man}_knee.summary.json
rem plus the slide clip dirs (--clips a;b) and --slide-spec (make_slide_spec.py). The current bundles are never written.
rem   woman-twistfix: the INSTALLED woman base (Build\test-data\LocomotionTwistFix-20261017, DB sha 392c5755, corrected Run/Sprint clips:
rem   clips-staging / extra-headlock / rtz and the explicit TwistFix bake inputs) + the slide clips -> Data\utpp_motion_woman_knee_twistfix_slide.utppmm
rem Usage: run_bake_slide.cmd woman-twistfix|man|man-twistfix  -> Data\utpp_motion_woman_knee_twistfix_slide.utppmm / utpp_motion_man_knee_slide.utppmm
rem   man-twistfix: male clips re-retargeted with the leg TwistFix, new dirs out_sourcepack_{man_knee,man_rtz_knee,slide_man}_twistfix -> Build\work\aimotion-db\outman1\man1.utppmm
rem Baker = this worktree (tools\mmbake, --slide-spec); spec = Research\inventory\mmbake-v2-slide.json
setlocal
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "T=%~dp0..\mmbake"
cd /d "%ROOT%"
set "TF=Build\test-data\LocomotionTwistFix-20261017"
if /i "%~1"=="woman-twistfix" (
  python "%T%\mmbake_base.py" --format v2 --clips "%TF%\bake-inputs\clips-staging;Animations\retarget\out_sourcepack_slide_woman" --meta Animations\sourcepack\SourcePack_export\meta --config Source\tools\mmbake\config\default.json --v2-config Research\inventory\mmbake-sourcepack.json --sourcepack-dir Extracted\sourcepack\sourcepack --clip-map Research\inventory\sourcepack-clip-map.json --rig Extracted\game\json\vanilla\player_woman_skeleton.rig.json --bonemap Source\tools\retarget\bonemap_source_to_game_woman.json --ue-tracks Extracted\sourcepack\sourcepack-ue-tracks\modifier_tracks.json --motion-src sourcepack_export --extra-clips "%TF%\bake-inputs\extra-headlock" --rtz-clips "%TF%\bake-inputs\rtz" --sourcepack-only --slide-spec Research\inventory\mmbake-v2-slide.json --out Data\utpp_motion_woman_knee_twistfix_slide.utppmm --log Research\logs\mmbake-sourcepack-woman-twistfix-slide.log > Research\logs\mmbake-sourcepack-woman-twistfix-slide.stdout.log 2>&1
) else if /i "%~1"=="man" (
  python "%T%\mmbake_base.py" --format v2 --clips "Animations\retarget\out_sourcepack_man_knee;Animations\retarget\out_sourcepack_slide_man" --rtz-clips Animations\retarget\out_sourcepack_man_rtz_knee --sourcepack-only --rig Extracted\game\json\vanilla-man\player_man_skeleton.rig.json --bonemap Source\tools\retarget\bonemap_source_to_game_man.json --motion-src sourcepack_export --slide-spec Research\inventory\mmbake-v2-slide.json --out Data\utpp_motion_man_knee_slide.utppmm --log Research\logs\mmbake-sourcepack-man-v2-knee-slide.log > Research\logs\mmbake-sourcepack-man-v2-knee-slide.stdout.log 2>&1
) else if /i "%~1"=="man-twistfix" (
  rem man-twistfix 2026-11-06: the 'man' recipe incl. the path-fixed v2 config/slide spec of Build\work\aimotion-db (reproduces the installed man DB) on the leg TwistFix retargets
  rem   out_sourcepack_man_knee_twistfix / out_sourcepack_man_rtz_knee_twistfix / out_sourcepack_slide_man_twistfix, retarget_batch.py ankle_offset_yaw ->  Build\work\aimotion-db\outman1\man1.utppmm
  if not exist Build\work\aimotion-db\outman1 mkdir Build\work\aimotion-db\outman1
  python "%T%\mmbake_base.py" --format v2 --clips "Animations\retarget\out_sourcepack_man_knee_twistfix;Animations\retarget\out_sourcepack_slide_man_twistfix" --rtz-clips Animations\retarget\out_sourcepack_man_rtz_knee_twistfix --sourcepack-only --meta Animations\sourcepack\SourcePack_export\meta --config Source\tools\mmbake\config\default.json --v2-config Build\work\aimotion-db\mmbake-sourcepack-fixed.json --sourcepack-dir Extracted\sourcepack\sourcepack --clip-map Research\inventory\sourcepack-clip-map.json --ue-tracks Extracted\sourcepack\sourcepack-ue-tracks\modifier_tracks.json --rig Extracted\game\json\vanilla-man\player_man_skeleton.rig.json --bonemap Source\tools\retarget\bonemap_source_to_game_man.json --motion-src sourcepack_export --slide-spec Build\work\aimotion-db\mmbake-v2-slide-fixed.json --out Build\work\aimotion-db\outman1\man1.utppmm --log Build\work\aimotion-db\outman1\man1.log > Build\work\aimotion-db\outman1\man1.stdout.log 2>&1
) else (
  echo usage: %~nx0 woman-twistfix^|man^|man-twistfix
  exit /b 2
)
exit /b %ERRORLEVEL%

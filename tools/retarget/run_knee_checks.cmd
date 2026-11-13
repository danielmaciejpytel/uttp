@echo off
rem Knee metrics + bake-blocking knee gate of a source pack retarget output. Usage: run_knee_checks.cmd woman|man <out-dir-name> <label> [gate-only]
rem -> Research\logs\knee-fix\knee_<label>.* (knee_metrics.py, skipped with gate-only) and
rem    Research\logs\knee-fix\check_knee_<label>.json / .txt (check_retarget_out.py --knee-cache; exit 2 = knee gate failed).
rem Nothing is written into the retarget output dir (safe on the current v2 inputs out_sourcepack_*).
setlocal
for %%I in ("%~dp0..\..\..") do set "ROOT=%%~fI"
set "T=%~dp0"
cd /d "%ROOT%"
if /i "%~1"=="woman" (set "RIG=Extracted\game\json\vanilla\player_woman_skeleton.rig.json") else (set "RIG=Extracted\game\json\vanilla-man\player_man_skeleton.rig.json")
set "OUT=Animations\retarget\%~2"
set "LOGD=Research\logs\knee-fix"
if /i not "%~4"=="gate-only" python "%T%knee_metrics.py" metrics --cache Research\logs\knee\ue_cache --out-dir "%OUT%" --label %~3 --report-dir %LOGD% > %LOGD%\metrics_%~3.log 2>&1
python "%T%check_retarget_out.py" --out-dir "%OUT%" --meta-dir Animations\retarget\meta_sourcepack --rig %RIG% --clip-map Research\inventory\sourcepack-clip-map.json --names Research\inventory\sourcepack-names.json --pair-set full --require-all-curves --knee-cache Research\logs\knee\ue_cache --json %LOGD%\check_knee_%~3.json > %LOGD%\check_knee_%~3.txt 2>&1
echo rc=%ERRORLEVEL% >> %LOGD%\check_knee_%~3.txt

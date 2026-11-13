@echo off
rem Weapon films 2026-11-09: compiles weapons_eval.cpp (production MotionPose.hpp, header-only) into Build\work\weapon-films-20261109\bin.
rem Never writes into Build\UTPP. Log: Build\work\weapon-films-20261109\bin\build.log
setlocal
for %%I in ("%~dp0..\..\..") do set "R=%%~fI"
set "W=%R%\Build\work\weapon-films-20261109\bin"
if not exist "%W%" mkdir "%W%"
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%W%"
cl /nologo /std:c++20 /EHsc /O2 /W3 /DNOMINMAX "%R%\Source\tools\films\weapons_eval.cpp" /Fo"%W%\\" /Fe"%W%\weapons_eval.exe" > "%W%\build.log" 2>&1
set RC=%ERRORLEVEL%
type "%W%\build.log"
echo BUILD_EXIT %RC%
exit /b %RC%

@echo off
REM Launch the Rerun lap-replay viewer.
REM
REM Usage:
REM   view_replay.bat                            (defaults: Sprint A, Tomas ghost + v2 sim)
REM   view_replay.bat slip                       (use the v3 slip-model sim trace)
REM   view_replay.bat <track> <ghost> <sim>      (full override, three paths)

setlocal

set REPO=%~dp0
set TRACKS=%REPO%tracks_csv\ks_nurburgring

set TRACK=%TRACKS%\layout_sprint_a.csv
set GHOST=%TRACKS%\layout_sprint_a__tomas_full_sim_telemetry.csv
set SIM=%TRACKS%\layout_sprint_a__tomas_full_sim_trace.csv

if /I "%~1"=="slip" (
    set SIM=%TRACKS%\layout_sprint_a__tomas_full_sim_trace_slip.csv
) else if not "%~3"=="" (
    set TRACK=%~1
    set GHOST=%~2
    set SIM=%~3
)

echo Killing any existing rerun.exe instances...
taskkill /F /IM rerun.exe >nul 2>&1

echo Track: %TRACK%
echo Ghost: %GHOST%
echo Sim:   %SIM%
echo.

pushd "%REPO%"
python -m viz.rerun_replay --track "%TRACK%" --ghost "%GHOST%" --sim "%SIM%"
popd

endlocal

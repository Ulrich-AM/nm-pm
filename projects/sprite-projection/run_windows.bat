@echo off
setlocal
cd /d "%~dp0"
title NM-PM Sprite Projection - Run

if exist "build\Release\vdspr_phase8.exe" (
  start "" "build\Release\vdspr_phase8.exe"
  exit /b 0
)

if exist "build\vdspr_phase8.exe" (
  start "" "build\vdspr_phase8.exe"
  exit /b 0
)

echo The Phase 8 executable has not been built yet.
echo Run build_windows.bat first.
echo.
pause
exit /b 1

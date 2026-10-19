@echo off
setlocal
cd /d "%~dp0"
title NM-PM Sprite Projection - Phase 8 Body Animation

echo ==============================================
echo  NM-PM Sprite Projection - Phase 8 Body Animation
echo ==============================================
echo.

where cmake >nul 2>nul
if errorlevel 1 goto :no_cmake

where git >nul 2>nul
if errorlevel 1 (
  echo [WARN] Git was not found in PATH.
  echo        This is only okay if raylib 6.2 is already installed for CMake.
  echo.
)

echo [1/3] Configuring...
cmake -S . -B build
if errorlevel 1 goto :failed

echo.
echo [2/3] Building Release...
cmake --build build --config Release
if errorlevel 1 goto :failed

echo.
echo [3/3] Running core + evaluator + orientation-grid tests...
ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Build and tests completed successfully.
echo.

if exist "build\Release\vdspr_phase8.exe" (
  echo Launching build\Release\vdspr_phase8.exe...
  start "" "build\Release\vdspr_phase8.exe"
  exit /b 0
)

if exist "build\vdspr_phase8.exe" (
  echo Launching build\vdspr_phase8.exe...
  start "" "build\vdspr_phase8.exe"
  exit /b 0
)

echo [ERROR] Build reported success, but vdspr_phase8.exe was not found.
echo Check the build folder for the generated executable.
echo.
pause
exit /b 1

:no_cmake
echo [ERROR] CMake was not found in PATH.
echo Install CMake and a C++20 compiler such as Visual Studio 2022 Build Tools.
echo Then run this file again.
echo.
pause
exit /b 1

:failed
echo.
echo ==============================================
echo  BUILD FAILED
echo  The error message is shown above.
echo  This window will stay open so you can read it.
echo ==============================================
echo.
pause
exit /b 1

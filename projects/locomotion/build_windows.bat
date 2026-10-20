@echo off
setlocal
cd /d "%~dp0"
title NM-PM Locomotion - LO-003 Standing Solver

echo ==============================================
echo  NM-PM Locomotion - LO-003 Standing Solver
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
echo [3/3] Running LO-001 + LO-002 + LO-003 tests...
ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Build and tests completed successfully.
echo.

if exist "build\Release\locomotion_phase2.exe" (
  start "" "build\Release\locomotion_phase2.exe"
  exit /b 0
)
if exist "build\locomotion_phase2.exe" (
  start "" "build\locomotion_phase2.exe"
  exit /b 0
)

echo [ERROR] Build reported success, but locomotion_phase2.exe was not found.
pause
exit /b 1

:no_cmake
echo [ERROR] CMake was not found in PATH.
echo Install CMake and a C++20 compiler such as Visual Studio 2022 Build Tools.
pause
exit /b 1

:failed
echo.
echo ==============================================
echo  BUILD FAILED
echo  The error message is shown above.
echo ==============================================
pause
exit /b 1

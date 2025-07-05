@echo off
chcp 65001 > nul
cd /d "%~dp0"

echo =========================================
echo     Drone GUI Teleoperation System
echo            (English Version)
echo =========================================
echo.

rem Set UTF-8 encoding
for /f "tokens=2 delims==" %%i in ('wmic os get codeset /value') do set CODESET=%%i
if not "%CODESET%"=="65001" (
    echo Setting UTF-8 encoding...
    chcp 65001 > nul
)

echo Checking system requirements...

rem Check if X11 display is available
echo Checking X11 display...
wsl test -S "/tmp/.X11-unix/X0"
if %ERRORLEVEL% neq 0 (
    echo.
    echo X11 display not available!
    echo Please set up X11 display server first.
    echo You can use setup_x11.bat for help.
    echo.
    pause
    exit /b 1
)

rem Check ROS2 environment
echo Checking ROS2 environment...
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" > nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo.
    echo ROS2 environment not ready!
    echo Please make sure:
    echo 1. WSL2 is running
    echo 2. ROS2 Humble is installed
    echo 3. Drone system is started (1_start_system.bat)
    echo.
    echo The GUI will start but may not connect to the drone.
    timeout /t 3 > nul
)

echo.
echo Starting GUI Teleoperation System...
echo.
echo Controls:
echo   WASD: Move Forward/Left/Backward/Right
echo   RF: Move Up/Down
echo   QE: Turn Left/Right
echo   Space: Takeoff
echo   X: Land
echo   H: Return Home
echo   C: Toggle Control
echo   ESC: Emergency Land
echo.
echo Click on map to set target position
echo Use mission buttons for automated patterns
echo.

rem Store current directory to handle path issues
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_PATH=%SCRIPT_DIR%gui_teleop_control_en.py"

rem Run GUI with proper encoding and path handling
wsl bash -c "cd '$( wslpath '%SCRIPT_DIR%' )' && export DISPLAY=:0 && export LANG=en_US.UTF-8 && export LC_ALL=en_US.UTF-8 && source /opt/ros/humble/setup.bash && python3 gui_teleop_control_en.py"

if %ERRORLEVEL% neq 0 (
    echo.
    echo GUI system exited with error.
    echo.
    echo Troubleshooting:
    echo 1. Make sure X11 display server is running
    echo 2. Check if drone system is started
    echo 3. Verify ROS2 environment is properly sourced
    echo.
) else (
    echo.
    echo GUI system closed successfully.
    echo.
)

pause 
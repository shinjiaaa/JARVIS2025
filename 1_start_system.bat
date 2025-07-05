@echo off
echo ========================================
echo     🚁 Drone System Startup
echo ========================================

echo Starting ROS2 and Gazebo simulation...

cd /d "%~dp0"

:: Start ROS2 and Gazebo in WSL
wsl bash -c "cd /mnt/c/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 launch launch/full_system.launch.py"

if %ERRORLEVEL% neq 0 (
    echo ❌ Failed to start system.
    echo Please check if WSL2 and ROS2 Humble are properly installed.
    pause
    exit /b 1
)

echo ✅ System startup completed.
pause 
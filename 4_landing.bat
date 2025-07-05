@echo off
echo ========================================
echo        🚁 Drone Landing
echo ========================================

echo Sending landing command to drone...

cd /d "%~dp0"

:: Send landing command via ROS2
wsl bash -c "cd /mnt/c/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub --once /ppo_target geometry_msgs/msg/Point 'x: 0.0, y: 0.0, z: 0.0'"

echo ✅ Landing command sent successfully.
echo The drone should now be landing.

pause 
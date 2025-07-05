@echo off
echo ========================================
echo        🚁 Drone Takeoff
echo ========================================

echo Sending takeoff command to drone...

cd /d "%~dp0"

:: Send takeoff command via ROS2
wsl bash -c "cd /mnt/c/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub --once /ppo_enable std_msgs/msg/Bool 'data: true' && ros2 topic pub --once /ppo_target geometry_msgs/msg/Point 'x: 0.0, y: 0.0, z: 2.0'"

echo ✅ Takeoff command sent successfully.
echo The drone should now be taking off to 2m altitude.

pause 
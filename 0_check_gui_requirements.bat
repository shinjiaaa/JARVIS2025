@echo off
echo 🔍 GUI Teleoperation System Requirements Check
echo ================================================

echo Checking requirements before running GUI system...

pause

:: Run requirements check script in WSL
wsl bash -c "cd /mnt/c/coding/DRONE && source /opt/ros/humble/setup.bash && python3 check_gui_requirements.py"

pause

echo Requirements check completed.
echo If all requirements are met, you can run 6_gui_teleop.bat.

pause 
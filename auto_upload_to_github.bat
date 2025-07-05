@echo off
chcp 65001 > nul
cd /d "%~dp0"

echo =============================================
echo     🚁 DRONE System GitHub Auto Upload
echo =============================================

echo Uploading to GitHub repository: ThugGyu/AUTO
echo.

:: Check if git is initialized
if not exist ".git" (
    echo Initializing git repository...
    git init
    git remote add origin https://github.com/ThugGyu/AUTO.git
)

:: Add all core files
echo Adding core files...
git add README_Usage_Guide.txt
git add GUI_Manual_EN.txt
git add package.xml
git add CMakeLists.txt
git add CORE_FILES_SUMMARY.txt
git add *.bat
git add *.py
git add config/
git add launch/
git add maps/
git add models/
git add install/

:: Commit changes
echo Committing changes...
git commit -m "Upload DRONE Control System - Complete Project

🚁 ROS2 + Gazebo + PPO Reinforcement Learning Drone Control System

✨ CORE FEATURES:
- GUI-based drone teleoperation with English interface
- Keyboard and mouse control methods
- Real-time 2D map visualization and mission planning
- Safety features and boundary protection
- PPO reinforcement learning integration

🎮 SYSTEM CONTROL FILES:
- 0_check_gui_requirements.bat → GUI requirements checker
- 1_start_system.bat → System startup (Gazebo + ROS2)
- 2_takeoff.bat → Drone takeoff
- 3_keyboard_control.bat → Keyboard menu control
- 4_landing.bat → Drone landing
- 6_gui_teleop_en.bat → GUI graphical control (English, recommended)

🐍 CORE SCRIPTS:
- gui_teleop_control_en.py → GUI teleoperation control (36KB)
- advanced_teleop_control.py → Advanced teleoperation control (21KB)
- check_gui_requirements.py → GUI requirements checker

📋 PROJECT CONFIGURATION:
- package.xml → ROS2 package configuration
- CMakeLists.txt → Build configuration

📖 DOCUMENTATION:
- README_Usage_Guide.txt → Usage guide
- GUI_Manual_EN.txt → GUI user manual (English)
- CORE_FILES_SUMMARY.txt → Project file summary

📁 CONFIGURATION FOLDERS:
- config/ → Configuration files (RVIZ, etc.)
- launch/ → ROS2 launch files
- maps/ → Map files for simulation
- models/ → 3D models and meshes
- install/ → Installation and build files

🚀 USAGE ORDER:
1. 1_start_system.bat (System startup)
2. 2_takeoff.bat (Takeoff)
3. 6_gui_teleop_en.bat (GUI control - recommended)
4. 4_landing.bat (Landing)

Project cleaned and optimized for core functionality only."

:: Push to GitHub
echo Pushing to GitHub...
git branch -M main
git push -u origin main

if %ERRORLEVEL% equ 0 (
    echo ✅ Successfully uploaded to GitHub!
    echo Repository: https://github.com/ThugGyu/AUTO
) else (
    echo ❌ Upload failed. Please check:
    echo 1. Internet connection
    echo 2. GitHub credentials
    echo 3. Repository permissions
)

echo.
pause 
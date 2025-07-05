#!/usr/bin/env python3
"""
🔍 GUI Teleoperation System Requirements Check Script
Check if required libraries are installed.
"""

import sys
import subprocess
import importlib

def check_python_version():
    """Check Python version"""
    print("🐍 Checking Python version...")
    version = sys.version_info
    print(f"   Current version: Python {version.major}.{version.minor}.{version.micro}")
    
    if version.major >= 3 and version.minor >= 8:
        print("   ✅ Python version OK")
        return True
    else:
        print("   ❌ Python 3.8 or higher required")
        return False

def check_library(lib_name, import_name=None, pip_name=None):
    """Check library installation"""
    if import_name is None:
        import_name = lib_name
    if pip_name is None:
        pip_name = lib_name
        
    try:
        importlib.import_module(import_name)
        print(f"   ✅ {lib_name} installed")
        return True
    except ImportError:
        print(f"   ❌ {lib_name} installation required")
        print(f"      Install command: pip3 install {pip_name}")
        return False

def check_ros2():
    """Check ROS2 installation"""
    print("🤖 Checking ROS2...")
    try:
        importlib.import_module('rclpy')
        print("   ✅ ROS2 Python bindings installed")
        return True
    except ImportError:
        print("   ❌ ROS2 Python bindings required")
        print("      ROS2 Humble installation needed")
        return False

def check_display():
    """Check display environment"""
    print("🖥️ Checking display environment...")
    import os
    
    if 'DISPLAY' in os.environ:
        print(f"   ✅ DISPLAY set: {os.environ['DISPLAY']}")
        return True
    else:
        print("   ⚠️ DISPLAY environment variable not found")
        print("      X11 display setup may be required for GUI execution in WSL")
        print("      Install X11 server like VcXsrv or X410")
        return False

def test_tkinter():
    """Test basic Tkinter functionality"""
    print("🧪 Testing Tkinter functionality...")
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()  # Hide window
        root.destroy()
        print("   ✅ Tkinter working properly")
        return True
    except Exception as e:
        print(f"   ❌ Tkinter error: {e}")
        return False

def main():
    print("🚁 GUI Teleoperation System Requirements Check")
    print("=" * 50)
    
    all_ok = True
    
    # Check Python version
    if not check_python_version():
        all_ok = False
    print()
    
    # Check essential libraries
    print("📦 Checking essential libraries...")
    libraries = [
        ("tkinter", "tkinter"),
        ("numpy", "numpy"),
        ("threading", "threading"),
        ("time", "time"),
        ("math", "math"),
        ("signal", "signal"),
        ("collections", "collections")
    ]
    
    for lib_name, import_name in libraries:
        if not check_library(lib_name, import_name):
            all_ok = False
    print()
    
    # Check ROS2
    if not check_ros2():
        all_ok = False
    print()
    
    # Check display
    display_ok = check_display()
    print()
    
    # Test Tkinter
    if not test_tkinter():
        all_ok = False
        display_ok = False
    print()
    
    # Summary results
    print("📋 Requirements Check Results")
    print("-" * 30)
    
    if all_ok and display_ok:
        print("✅ All requirements are satisfied!")
        print("🚁 GUI teleoperation system can be executed.")
        print("\nExecution method:")
        print("1. Run 1_start_system.bat")
        print("2. Run 2_takeoff.bat") 
        print("3. Run 6_gui_teleop.bat")
    elif all_ok and not display_ok:
        print("⚠️ All libraries are installed but there may be display setup issues.")
        print("X11 server setup is required in WSL environment.")
        print("\nSolution:")
        print("1. Install VcXsrv or X410")
        print("2. Run 'export DISPLAY=:0' in WSL")
        print("3. Test again")
    else:
        print("❌ Some requirements are not satisfied.")
        print("Please install the libraries shown above and run again.")
    
    print("\n🔧 Troubleshooting:")
    print("- Python libraries: pip3 install <library_name>")
    print("- ROS2: sudo apt install ros-humble-desktop")
    print("- X11 display: Use VcXsrv, X410, or WSLg")
    
    return all_ok and display_ok

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 
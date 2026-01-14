#!/usr/bin/env python3
"""
Script to verify ROS2 package structure for E-WheelChAIr
"""

import os
import sys
from pathlib import Path

def check_package_structure(package_path, package_name):
    """Check if package has correct structure"""
    print(f"\n=== Checking {package_name} ===")
    
    # Check main files
    required_files = [
        f"{package_name}/__init__.py",
        f"{package_name}/{package_name}_node.py",
        "setup.py",
        "package.xml",
        f"resource/{package_name}"
    ]
    
    missing_files = []
    for file_path in required_files:
        full_path = package_path / file_path
        if not full_path.exists():
            missing_files.append(file_path)
            print(f"❌ Missing: {file_path}")
        else:
            print(f"✅ Found: {file_path}")
    
    # Check config files
    config_dir = package_path / "config"
    if config_dir.exists():
        print(f"✅ Config directory exists")
        for config_file in config_dir.glob("*.yaml"):
            print(f"  ✅ Config file: {config_file.name}")
    else:
        print(f"❌ Config directory missing")
    
    # Check launch files
    launch_dir = package_path / "launch"
    if launch_dir.exists():
        print(f"✅ Launch directory exists")
        for launch_file in launch_dir.glob("*.launch.py"):
            print(f"  ✅ Launch file: {launch_file.name}")
    else:
        print(f"❌ Launch directory missing")
    
    return len(missing_files) == 0

def main():
    base_path = Path("src")
    
    packages = [
        (base_path / "teleop_joystick", "teleop_joystick"),
        (base_path / "servo_controller", "servo_controller")
    ]
    
    all_ok = True
    for package_path, package_name in packages:
        if not check_package_structure(package_path, package_name):
            all_ok = False
    
    if all_ok:
        print("\n✅ All packages have correct structure!")
        return 0
    else:
        print("\n❌ Some packages have missing files!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
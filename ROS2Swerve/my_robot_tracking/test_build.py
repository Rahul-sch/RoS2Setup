#!/usr/bin/env python3

import subprocess
import sys
import os

def test_build():
    """Test if the package builds successfully"""
    print("Testing ROS2 package build...")
    
    # We're already in the package directory
    # os.chdir('my_robot_tracking')
    
    try:
        # Try to build the package
        result = subprocess.run([
            'colcon', 'build', '--packages-select', 'my_robot_tracking'
        ], capture_output=True, text=True, timeout=60)
        
        if result.returncode == 0:
            print("✅ Package builds successfully!")
            print("Build output:")
            print(result.stdout)
            return True
        else:
            print("❌ Build failed!")
            print("Error output:")
            print(result.stderr)
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ Build timed out")
        return False
    except FileNotFoundError:
        print("❌ colcon not found - make sure ROS2 is installed")
        return False
    except Exception as e:
        print(f"❌ Build error: {e}")
        return False

if __name__ == '__main__':
    success = test_build()
    sys.exit(0 if success else 1)

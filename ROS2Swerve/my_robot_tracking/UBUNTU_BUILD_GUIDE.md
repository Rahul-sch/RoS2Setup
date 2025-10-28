# 🔧 Ubuntu Build Troubleshooting Guide

## Issue: "No executable found"

This happens when ROS2 can't find your node executables. Follow these steps **exactly** on your Ubuntu system.

---

## 📋 Step-by-Step Fix (Run on Ubuntu)

### Step 1: Navigate to Package
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
pwd  # Should show: /home/YOUR_USERNAME/Ros2Setup/ROS2Swerve/my_robot_tracking
```

### Step 2: Check File Structure
```bash
ls -la src/my_robot_tracking/my_robot_tracking/
```

**✅ You should see:**
```
camera_node.py
tracking_node.py
hardware_node.py
teleop_node.py
__init__.py
scripts/
```

**❌ If you DON'T see these files:**
```bash
# Pull from git again
cd ~/Ros2Setup/ROS2Swerve
git pull origin main
cd my_robot_tracking
```

### Step 3: Verify __init__.py Files Exist
```bash
cat src/my_robot_tracking/my_robot_tracking/__init__.py
cat src/my_robot_tracking/my_robot_tracking/scripts/__init__.py
```

**Both files should exist (even if empty)**

**If missing, create them:**
```bash
touch src/my_robot_tracking/my_robot_tracking/__init__.py
touch src/my_robot_tracking/my_robot_tracking/scripts/__init__.py
```

### Step 4: Clean Everything
```bash
rm -rf build install log
```

### Step 5: Build with Verbose Output
```bash
colcon build --packages-select my_robot_tracking --symlink-install --event-handlers console_direct+
```

**✅ SUCCESS looks like:**
```
Starting >>> my_robot_tracking
...
Finished <<< my_robot_tracking [X.XXs]
Summary: 1 package finished
```

**❌ If you see errors, copy/paste them!**

### Step 6: Source the Workspace
```bash
source install/setup.bash
```

**IMPORTANT:** You must source **THIS workspace**, not just `/opt/ros/humble/setup.bash`!

### Step 7: Verify Executables Are Installed
```bash
ros2 pkg executables my_robot_tracking
```

**✅ You should see:**
```
my_robot_tracking camera_node
my_robot_tracking hardware_node
my_robot_tracking object_selector
my_robot_tracking teleop_node
my_robot_tracking tracking_node
```

**❌ If you see nothing, the build failed silently. Check Step 5 output.**

### Step 8: Test Camera Node
```bash
ros2 run my_robot_tracking camera_node
```

**✅ SUCCESS:**
```
[INFO] [camera_node]: Camera node started
```

**❌ "No executable found":**
Go back to Step 6 and make sure you sourced `install/setup.bash`

---

## 🐛 Common Issues & Fixes

### Issue 1: "No executable found" after building successfully
**Cause:** Didn't source the workspace

**Fix:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
source install/setup.bash  # Do this in EVERY new terminal!
```

### Issue 2: "Package 'my_robot_tracking' not found"
**Cause:** Wrong directory or didn't build

**Fix:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
colcon build --packages-select my_robot_tracking
source install/setup.bash
```

### Issue 3: Build completes but no executables
**Cause:** Missing `__init__.py` or wrong file structure

**Fix:**
```bash
# Check structure
ls src/my_robot_tracking/my_robot_tracking/*.py

# Should show:
# camera_node.py
# tracking_node.py
# hardware_node.py
# teleop_node.py

# If files are in wrong place:
cd src/my_robot_tracking
mkdir -p my_robot_tracking/scripts
# Move files if needed
```

### Issue 4: "error: can't copy 'resource/my_robot_tracking'"
**Cause:** Missing resource file

**Fix:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking/src/my_robot_tracking
mkdir -p resource
touch resource/my_robot_tracking
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
colcon build --packages-select my_robot_tracking
```

---

## 🎯 Quick Diagnostic Script

Run this to diagnose all issues at once:

```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking

echo "=== Checking File Structure ==="
ls -la src/my_robot_tracking/my_robot_tracking/*.py 2>/dev/null && echo "✅ Nodes found" || echo "❌ Nodes missing"

echo ""
echo "=== Checking __init__.py ==="
test -f src/my_robot_tracking/my_robot_tracking/__init__.py && echo "✅ __init__.py exists" || echo "❌ Missing __init__.py"

echo ""
echo "=== Checking Resource File ==="
test -f src/my_robot_tracking/resource/my_robot_tracking && echo "✅ Resource exists" || echo "❌ Missing resource"

echo ""
echo "=== Checking Build ==="
test -d install && echo "✅ Install directory exists" || echo "❌ Not built yet"

echo ""
echo "=== Checking Executables ==="
source install/setup.bash 2>/dev/null
ros2 pkg executables my_robot_tracking 2>/dev/null | grep -q camera_node && echo "✅ Executables installed" || echo "❌ No executables found"
```

---

## 📝 The Correct Workflow (Every Time)

**After pulling code from Git:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
rm -rf build install log               # Clean
colcon build --packages-select my_robot_tracking --symlink-install  # Build
source install/setup.bash               # Source
ros2 run my_robot_tracking camera_node  # Run
```

**In a new terminal (to run another node):**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
source install/setup.bash               # Must source again!
ros2 run my_robot_tracking tracking_node
```

**OR use launch file to run everything:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
source install/setup.bash
ros2 launch my_robot_tracking tracking_system.launch.py
```

---

## 🆘 If Still Broken

**Copy/paste the output of these commands:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
pwd
ls -la src/my_robot_tracking/
ls -la src/my_robot_tracking/my_robot_tracking/
cat src/my_robot_tracking/setup.py
colcon build --packages-select my_robot_tracking --event-handlers console_direct+ 2>&1 | tail -20
source install/setup.bash
ros2 pkg executables my_robot_tracking
```

Send me that output and I'll tell you exactly what's wrong! 🔍


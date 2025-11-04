#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_tracking')
    
    # Declare launch arguments
    use_pyqt_arg = DeclareLaunchArgument(
        'use_pyqt',
        default_value='true',
        description='Launch PyQt UI'
    )
    
    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Enable camera node'
    )
    
    tracking_enabled_arg = DeclareLaunchArgument(
        'tracking_enabled',
        default_value='true',
        description='Enable tracking node'
    )
    
    hardware_enabled_arg = DeclareLaunchArgument(
        'hardware_enabled',
        default_value='true',
        description='Enable hardware interface node'
    )
    
    teleop_enabled_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Enable teleop node'
    )
    
    lidar_enabled_arg = DeclareLaunchArgument(
        'lidar_enabled',
        default_value='true',
        description='Enable LIDAR guard node'
    )
    
    # Camera node
    camera_node = Node(
        package='my_robot_tracking',
        executable='camera_node',
        name='camera_node',
        condition=IfCondition(LaunchConfiguration('camera_enabled')),
        output='screen'
    )
    
    # Hardware node
    hardware_node = Node(
        package='my_robot_tracking',
        executable='hardware_node',
        name='hardware_node',
        condition=IfCondition(LaunchConfiguration('hardware_enabled')),
        output='screen'
    )
    
    # Tracking node
    tracking_node = Node(
        package='my_robot_tracking',
        executable='tracking_node',
        name='tracking_node',
        condition=IfCondition(LaunchConfiguration('tracking_enabled')),
        output='screen'
    )
    
    # Teleop node
    teleop_node = Node(
        package='my_robot_tracking',
        executable='teleop_node',
        name='teleop_node',
        condition=IfCondition(LaunchConfiguration('teleop_enabled')),
        output='screen'
    )
    
    # LIDAR guard node
    lidar_node = Node(
        package='my_robot_tracking',
        executable='lidar_guard',
        name='lidar_guard',
        condition=IfCondition(LaunchConfiguration('lidar_enabled')),
        output='screen'
    )
    
    # PyQt UI (launch as separate process)
    pyqt_ui = ExecuteProcess(
        cmd=['ros2', 'run', 'my_robot_tracking', 'pyqt_ui'],
        condition=IfCondition(LaunchConfiguration('use_pyqt')),
        output='screen'
    )
    
    return LaunchDescription([
        use_pyqt_arg,
        camera_enabled_arg,
        tracking_enabled_arg,
        hardware_enabled_arg,
        teleop_enabled_arg,
        lidar_enabled_arg,
        camera_node,
        hardware_node,
        tracking_node,
        teleop_node,
        lidar_node,
        pyqt_ui,
    ])


#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_tracking')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_enabled = LaunchConfiguration('camera_enabled')
    tracking_enabled = LaunchConfiguration('tracking_enabled')
    hardware_enabled = LaunchConfiguration('hardware_enabled')
    teleop_enabled = LaunchConfiguration('teleop_enabled')
    
    # Camera node
    camera_node = Node(
        package='my_robot_tracking',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_width': 640,
            'frame_height': 480,
            'fps': 30
        }],
        condition=IfCondition(camera_enabled)
    )
    
    # Tracking node
    tracking_node = Node(
        package='my_robot_tracking',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_speed': 100,
            'center_threshold': 30.0,
            'angle_send_threshold': 5.0,
            'steering_delay': 0.3,
            'max_distance': 200.0
        }],
        condition=IfCondition(tracking_enabled)
    )
    
    # Hardware interface node
    hardware_node = Node(
        package='my_robot_tracking',
        executable='hardware_node',
        name='hardware_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': '/dev/ttyACM0',
            'serial_baud': 115200,
            'max_speed': 100,
            'wheel_drive_dir': [1, 1, 1, 1],
            'steer_dir': [1, 1, 1, 1],
            'steer_offsets': [0, 0, 0, 0]
        }],
        condition=IfCondition(hardware_enabled)
    )
    
    # Teleop node
    teleop_node = Node(
        package='my_robot_tracking',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_speed': 100,
            'deadzone': 0.18,
            'max_angular': 1.0
        }],
        condition=IfCondition(teleop_enabled)
    )
    
    # Joy node for Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dev': '/dev/input/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0
        }],
        condition=IfCondition(teleop_enabled)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        camera_enabled_arg,
        tracking_enabled_arg,
        hardware_enabled_arg,
        teleop_enabled_arg,
        camera_node,
        tracking_node,
        hardware_node,
        teleop_node,
        joy_node,
    ])

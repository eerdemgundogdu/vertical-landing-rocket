#!/usr/bin/env python3
"""ROS2 launch file for Vertical Landing Rocket."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for rocket guidance system."""
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Teensy communication'
    )
    
    gcs_ip_arg = DeclareLaunchArgument(
        'gcs_ip',
        default_value='192.168.1.100',
        description='Ground Control Station IP address'
    )
    
    # Serial Bridge Node
    serial_bridge_node = Node(
        package='rocket_guidance',
        executable='serial_bridge.py',
        name='serial_bridge',
        parameters=[{
            'port': LaunchConfiguration('serial_port'),
            'baudrate': 921600
        }],
        output='screen'
    )
    
    # Guidance Node
    guidance_node = Node(
        package='rocket_guidance',
        executable='guidance_node.py',
        name='guidance_node',
        parameters=[{
            'update_rate': 50.0,
            'landing_target_n': 0.0,
            'landing_target_e': 0.0,
            'max_tilt_angle': 0.5,
            'terminal_altitude': 5.0,
            'terminal_velocity': 2.0
        }],
        output='screen'
    )
    
    # Vision Node
    vision_node = Node(
        package='rocket_guidance',
        executable='vision_node.py',
        name='vision_node',
        parameters=[{
            'aruco_dict': 'DICT_4X4_50',
            'marker_size': 0.2,
            'camera_topic': '/camera/image_raw',
            'publish_debug': True
        }],
        output='screen'
    )
    
    # Telemetry Node
    telemetry_node = Node(
        package='rocket_guidance',
        executable='telemetry_node.py',
        name='telemetry_node',
        parameters=[{
            'gcs_ip': LaunchConfiguration('gcs_ip'),
            'gcs_port': 14550,
            'broadcast_rate': 20.0,
            'json_mode': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        gcs_ip_arg,
        serial_bridge_node,
        guidance_node,
        vision_node,
        telemetry_node
    ])

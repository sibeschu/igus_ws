#!/usr/bin/env python3

"""
Launch File für das Roboter Template
Startet move_group und bereitet alles für das Student Template vor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    # Deklariere Launch-Argumente
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    hardware_protocol = LaunchConfiguration('hardware_protocol', default='cri')
    
    # Pfad zur MoveIt Config
    moveit_config_package = FindPackageShare('igus_rebel_moveit_config')
    
    # Include move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                moveit_config_package,
                'launch',
                'move_group.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'hardware_protocol': hardware_protocol,
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (set to true when using Gazebo)'
        ),
        DeclareLaunchArgument(
            'hardware_protocol',
            default_value='cri',
            description='Hardware protocol (cri for real robot, mock for testing)'
        ),
        
        # Start move_group
        move_group_launch,
        
        # Informative message
        Node(
            package='sample_package',
            executable='roboter_template',
            name='roboter_template',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # Add a delay to ensure move_group is ready
            # This is done via arguments, not prefix
        ),
    ])

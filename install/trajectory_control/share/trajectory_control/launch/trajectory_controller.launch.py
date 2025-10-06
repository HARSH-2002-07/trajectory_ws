#!/usr/bin/env python3
"""
Launch file for trajectory control system with TurtleBot3 simulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_trajectory_control = get_package_share_directory('trajectory_control')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty_world')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='World file name')
    
    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Trajectory controller node
    trajectory_controller = Node(
        package='trajectory_control',
        executable='trajectory_controller_node',
        name='trajectory_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('trajectory_control'),
                'config',
                'controller_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', '/odom'),
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # RViz for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('trajectory_control'),
        'config',
        'trajectory_control.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Waypoint publisher (for testing)
    waypoint_publisher = Node(
        package='trajectory_control',
        executable='waypoint_publisher.py',
        name='waypoint_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        turtlebot3_gazebo,
        trajectory_controller,
        rviz,
        waypoint_publisher,
    ])

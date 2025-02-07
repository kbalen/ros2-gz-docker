#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument



def get_turtlebot3_world():
    pkg_dir = get_package_share_directory("turtlebot3_gazebo")



def generate_launch_description():
    pkg_dir = get_package_share_directory("rb-car-sim")
    model_path = os.path.join(pkg_dir, 'models', 'model.sdf')
    world_path = os.path.join(pkg_dir, 'worlds', 'turtlebot3_world', 'model.sdf')
    
    # Launch Gazebo with verbose output
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg_dir, 'launch', 'gazebo.launch.py')
    launch_description_source = PythonLaunchDescriptionSource([gazebo_launch_file])
    gazebo = IncludeLaunchDescription(
        launch_description_source,
        launch_arguments={'world': world_path,'verbose': 'true'}.items()
    )

    # Let us spawn a robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable="spawn_entity.py",
        arguments=['-entity', 'rb_car', '-file', model_path],
        output='screen'
    )

    # # Delay the spawn_entity
    # delayed_spawn = TimerAction(
    #     period=5.0,  # 5 second delay
    #     actions=[spawn_entity]
    # )

    return LaunchDescription([gazebo, spawn_entity])


# def generate_launch_description_old():
#     # Get the package directories
#     pkg_dir = get_package_share_directory('rb-car-sim')
#     turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
#     # Set paths for models
#     gazebo_models_path = os.path.join(pkg_dir, 'models')
    
#     # Model path
#     model_path = os.path.join(gazebo_models_path, 'model.sdf')
    
#     # Use TurtleBot3 world file
#     world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    
#     # Gazebo launch
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
#         launch_arguments={'world': world_path}.items()
#     )
    
#     # Spawn the robot
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'rb_car', '-file', model_path],
#         output='screen'
#     )
    
#     return LaunchDescription([
#         gazebo,
#         spawn_entity,
#     ]) 
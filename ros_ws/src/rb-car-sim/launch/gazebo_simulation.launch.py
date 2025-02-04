#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('rb-car-sim')
    
    # Set paths for models and worlds
    gazebo_models_path = os.path.join(pkg_dir, 'models')
    gazebo_worlds_path = os.path.join(pkg_dir, 'worlds')
    
    # Model path
    model_path = os.path.join(gazebo_models_path, 'model.sdf')
    
    # World path (assuming you have a .world file in the worlds directory)
    world_path = os.path.join(gazebo_worlds_path, 'turtlebot3_world')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rb_car', '-file', model_path],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
    ]) 
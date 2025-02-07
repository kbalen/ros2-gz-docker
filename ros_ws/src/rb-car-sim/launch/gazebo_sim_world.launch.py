from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription



def generate_world_launch_description():
    # Launch Gazebo
    world_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage2.world')
    gazebo_ros = os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gazebo.launch.py')
    gazebo_ros_launch = PythonLaunchDescriptionSource([gazebo_ros])
    gazebo_ros_launch = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    return gazebo_ros_launch



def generate_launch_description():
    
    world_launch_description = generate_world_launch_description()
    pkg_dir = get_package_share_directory("rb-car-sim")
    model_path = os.path.join(pkg_dir, 'models', 'model.sdf')
    # Let us spawn a robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable="spawn_entity.py",
        arguments=['-entity', 'rb_car', '-file', model_path],
        output='screen'
    )

    return LaunchDescription([world_launch_description, spawn_entity])
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare(package='car_trailer').find('car_trailer')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Set Gazebo model path so it can find the meshes
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        pkg_share
    )
    
    # Paths to necessary files
    urdf_file = os.path.join(pkg_share, 'urdf', 'car_2trailers.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Read and process URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Gazebo launch with our custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'car_2trailers',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])

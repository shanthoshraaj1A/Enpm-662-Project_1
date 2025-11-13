import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare(package='car_trailer').find('car_trailer')
    
    # Paths to necessary files
    urdf_file = os.path.join(pkg_share, 'urdf', 'car_2trailers.urdf.xacro')
    
    # Read and process URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
        }]
    )
    
    # Joint State Publisher (without GUI to avoid library conflicts)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'view_robot.rviz')) else []
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    resources_package = 'diff_drive'
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # URDF file path
    urdf_file = os.path.join(get_package_share_directory(resources_package), 'urdf', 'Robot.urdf')
    
    # CRITICAL FIX: Pass robot_description as parameter, not topic
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher (still needed for visualization)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Controller Manager - FIXED: Direct parameter instead of topic subscription
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {
                'robot_description': robot_desc,  # CRITICAL: Direct parameter
                'use_sim_time': use_sim_time
            },
            os.path.join(get_package_share_directory(resources_package), 'overcross', 'Robot_controllers.yaml')
        ],
        output='screen',
    )

    # Spawner with delay
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # Wait for controller_manager to fully initialize
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    diff_drive_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])

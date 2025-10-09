import os
import sys

import launch

from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():
    # Your package name

    pkg_name = 'navigation'
    share_dir = FindPackageShare(pkg_name)

    config_path = PathJoinSubstitution([share_dir, 'config'])
    
    # World file path - using terrainxl file

    terrain_xl_dir = FindPackageShare('terrainxl')

    world_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'earthsmall.sdf'])
    print(world_path)
    # SJTU drone URDF processing
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    xacro_file_name = "scout.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory('navigation'),
        "models", "scout", xacro_file_name
    )
    
    # Process xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    model_ns = "drone"

    # Alternative method using Command (in case xacro.process_file has issues)
    robot_description_alt = ParameterValue(
        Command(['xacro', xacro_file]),
        value_type=str
    )

    # Start Ignition Gazebo with your world
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', world_path],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Spawn the drone in Ignition Gazebo
    spawn_drone = TimerAction(
        period=3.0,  # Wait for Gazebo to be ready
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                name='spawn_drone',
                output='screen',
                arguments=['-name', 'scout', '-topic', 'robot_description', '-y', '-7946.740000', '-x', '-178.273000', '-z', '567.6510']
            )
        ]
    )

    #Launch ROS bridge

    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )

    # Optional: RViz for visualization (remove if you don't have a config file)
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='navigation_viz',
    #     output={'both': 'log'},
    #     arguments=['-d', os.path.join(get_package_share_directory('navigation'), 'rviz', 'navigation.rviz')]
    # )

    Node(
        package='navigation',
        executable='pose_to_odom',
        name='pose_to_odom',
        output='screen',
    ),


    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Set environment variable for Gazebo to find your models/worlds
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [share_dir]),
          
        ignition_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_drone,
        gazebo_bridge,
        
        # rviz,  # Uncomment if you have RViz config
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()

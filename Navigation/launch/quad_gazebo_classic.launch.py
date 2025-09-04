from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    hector_pkg = get_package_share_directory('hector_quadrotor_gazebo')
    world = os.path.join(hector_pkg, 'worlds', 'empty.world')

    return LaunchDescription([
        # Gazebo Classic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items()
        ),

        # Spawn a basic hector quadrotor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hector_pkg, 'launch', 'spawn_quadrotor_empty_world.launch.py')),
            # remap topics to match your controller
            launch_arguments={
                'model': 'quadrotor',    # default model
                # Your controller publishes drone/cmd_vel and expects odom on /drone/gt_odom
            }.items()
        ),

        # Your controller (remaps to match hector topics)
        Node(
            package='navigation',
            executable='navigation_node',
            output='screen',
            remappings=[
                ('/drone/gt_odom', '/ground_truth/state'),  # hector publishes this
                ('drone/cmd_vel',  '/cmd_vel')              # hector consumes this
            ],
        )
    ])

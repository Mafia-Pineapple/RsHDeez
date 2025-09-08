from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Your package (contains worlds/ and this launch file)
    pkg_name = 'navigation'
    share_dir = FindPackageShare(pkg_name)

    # World file inside your package
    world_path = PathJoinSubstitution([share_dir, 'worlds', 'demo.world.sdf'])

    # ---- Expand SJTU drone xacro into a URDF string ----
    sjtu_share = get_package_share_directory('sjtu_drone_description')
    xacro_file = os.path.join(sjtu_share, 'urdf', 'sjtu_drone.urdf.xacro')

    # Call xacro as a command - use 'xacro' instead of full path
    robot_description = ParameterValue(
        Command(['xacro', xacro_file]),
        value_type=str
    )

    # Node: publish TF tree from URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Start Gazebo (Ignition / GZ) with your world
    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', world_path],
        output='screen'
    )

    # Spawn the URDF into Gazebo using ros_gz_sim's spawner
    spawn = TimerAction(
        period=2.0,  # wait a bit so the sim is ready
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_drone',
                output='screen',
                arguments=['-name', 'drone', '-topic', 'robot_description']
            )
        ]
    )

    return LaunchDescription([
        # Make your package resources (models/worlds) visible to GZ
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [share_dir]),

        gz,
        rsp,
        spawn,
    ])
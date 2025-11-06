import os
import launch
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_name = 'navigation'
    share_dir = FindPackageShare(pkg_name)
    config_path = PathJoinSubstitution([share_dir, 'config'])
    terrain_xl_dir = FindPackageShare('terrainxl')
    world_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'earthsmall.sdf'])
    gui_path   = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'gui.config'])
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    xacro_file = os.path.join(
        get_package_share_directory('navigation'),
        "models", "scout", "scout.urdf.xacro"
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    # -------------------- ENV PATHS --------------------
    set_paths_ign = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        [
            FindPackageShare('navigation'), ':',
            FindPackageShare('terrainxl'),  ':',
            '/home/student/41068_ws/src/RsHDeez/terrainxl/src/models'
        ]
    )
    set_paths_gz = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [
            FindPackageShare('navigation'), ':',
            FindPackageShare('terrainxl'),  ':',
            '/home/student/41068_ws/src/RsHDeez/terrainxl/src/models'
        ]
    )

    # -------------------- IGNITION SIM --------------------
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', world_path, '--gui-config', gui_path],
        output='screen'
    )

    # -------------------- ROBOT NODES --------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
    )
    spawn_drone = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_drone',
            output='screen',
            arguments=[
                '-name','scout',
                '-topic','robot_description',
                '-y','-7938.0000',
                '-x','-183.5000',
                '-z','574.6510'
            ],
        )]
    )

    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'slam_toolbox.yaml']),
                    {'use_sim_time': use_sim_time}],
    )
    quadcopter_node = Node(
        package='navigation',
        executable='quadcopter_node',
        name='quadcopter_controller',
        output='screen',
    )
    agl_parser = Node(
        package='navigation',
        executable='agl_parser',
        name='agl_parser',
        output='screen',
    )
    odometry_offset_node = Node(
        package='navigation',
        executable='odometry_offset_node',
        name='odometry_offset',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'offset_x': -178.273},
            {'offset_y': -7946.740},
            {'offset_z': 567.651}
        ]
    )

    # -------------------- SPAWN ANIMALS --------------------
    spawn_animals = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    '/home/student/41068_ws/src/RsHDeez/terrainxl/src/spawn_random_animals.py'
                ],
                output='screen'
            )
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='true', description='Use simulation time'
        ),
        set_paths_ign,
        set_paths_gz,
        ignition_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_drone,
        gazebo_bridge,
        agl_parser,
        quadcopter_node,
        slam_toolbox,
        odometry_offset_node,
        spawn_animals,
    ])

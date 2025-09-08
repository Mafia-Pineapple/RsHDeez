from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # args
    world = LaunchConfiguration('world')
    model_name = LaunchConfiguration('model_name')
    spawn_x = LaunchConfiguration('x'); spawn_y = LaunchConfiguration('y'); spawn_z = LaunchConfiguration('z')

    pkg_name = 'TerrainXL'  # must match <name> in TerrainXL/package.xml
    pkg_share = FindPackageShare(pkg_name)
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world])

    # inline SDF for a minimal “drone” model
    quad_inline_sdf = (
        '<?xml version="1.0"?>'
        '<sdf version="1.7">'
        '  <model name="quad">'
        '    <link name="base">'
        '      <inertial><mass>1.0</mass></inertial>'
        '      <collision name="c"><geometry><box><size>0.3 0.3 0.1</size></box></geometry></collision>'
        '      <visual name="v"><geometry><box><size>0.3 0.3 0.1</size></box></geometry></visual>'
        '    </link>'
        '  </model>'
        '</sdf>'
    )

    return LaunchDescription([
        # defaults
        DeclareLaunchArgument('world', default_value='world_big_demo.sdf'),
        DeclareLaunchArgument('model_name', default_value='quad'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='1.0'),

        # Let Gazebo find package resources
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [pkg_share]),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', [pkg_share]),

        # Start Ignition/Gazebo Sim
        ExecuteProcess(cmd=['ign', 'gazebo', '-v', '3', world_path], output='screen'),

        # Spawn the drone (wait ~2s so sim is ready)
        TimerAction(period=2.0, actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                output='screen',
                arguments=[
                    '-string', quad_inline_sdf,
                    '-name', model_name,
                    '-x', spawn_x, '-y', spawn_y, '-z', spawn_z
                ]
            )
        ]),
    ])

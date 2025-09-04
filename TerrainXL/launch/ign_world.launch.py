from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = LaunchConfiguration('world', default='large_demo.sdf')

    pkg_share = FindPackageShare('TerrainXL')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world])

    # Add TerrainXL/share to GZ_SIM_RESOURCE_PATH for relative assets
    resource_path = FindPackageShare('TerrainXL')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[resource_path]
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '3', world_path],
            output='screen'
        ),
    ])

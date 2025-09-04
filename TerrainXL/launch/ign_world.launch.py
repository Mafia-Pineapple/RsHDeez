from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = LaunchConfiguration('world', default='world_big_demo.sdf')

    pkg_name = 'TerrainXL'  # <-- change to match <name> in package.xml (e.g., 'terrainxl')

    pkg_share = FindPackageShare(pkg_name)
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world])

    return LaunchDescription([
        # Make package resources (models/meshes) discoverable by Gazebo Sim
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[pkg_share]),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[pkg_share]),  # legacy alias

        # Start Ignition/Gazebo Sim with the selected world
        ExecuteProcess(cmd=['ign', 'gazebo', '-v', '3', world_path], output='screen'),
    ])

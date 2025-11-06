from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch the intelligent autonomous explorer with all necessary parameters
    """
    
    # Declare launch arguments
    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed',
        default_value='0.8',
        description='Maximum cruise speed in m/s'
    )
    
    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude',
        default_value='3.0',
        description='Target flight altitude in meters'
    )
    
    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance',
        default_value='3.0',
        description='Critical obstacle avoidance distance in meters'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='5.0',
        description='Safe distance to start slowing down in meters'
    )
    
    bear_avoidance_distance_arg = DeclareLaunchArgument(
        'bear_avoidance_distance',
        default_value='8.0',
        description='Distance to avoid bears in meters'
    )
    
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.5',
        description='Occupancy grid resolution in meters'
    )
    
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius',
        default_value='30.0',
        description='Maximum exploration radius from start in meters'
    )
    
    # Intelligent Explorer node
    explorer_node = Node(
        package='navigation',
        executable='intelligent_explorer',
        name='intelligent_explorer',
        output='screen',
        parameters=[{
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'target_altitude': LaunchConfiguration('target_altitude'),
            'obstacle_distance': LaunchConfiguration('obstacle_distance'),
            'safe_distance': LaunchConfiguration('safe_distance'),
            'bear_avoidance_distance': LaunchConfiguration('bear_avoidance_distance'),
            'goal_tolerance': 1.0,
            'grid_resolution': LaunchConfiguration('grid_resolution'),
            'exploration_radius': LaunchConfiguration('exploration_radius'),
            'frontier_cluster_size': 3,
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        cruise_speed_arg,
        target_altitude_arg,
        obstacle_distance_arg,
        safe_distance_arg,
        bear_avoidance_distance_arg,
        grid_resolution_arg,
        exploration_radius_arg,
        explorer_node,
    ])
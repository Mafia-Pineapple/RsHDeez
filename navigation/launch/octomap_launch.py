from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.2,              # 20cm voxels
            'frame_id': 'map',
            'sensor_model/max_range': 15.0,
            'sensor_model/hit': 0.7,
            'sensor_model/miss': 0.4,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
            'filter_ground': False,         # Keep ground for terrain
            'pointcloud_min_z': -10.0,      # Include terrain below
            'pointcloud_max_z': 20.0,       # Include tall trees
            'occupancy_min_z': -10.0,
            'occupancy_max_z': 20.0,
        }],
        remappings=[
            ('cloud_in', '/rtabmap/cloud_map'),
        ]
    )
    
    return LaunchDescription([
        octomap_server,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Main RTAB-Map SLAM node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': False,
            'approx_sync': False,
            'sync_queue_size': 10,
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
            'Icp/VoxelSize': '0.05',
            'Icp/MaxTranslation': '0.5',
            'Icp/MaxRotation': '0.785',
            'Icp/CorrespondenceRatio': '0.3',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.3',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'Mem/STMSize': '30',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/CreateOccupancyGrid': 'true',
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr': '0',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/RangeMax': '15.0',
            'Grid/RangeMin': '0.2',
            'Grid/CellSize': '0.1',
            'Grid/GroundIsObstacle': 'false',
            'Grid/MaxObstacleHeight': '20.0',
            'Grid/MinGroundHeight': '-10.0',
            'Grid/MaxGroundHeight': '0.5',
            'Grid/NormalsSegmentation': 'false',
            'Grid/ClusterRadius': '0.3',
            'Grid/FlatObstacleDetected': 'false',
            'Grid/FromDepth': 'false',
            'publish_tf': True,
            'tf_delay': 0.05,
            'tf_tolerance': 0.1,
        }],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odometry'),
            ('grid_map', '/map'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # Point cloud assembler - THIS CREATES /rtabmap/cloud_map!
    point_cloud_assembler = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_clouds': 10,           # Keep last 10 scans
            'fixed_frame_id': 'map',
            'voxel_size': 0.05,         # Downsample to 5cm voxels
        }],
        remappings=[
            ('cloud', '/scan/points'),   # Input: LaserScan as PointCloud2
        ]
    )
    
    # LaserScan to PointCloud2 converter
    scan_to_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='scan_to_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'voxel_size': 0.05,
            'decimation': 1,
            'max_depth': 15.0,
            'min_depth': 0.2,
        }],
        remappings=[
            ('scan', '/scan'),
            ('cloud', '/scan/points'),
        ]
    )
    
    # Visualizer
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': False,
            'sync_queue_size': 10,
        }],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odometry'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap,
        scan_to_cloud,
        point_cloud_assembler,
        rtabmap_viz,
    ])
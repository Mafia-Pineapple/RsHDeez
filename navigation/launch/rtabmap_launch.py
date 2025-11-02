from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RTAB-Map core SLAM node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.2,  # Increase from default
            'queue_size': 50,
            'qos': 1,                         # Try different QoS

            # Memory management
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # SLAM parameters for aerial vehicle
            'RGBD/AngularUpdate': '0.05',        # Update every 3 degrees rotation
            'RGBD/LinearUpdate': '0.05',         # Update every 5cm movement
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            
            # Loop closure
            'Rtabmap/DetectionRate': '1',        # Hz for loop closure detection
            'Rtabmap/TimeThr': '700',            # Time threshold for loop closure
            
            # 3D mapping
            'Grid/3D': 'true',                   # Create 3D occupancy grid
            'Grid/RayTracing': 'true',           # Ray trace to clear space
            'Grid/RangeMax': '10.0',             # Max sensor range
            'Grid/CellSize': '0.1',              # 10cm voxels
            'Grid/ClusterRadius': '0.3',
            'Grid/GroundIsObstacle': 'false',    # Ground is not obstacle for drone
            'Grid/MaxObstacleHeight': '10.0',    # Trees can be tall
            'Grid/MinGroundHeight': '-10.0',     # Allow terrain below
            'Grid/MaxGroundHeight': '0.5',       # What counts as ground
            
            # Odometry
            'Odom/Strategy': '0',                # 0=Frame-to-Map, 1=Frame-to-Frame
            'Odom/ResetCountdown': '15',
            
            # For forest/outdoor
            'Kp/MaxFeatures': '400',
            'Kp/DetectorStrategy': '6',          # GFTT/Good features to track
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image'),
            ('scan', '/scan'),
            ('scan_cloud', '/scan/points'),      # If you add the converter
            ('odom', '/odometry')
        ],
        arguments=['--delete_db_on_start']
    )
    
    # RTAB-Map visualization (optional but helpful)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image'),
            ('scan', '/scan'),
            ('odom', '/odometry')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap,
        rtabmap_viz,
    ])
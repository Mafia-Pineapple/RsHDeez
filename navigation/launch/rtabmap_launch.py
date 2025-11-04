from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RTAB-Map core SLAM node - LIDAR ONLY
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            
            # DISABLE CAMERA
            'subscribe_depth': False,           # Changed to False
            'subscribe_rgb': False,             # Changed to False
            'subscribe_scan': True,             # Keep LIDAR
            'subscribe_scan_cloud': False,      # Changed to False (no converter yet)
            
            'approx_sync': False,               # Not needed without camera
            'queue_size': 10,
            
            # USE PROVIDED ODOMETRY (don't compute visual odom)
            'Odom/Strategy': '0',               # Use external odometry
            'Reg/Strategy': '1',                # ICP registration (LIDAR-based)
            'Reg/Force3DoF': 'false',           # Allow 6DOF for drone
            
            # ICP PARAMETERS
            'Icp/VoxelSize': '0.1',
            'Icp/MaxTranslation': '2.0',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '5',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            
            # MEMORY MANAGEMENT
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # SLAM PARAMETERS
            'RGBD/AngularUpdate': '0.01',        # Update every 6 degrees
            'RGBD/LinearUpdate': '0.01',         # Update every 10cm
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            
            # LOOP CLOSURE
            'Rtabmap/DetectionRate': '1.0',     # Check for loops every second
            'Rtabmap/TimeThr': '0',
            
            # 3D MAPPING FROM LIDAR
            'Grid/3D': 'true',                  # Create 3D occupancy grid
            'Grid/RayTracing': 'true',          # Ray trace to clear space
            'Grid/RangeMax': '15.0',            # Max LIDAR range
            'Grid/CellSize': '0.1',             # 10cm voxels
            'Grid/GroundIsObstacle': 'false',   # Ground is not obstacle for drone
            'Grid/MaxObstacleHeight': '20.0',   # Trees can be tall
            'Grid/MinGroundHeight': '-10.0',    # Allow terrain below
            'Grid/FromDepth': 'false',          # Don't use depth camera
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry'),
            ('grid_map', '/map'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # RTAB-Map visualization - LIDAR ONLY
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            
            # DISABLE CAMERA VISUALIZATION
            'subscribe_depth': False,           # Changed to False
            'subscribe_rgb': False,             # Changed to False
            'subscribe_scan': True,             # Keep LIDAR
            'approx_sync': False,
            'queue_size': 10,
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry'),
            ('grid_map', '/map'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap,
        rtabmap_viz,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            
            # Subscriptions (native ROS params - use Python types)
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': False,
            
            'approx_sync': False,
            'sync_queue_size': 10,
            
            # Use external odometry (from your ICP node)
            'Odom/Strategy': '0',  # 0 = Frame-to-Map, accepts external odom
            'Odom/GuessMotion': 'true',  # Use odometry for motion prediction
            'Odom/ResetCountdown': '1',  # Reset if odometry jumps
            
            # ICP parameters
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
            
            # Registration
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            
            # Memory
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'Mem/STMSize': '30',
            
            # SLAM updates
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/CreateOccupancyGrid': 'true',
            
            # Loop closure
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr': '0',
            
            # 3D Mapping
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
            
            # TF publishing (native ROS param)
            'publish_tf': True,
            'tf_delay': 0.05,
            'tf_tolerance': 0.1,
        }],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odometry'),  # CRITICAL: Map to your odometry topic
            ('grid_map', '/map'),
        ],
        arguments=['--delete_db_on_start']
    )
    
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
            ('odom', '/odometry'),  # CRITICAL: Map to your odometry topic
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap,
        rtabmap_viz,
    ])
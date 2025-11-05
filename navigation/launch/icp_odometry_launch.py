from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch RTAB-Map's ICP odometry node for LIDAR-based odometry.
    This replaces Gazebo's ground-truth odometry with sensor-based odometry.
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ICP Odometry Node (replaces Gazebo odometry)
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            
            # Subscription settings (these are native ROS params, use Python types)
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'scan_voxel_size': 0.05,
            'scan_normal_k': 10,
            
            # ICP parameters (RTAB-Map expects strings!)
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/MaxTranslation': '2.0',
            'Icp/MaxRotation': '1.57',
            'Icp/CorrespondenceRatio': '0.3',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.3',
            'Icp/OutlierRatio': '0.7',
            
            # Odometry settings (RTAB-Map expects strings!)
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
            'OdomF2M/ScanSubtractRadius': '0.2',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/MaxSize': '10000',
            
            # Publishing (native ROS params, use Python types)
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'publish_null_when_lost': False,
            
            # Queue size (native ROS param)
            'queue_size': 10,
        }],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odometry'),  # Publish to /odometry
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        icp_odometry,
    ])
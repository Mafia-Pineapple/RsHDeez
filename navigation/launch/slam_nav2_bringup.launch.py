# slam_nav2_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time   = LaunchConfiguration('use_sim_time', default='true')
    enable_viz     = LaunchConfiguration('enable_viz', default='true')
    fuse_with_scan = LaunchConfiguration('fuse_with_scan', default='true')

    # --- STATIC TFs for the camera (fix optical frame so depth isn't sideways) ---
    base_to_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_camera',
        arguments=[
            '--x','0.10','--y','0.0','--z','0.15',
            '--roll','0','--pitch','0','--yaw','0',
            '--frame-id','base_link','--child-frame-id','camera_link'
        ],
        output='screen'
    )
    cam_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_to_optical',
        arguments=[
            '--x','0','--y','0','--z','0',
            # -90° about X, then -90° about Z (REP-105 optical: Z fwd, X right, Y down)
            '--roll','-1.5707963','--pitch','0','--yaw','-1.5707963',
            '--frame-id','camera_link','--child-frame-id','camera_optical_frame'
        ],
        output='screen'
    )

    # --- RTAB-Map (RGB-D + LiDAR fusion), publishes /map and map->odom ---
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'publish_tf': True,              # publish map->odom
            # RGB-D + LiDAR
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.2,
            'queue_size': 50,
            # 6DoF SLAM
            'Reg/Force3DoF': False,
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            # Loop closure
            'Rtabmap/DetectionRate': '1',
            'Rtabmap/TimeThr': '700',
            # Occupancy grid from depth + scan
            'Grid/3D': 'true',
            'Grid/FromDepth': 'true',
            'Grid/FromScan': 'true',
            'Grid/RayTracing': 'true',
            'Grid/RangeMax': '15.0',
            'Grid/CellSize': '0.1',
            'Grid/GroundIsObstacle': 'false',
            'Grid/MaxObstacleHeight': '20.0',
            'Grid/MinGroundHeight': '-10.0',
        }],
        remappings=[
            ('rgb/image',       '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image',     '/camera/depth/image'),
            ('scan',            '/scan'),
            ('odom',            '/odometry'),
            # RTAB-Map publishes /map (nav_msgs/OccupancyGrid) by default
        ],
        arguments=['--delete_db_on_start']
    )

    # Optional: RTAB-Map Visualizer
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image',       '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image',     '/camera/depth/image'),
            ('scan',            '/scan'),
            ('odom',            '/odometry'),
        ],
        condition=None if str(enable_viz) != 'false' else False
    )

    # --- Nav2 bringup, fed by RTAB-Map's /map (no map_server) ---
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config',
        'nav2_params.yaml'
    ])
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            # Do NOT pass a static map YAML; the StaticLayer will listen to /map
        }.items()
    )

    # Start SLAM first, give it a second to produce /map & map->odom, then Nav2
    nav2_after_slam = TimerAction(period=2.0, actions=[nav2_bringup])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_viz',   default_value='true'),
        DeclareLaunchArgument('fuse_with_scan', default_value='true'),
        base_to_cam,
        cam_to_optical,
        rtabmap,
        rtabmap_viz,
        nav2_after_slam,
    ])

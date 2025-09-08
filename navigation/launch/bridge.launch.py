from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to bridge topics between ROS2 and Ignition Gazebo
    """
    
    # Bridge for drone command velocities (ROS -> Gazebo)
    # SJTU drone uses /cmd_vel topic
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )
    
    # Bridge for drone odometry (Gazebo -> ROS)
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge', 
        name='odom_bridge',
        arguments=[
            '/model/drone/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/drone/odometry', '/drone/odom')
        ],
        output='screen'
    )
    
    # Bridge for drone pose (Gazebo -> ROS) - alternative to odometry
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge', 
        arguments=[
            '/model/drone/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'
        ],
        remappings=[
            ('/model/drone/pose', '/drone/pose')
        ],
        output='screen'
    )
    
    # Bridge for IMU data (if available)
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        remappings=[
            ('/imu', '/drone/imu')
        ],
        output='screen'
    )
    
    # Bridge for clock synchronization
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        cmd_vel_bridge,
        odom_bridge, 
        pose_bridge,
        imu_bridge,
        clock_bridge,
    ])
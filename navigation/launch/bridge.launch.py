from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Bridges between ROS 2 and Gazebo (Ignition/GZ).
    NOTE: If your world name isn't 'default', or your model/link/sensor
    differ, change the AGL GZ topic accordingly.
    """

    # /cmd_vel (ROS -> GZ). Keep bidirectional if you actually need it both ways.
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # Odometry (GZ -> ROS).
    # ⚠ If your model is 'scout' (it is, per your SDF), change '/model/drone/...' to '/model/scout/...'
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/model/scout/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/scout/odometry', '/drone/odom')
        ],
        output='screen'
    )

    # Pose (GZ -> ROS).
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            '/model/scout/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'
        ],
        remappings=[
            ('/model/scout/pose', '/drone/pose')
        ],
        output='screen'
    )

    # IMU (GZ -> ROS) — only if you have this topic in sim.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/imu', '/drone/imu')
        ],
        output='screen'
    )

    # Clock (GZ -> ROS).
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # AGL / downward laser (GZ -> ROS) — this is the important one.
    # Default GZ topic when <topic> is removed from the SDF:
    #   /world/<WORLD>/model/scout/link/body/sensor/agl/scan
    # Change 'default' if your world has a different name.
    agl_gz_topic = '/world/default/model/scout/link/body/sensor/agl/scan'
    agl_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='agl_bridge',
        arguments=[
            f'{agl_gz_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            (agl_gz_topic, '/drone/agl')
        ],
        output='screen'
    )

    return LaunchDescription([
        cmd_vel_bridge,
        odom_bridge,
        pose_bridge,
        imu_bridge,
        clock_bridge,
        agl_bridge,
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo Sim (empty world)
        ExecuteProcess(cmd=['gz', 'sim', '-v', '3', 'empty.sdf'], output='screen'),

        # Your adapter (bridge-free control + odom)
        Node(package='navigation', executable='gz_adapter', output='screen',
             parameters=[{'model_name': 'quad'}]),

        # Your controller
        Node(package='navigation', executable='navigation_node', output='screen'),
    ])

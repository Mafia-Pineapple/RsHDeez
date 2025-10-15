from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal_scan_trigger',
            executable='thermal_scan_trigger_node',
            name='thermal_scan_trigger',
            parameters=[{
                # Match your bridge topics
                'thermal_topic': '/model/scout/thermal/image',
                'rgb_topic': '/model/scout/camera',
                'save_dir': '/tmp/thermal_captures',

                # Tune for your environment
                'hotspot_threshold_c': 40.0,
                'min_blob_area_px': 150,
                'kelvin_scale': 100.0,      # Gazebo thermal often K*100 -> adjust if needed
                'publish_preview': True
            }]
        )
    ])

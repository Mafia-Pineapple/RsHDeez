import os
import sys

import csv

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
import random

import xacro

def generate_launch_description():
    james_randomiser = LaunchConfiguration('james_randomiser', default='false')
    # Your package name
    pkg_name = 'navigation'
    share_dir = FindPackageShare(pkg_name)

    config_path = PathJoinSubstitution([share_dir, 'config'])
    
    # World file path - using terrainxl file
    terrain_xl_dir = FindPackageShare('terrainxl')
    if james_randomiser:
        print("Using james randomiser to generate world file with animals")
        world_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'earthsmall_header.sdf'])
#         <include>
# 			<uri>
# 				model://models/bear
# 			</uri>
# 			<name> bear_0</name>
# 			<pose>-176.273 -7947.7400 567 0 0 0</pose>
# 		</include>
# 	</world>
# </sdf>
#spam these 
        csv_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'animallocations_0001.csv'])
        evilfakecontext = LaunchContext()
        csv_path_string = csv_path.perform(evilfakecontext)
        world_path_string = world_path.perform(evilfakecontext)
        #CSV file has header row, x y z data starts from second row
        animalx = []
        animaly = []
        animalz = []
        with open(csv_path_string) as csvDataFile:
            csvReader = csv.reader(csvDataFile)
            next(csvReader)  # Skip header row
            for row in csvReader:
                if random.uniform(0,1) < 0.03:
                    animalx.append(str(float(row[0]) - 252.5))
                    animaly.append(str(float(row[1]) - 7742.5))
                    animalz.append(str(float(row[2]) + 1))
        #correct for coordinate system differences
        #but apply nothing for now, just complete the earthsmall header sdf
        #adjustment code here
        #delete earthsmall generated first

        if os.path.exists('/tmp/earthsmall_generated.sdf'):
            try:
                os.remove('/tmp/earthsmall_generated.sdf')
            except OSError as e:
                print()
                
        
        with open('/tmp/earthsmall_generated.sdf', mode='w', newline='') as correctedFile:
            with open(world_path_string, mode='r', newline='') as headerFile:
                for line in headerFile:
                    correctedFile.write(line) # copy header
            for i in range(len(animalx)):
                correctedFile.write('    <include>\n')
                correctedFile.write('        <uri>\n')
                correctedFile.write('            model://models/bear\n')
                correctedFile.write('        </uri>\n')
                correctedFile.write(f'        <static>true</static>\n')
                correctedFile.write(f'        <name>bear_{i}</name>\n')
                correctedFile.write(f'        <pose>{animalx[i]} {animaly[i]} {animalz[i]} 0 0 {random.uniform(0, 6.283)}</pose>\n')
                correctedFile.write('    </include>\n')
            correctedFile.write('</world>\n')
            correctedFile.write('</sdf>\n')
        world_path = '/tmp/earthsmall_generated.sdf'
    else: 
        world_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'earthsmall.sdf'])
    print(world_path)

    # GUI config path - using terrainxl file
    gui_path = PathJoinSubstitution([terrain_xl_dir, 'worlds', 'gui.config'])
    print(gui_path)
    
    # SJTU drone URDF processing
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    xacro_file_name = "scout.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory('navigation'),
        "models", "scout", xacro_file_name
    )
    
    # Process xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    model_ns = "drone"

    # Alternative method using Command (in case xacro.process_file has issues)
    robot_description_alt = ParameterValue(
        Command(['xacro', xacro_file]),
        value_type=str
    )

    # Start Ignition Gazebo with your world
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', world_path, '--gui-config', gui_path],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
        PathJoinSubstitution([config_path, 'slam_toolbox.yaml']),
        {'use_sim_time': use_sim_time}],
    )   


    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
    )


    # Quadcopter controller node
    quadcopter_node = Node(
        package='navigation',
        executable='quadcopter_node',
        name='quadcopter_controller',
        output='screen',
    )


    # Spawn the drone in Ignition Gazebo
    spawn_drone = TimerAction(
        period=3.0,  # Wait for Gazebo to be ready
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                name='spawn_drone',
                output='screen',
                arguments=['-name', 'scout', '-topic', 'robot_description', '-y', '-7938.0000', '-x', '-183.5000', '-z', '574.6510'],
            )
        ]
    )

    # Launch ROS bridge
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )

    # # Pose to odom converter
    # pose_to_odom = Node(
    #     package='navigation',
    #     executable='pose_to_odom',
    #     name='pose_to_odom',
    #     output='screen',
    # )

    # AGL parser node
    agl_parser = Node(
        package='navigation',
        executable='agl_parser',
        name='agl_parser',
        output='screen',
    )
    
    # Add this node
    odometry_offset_node = Node(
        package='navigation',
        executable='odometry_offset_node',
        name='odometry_offset',
        output='screen',
        parameters=[
        {'use_sim_time': use_sim_time},
        # Optional: manually specify offset if you want
        {'offset_x': -178.273},
        {'offset_y': -7946.740},
        {'offset_z': 567.651}
        ]
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument('james_randomiser', default_value='false'),

        # Set environment variable for Gazebo to find your models/worlds
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [share_dir]),
          
        ignition_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_drone,
        gazebo_bridge,
     #   pose_to_odom,
        agl_parser,
        quadcopter_node, 
        slam_toolbox,
        odometry_offset_node,
        # rviz,  # Uncomment if you have RViz config
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()

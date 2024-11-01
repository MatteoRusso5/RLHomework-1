from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


 

def generate_launch_description():
    declared_arguments = []

    
    urdf_path = get_package_share_directory('arm_description')

    urdf_file = os.path.join(urdf_path, "urdf", "arm.urdf.xacro")

    with open(urdf_file, 'r') as infp:
        link_desc = infp.read()


    robot_description_xacro = {"robot_description": Command(['xacro ', urdf_file])}
    
   
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_xacro,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )


    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    position = [0.0, 0.0, 0.0]

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'arm',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),],
    )
 
    ign = [gazebo_ignition, gz_spawn_entity]

     

     
    nodes_to_start = [
        robot_state_publisher_node,  
        *ign,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 
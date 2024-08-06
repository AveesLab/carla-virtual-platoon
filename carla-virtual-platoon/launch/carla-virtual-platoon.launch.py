import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_nodes(context, *, num_trucks, map_name):
    nodes = []

    ros_param_file = os.path.join(
            get_package_share_directory('carla-virtual-platoon'), 
            'config', 
            'config.yaml') 
    
    sync_param_file = os.path.join(
            get_package_share_directory('carla-virtual-platoon'), 
            'config', 
            'sync_config.yaml')     

    for i in range(1, int(num_trucks) + 1):
        node = Node(
            package='carla-virtual-platoon',
            executable='main',
            name=f'bridge{i-1}',
            namespace=f'truck{i-1}',
            output='screen',
            parameters=[ros_param_file,sync_param_file],
            arguments=[
                f'--truck_id={i-1}', 
                f'--map={map_name}'
            ],
            on_exit=launch.actions.Shutdown()  
        )
        nodes.append(node)
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    map_name = LaunchConfiguration('Map').perform(context)  
    return generate_nodes(context, num_trucks=num_trucks, map_name=map_name)


def generate_launch_description():

    declare_num_trucks = DeclareLaunchArgument(
        'NumTrucks',
        default_value='1',
        description='Number of trucks'
    )

    declare_map_name = DeclareLaunchArgument(
        'Map',
        default_value='IHP',
        description='MapName'
    )

    return LaunchDescription([
        declare_num_trucks,
        declare_map_name,
        OpaqueFunction(function=launch_setup)
    ])

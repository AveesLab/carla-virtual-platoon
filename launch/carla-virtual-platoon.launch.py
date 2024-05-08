import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_nodes(context, *, num_trucks, map_name):
    nodes = []

    ros_param_file = os.path.join(
            get_package_share_directory('carla-virtual-platoon'), 
            'config', 
            'config.yaml') 

    for i in range(1, int(num_trucks) + 1):
        node = Node(
            package='carla-virtual-platoon',
            executable='main',
            name=f'main{i}',
            namespace=f'truck{i-1}',
            output='screen',
            parameters=[ros_param_file],
            arguments=[
                f'--truck_id={i-1}', 
                f'--map={map_name}'
            ]  
        )
        nodes.append(node)
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    map_name = LaunchConfiguration('MapName').perform(context)  
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
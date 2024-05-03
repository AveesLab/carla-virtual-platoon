from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_nodes(context, *, num_trucks):
    nodes = []
    for i in range(1, int(num_trucks) + 1):
        node = Node(
            package='carla-virtual-platoon',
            executable='main',
            name=f'main{i}',
            namespace=f'truck{i}',
            output='screen',
            parameters=[{'param_name': f'value{i}'}],
            arguments=[f'--truck_id={i-1}']  # 명령줄 인자 추가
        )
        nodes.append(node)
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    return generate_nodes(context, num_trucks=num_trucks)

def generate_launch_description():
    declare_num_trucks = DeclareLaunchArgument(
        'NumTrucks',
        default_value='1',
        description='Number of trucks'
    )

    return LaunchDescription([
        declare_num_trucks,
        OpaqueFunction(function=launch_setup)
    ])
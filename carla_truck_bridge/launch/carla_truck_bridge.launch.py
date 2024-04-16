from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo
import os


def generate_launch_description():
    carla_truck_bridge_pkg_prefix = get_package_share_directory('carla_truck_bridge')

    carla_truck_bridge_params_file = os.path.join(
        carla_truck_bridge_pkg_prefix,  'config', 
            'params.yaml')

    carla_truck_bridge_params = DeclareLaunchArgument(
        'carla_truck_bridge_params_file',
        default_value = carla_truck_bridge_params_file,
        description = 'Path to config file for carla_truck_bridge'
    )


    LV = Node(
        namespace='LV',
        package = 'carla_truck_bridge',
        executable = 'LV_bridge',
        name = 'LV_bridge',
        output = 'screen',
        parameters = [
            carla_truck_bridge_params_file,
            {"rgbcam_topic_name": "carla/image_raw"},
            {"radar_topic_name": "carla/radar"},
            {"lidar_topic_name": "carla/lidar"}
        ]
    )


    FV1 = Node(
        namespace='FV1',
        package = 'carla_truck_bridge',
        executable = 'FV1_bridge',
        name = 'FV1_bridge',
        output = 'screen',
        parameters = [
            carla_truck_bridge_params_file,
            {"rgbcam_topic_name": "carla/image_raw"},
            {"radar_topic_name": "carla/radar"},
            {"lidar_topic_name": "carla/lidar"}
        ]
    )

    FV2 = Node(
        namespace='FV2',
        package = 'carla_truck_bridge',
        executable = 'FV2_bridge',
        name = 'FV2_bridge',
        output = 'screen',
        parameters = [
            carla_truck_bridge_params_file,
            {"rgbcam_topic_name": "carla/image_raw"},
            {"radar_topic_name": "carla/radar"},
            {"lidar_topic_name": "carla/lidar"}
        ]
    )
    
    controller_node=Node(
        package='carla_truck_bridge', 
#       namespace='Controller', 
        name='Controller', 
        executable='controller', 
        output='screen')

    return LaunchDescription([
        LV,
        FV1,
        FV2,
        controller_node
    ])

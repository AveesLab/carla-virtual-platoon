from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    carla_truck_bridge_pkg_prefix = get_package_share_directory('carla_truck_bridge')

    carla_truck_bridge_params_file = os.path.join(
        carla_truck_bridge_pkg_prefix, 'config/params.yaml')

    carla_truck_bridge_params = DeclareLaunchArgument(
        'carla_truck_bridge_params_file',
        default_value = carla_truck_bridge_params_file,
        description = 'Path to config file for carla_truck_bridge'
    )


    LV = Node(
        package = 'carla_truck_bridge',
        executable = 'LV_bridge',
        name = 'LV_bridge',
        output = 'screen',
        parameters = [
            LaunchConfiguration('carla_truck_bridge_params_file'),
            {"rgbcam_topic_name": "LV/carla/camera"},
            {"radar_topic_name": "LV/carla/radar"},
        ]
    )


    FV1 = Node(
        package = 'carla_truck_bridge',
        executable = 'FV1_bridge',
        name = 'FV1_bridge',
        output = 'screen',
        parameters = [
            LaunchConfiguration('carla_truck_bridge_params_file'),
            {"rgbcam_topic_name": "FV1/carla/camera"},
            {"radar_topic_name": "FV1/carla/radar"},
        ]
    )
    
    return LaunchDescription([
        carla_truck_bridge_params,
        LV,
        FV1
    ])
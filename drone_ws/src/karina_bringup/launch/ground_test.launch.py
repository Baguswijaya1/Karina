import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # lidar
    pkg_lidar = get_package_share_directory("rplidar_ros")
    path_lidar = os.path.join(pkg_lidar, "launch", "view_rplidar_c1_launch.py")
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_lidar)
    )

    # avoidance
    avoidance_launch = Node(
        package='avoidance',
        executable = 'obs_avoidance',
    )

    node_list = [lidar_launch, avoidance_launch]
    return LaunchDescription(node_list)
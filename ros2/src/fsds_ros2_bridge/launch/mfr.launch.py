import launch
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node,PushRosNamespace,SetRemap
from launch.substitutions import LaunchConfiguration,TextSubstitution,PathJoinSubstitution


def generate_launch_description():

    fsds_ros2_bridge_path = get_package_share_directory('fsds_ros2_bridge')

    # launch bridge
    fsds_ros2_bridge_launch = GroupAction(
        actions=[
            SetRemap(src='/lidar/lidar',dst='/lidar/points'),
            SetRemap(src='/gps',dst='/gps/fix'),
            SetRemap(src="/testing_only/odom", dst="/ground_truth/odom"),
            IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(fsds_ros2_bridge_path + '/launch/fsds_ros2_bridge.launch.py')
            )
        ]
    )


    ld= LaunchDescription(
        [
            fsds_ros2_bridge_launch
        ]
    )

    return ld




import launch
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node,PushRosNamespace,SetRemap
from launch.substitutions import LaunchConfiguration,TextSubstitution,PathJoinSubstitution


def generate_launch_description():
    
    # launch fsds bridge
    fsds_ros2_bridge_path = get_package_share_directory('fsds_ros2_bridge')
    fsds_ros2_bridge_launch = GroupAction(
        actions=[
            SetRemap(src='/lidar/lidar',dst='/lidar/points'),
            SetRemap(src='/gps',dst='/gps/fix'),
            SetRemap(src="/testing_only/odom", dst="/odom/perfect"),
            IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(fsds_ros2_bridge_path + '/launch/fsds_ros2_bridge.launch.py')
            )
        ]
    )

    # launch foxglove bridge
    foxglove_path = get_package_share_directory("foxglove_bridge")
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_path + "/launch/foxglove_bridge_launch.xml")
    )

    # point cloud to laserscan
    pointcloud_to_laserscan_node = Node(
        name="pcl_to_scan_node",
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", "/lidar/points")
        ]
    )

    # slam toolbox
    slam_toolbox_params = get_package_share_directory("mfr") + "/config/slam_toolbox_params.yaml"
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("slam_toolbox") + "/launch/online_async_launch.py"),
        launch_arguments={"params_file": slam_toolbox_params}.items()
    )

    static_tf_node  = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "0", "0", "0", "0", "base_link", "fsds/FSCar"])
    
    odom_to_tf_params = get_package_share_directory("mfr") + "/config/odom_to_tf_params.yaml"
    odom_to_tf_node = Node(
        name="odom_to_tf_node",
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        parameters=[odom_to_tf_params]
    )


    ld= LaunchDescription(
        [
            fsds_ros2_bridge_launch,
            foxglove_launch,
            pointcloud_to_laserscan_node,
            slam_toolbox_launch,
            static_tf_node,
            odom_to_tf_node
        ]
    )

    return ld



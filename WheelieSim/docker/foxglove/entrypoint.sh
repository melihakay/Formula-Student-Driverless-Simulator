#!/bin/bash

ROS_PATH=/opt/ros/humble
ROS_WS_PATH=/ros-ws

source $ROS_PATH/setup.bash
source $ROS_WS_PATH/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

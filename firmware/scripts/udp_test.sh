#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/src/turtle_bot/microros_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --verbose 5

# source /opt/ros/humble/setup.zsh
# ros2 topic echo micro_ros_platformio_node_publisher

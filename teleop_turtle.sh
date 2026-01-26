#!/bin/bash

# for project 1 only. 

# for sim
TURTLEBOT3_MODEL=burger ros2 run turtlebot3_teleop teleop_keyboard #--ros-args -r __ns:=/turtle

# for physical control
# ROS_DOMAIN_ID=1 TURTLEBOT3_MODEL=burger ros2 run turtlebot3_teleop teleop_keyboard #--ros-args -r __ns:=/turtle

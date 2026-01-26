#!/bin/bash

source install/setup.bash
# ros2 daemon stop
ROS_DOMAIN_ID=23 ros2 launch ee4308_bringup proj1.launch.py
./kill_gz.sh
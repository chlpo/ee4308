#!/bin/bash

source install/setup.bash
# ros2 daemon stop
ROS_DOMAIN_ID=$1 ros2 launch ee4308_bringup $2.launch.py $3
./kill_gz.sh
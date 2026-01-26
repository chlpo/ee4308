#!/bin/bash

source install/setup.bash
# ros2 daemon stop
ros2 launch ee4308_bringup proj1_sim_slam.launch.py
./kill_gz.sh

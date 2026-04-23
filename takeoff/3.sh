#!/usr/bin/env bash
set -eo pipefail
conda init || true
conda deactivate || true
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
source /opt/ros/humble/setup.bash
source /home/carlson/Auto-Multilift_simulation/install/local_setup.bash

ros2 launch px4_offboard multilift_mpc.launch.py

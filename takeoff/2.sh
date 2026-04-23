set -eo pipefail

conda deactivate || true
pkill -f px4 || true
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
source /home/carlson/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash

ISAACSIM_PYTHON /home/carlson/Auto-Multilift_simulation/src/sitl_sim/sitl_sim/sitl_stable.py

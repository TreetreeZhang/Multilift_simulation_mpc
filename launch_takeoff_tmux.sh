#!/usr/bin/env bash
# tmux launch for takeoff steps

set -eo pipefail
SESSION=${1:-takeoff}
DIR="$(cd "$(dirname "$0")" && pwd)"

# clean old session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# pane-0: Micro XRCE Agent
tmux new -d -s "$SESSION" -n main \
  "bash -lc 'conda deactivate || true && MicroXRCEAgent udp4 -p 8888'"

# pane-1: Isaac Sim + Pegasus (PX4 autolaunch)
tmux split-window -h \
  "bash -lc 'conda deactivate || true && export ROS_DISTRO=humble && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib && source /home/carlson/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash && \${ISAACSIM_PYTHON} $DIR/src/sitl_sim/sitl_sim/sitl_stable.py'"

# pane-2: ROS2 launch
tmux split-window -v -t "$SESSION":0.0 \
  "bash -lc 'conda deactivate || true && export ROS_DISTRO=humble && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib && source /opt/ros/humble/setup.bash && source $DIR/install/local_setup.bash && ros2 launch px4_offboard multilift_mpc.launch.py'"

tmux select-layout tiled
tmux attach -t "$SESSION"

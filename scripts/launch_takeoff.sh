#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${MULTILIFT_TMUX_SESSION:-takeoff}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash}"
ISAAC_ROS_BRIDGE_LIB="${ISAAC_ROS_BRIDGE_LIB:-$HOME/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib}"
SITL_SCRIPT="${SITL_SCRIPT:-$ROOT_DIR/src/sitl_sim/sitl_sim/sitl_stable.py}"
ROS_LAUNCH_FILE="${ROS_LAUNCH_FILE:-multilift_mpc.launch.py}"

usage() {
  cat <<'USAGE'
Usage: scripts/launch_takeoff.sh [agent|sim|ros|all|help]

Commands:
  agent   Start Micro XRCE-DDS Agent.
  sim     Kill stale PX4 processes, then start Isaac Sim SITL.
  ros     Source ROS2 workspace and launch the multilift MPC stack.
  all     Start the three takeoff steps in one tmux session.
  help    Show this help.

Environment overrides:
  MULTILIFT_TMUX_SESSION, ISAACSIM_PYTHON, ISAAC_ROS_WS,
  ISAAC_ROS_BRIDGE_LIB, SITL_SCRIPT, ROS_LAUNCH_FILE
USAGE
}

deactivate_conda() {
  conda deactivate 2>/dev/null || true
}

export_ros_env() {
  export ROS_DISTRO
  export RMW_IMPLEMENTATION
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_ROS_BRIDGE_LIB"
}

run_agent() {
  deactivate_conda
  MicroXRCEAgent udp4 -p 8888
}

run_sim() {
  deactivate_conda
  pkill -f px4 || true
  export_ros_env
  source "$ISAAC_ROS_WS"
  "${ISAACSIM_PYTHON:?ISAACSIM_PYTHON is not set}" "$SITL_SCRIPT"
}

run_ros() {
  deactivate_conda
  export_ros_env
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  source "$ROOT_DIR/install/local_setup.bash"
  ros2 launch px4_offboard "$ROS_LAUNCH_FILE"
}

run_all() {
  command -v tmux >/dev/null
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  tmux new -d -s "$SESSION" -n takeoff \
    "bash -lc '\"$ROOT_DIR/scripts/launch_takeoff.sh\" agent'"
  tmux split-window -h -t "$SESSION":0 \
    "bash -lc '\"$ROOT_DIR/scripts/launch_takeoff.sh\" sim'"
  tmux split-window -v -t "$SESSION":0.0 \
    "bash -lc '\"$ROOT_DIR/scripts/launch_takeoff.sh\" ros'"
  tmux select-layout -t "$SESSION":0 tiled
  tmux attach -t "$SESSION"
}

case "${1:-all}" in
  agent) run_agent ;;
  sim) run_sim ;;
  ros) run_ros ;;
  all) run_all ;;
  help|-h|--help) usage ;;
  *)
    usage
    exit 2
    ;;
esac

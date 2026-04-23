#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/home/carlson/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash}"
ISAAC_ROS_BRIDGE_LIB="${ISAAC_ROS_BRIDGE_LIB:-/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib}"
SITL_SCRIPT="${SITL_SCRIPT:-$ROOT_DIR/src/sitl_sim/sitl_sim/sitl_stable.py}"
ROS_LAUNCH_FILE="${ROS_LAUNCH_FILE:-multilift_mpc.launch.py}"

usage() {
  cat <<'USAGE'
Usage: ./scripts/launch_takeoff.sh [agent|sim|ros|cleanup|all|help]

Commands:
  agent   Run Micro XRCE-DDS Agent.
  sim     Run Isaac Sim SITL step from takeoff.md.
  ros     Run ROS2 multilift launch step from takeoff.md.
  cleanup Stop stale Micro XRCE, PX4, and multilift ROS processes.
  all     Open three normal interactive terminal windows and run agent/sim/ros.
  help    Show this help.

Notes:
  This script intentionally uses normal interactive bash shells so aliases,
  functions, and variables from your usual terminal are available.
USAGE
}

cleanup_cmd() {
  cat <<EOF
cd "$ROOT_DIR"
conda deactivate || true
pkill -f MicroXRCEAgent || true
pkill -f px4 || true
pkill -f multilift_sync_node || true
pkill -f multilift_quad_node || true
pkill -f visualizer || true
pkill -f "ros2 launch px4_offboard" || true
EOF
}

agent_cmd() {
  cat <<EOF
cd "$ROOT_DIR"
conda deactivate || true
MicroXRCEAgent udp4 -p 8888
EOF
}

sim_cmd() {
  cat <<EOF
cd "$ROOT_DIR"
conda deactivate || true
export ROS_DISTRO=$ROS_DISTRO
export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
export LD_LIBRARY_PATH="\$LD_LIBRARY_PATH:$ISAAC_ROS_BRIDGE_LIB"
source "$ISAAC_ROS_WS"
ISAACSIM_PYTHON "$SITL_SCRIPT"
EOF
}

ros_cmd() {
  cat <<EOF
cd "$ROOT_DIR"
conda deactivate || true
source /opt/ros/$ROS_DISTRO/setup.bash
source "$ROOT_DIR/install/local_setup.bash"
ros2 launch px4_offboard "$ROS_LAUNCH_FILE"
EOF
}

run_interactive() {
  local step="$1"
  local cmd

  case "$step" in
    cleanup) cmd="$(cleanup_cmd)" ;;
    agent) cmd="$(agent_cmd)" ;;
    sim) cmd="$(sim_cmd)" ;;
    ros) cmd="$(ros_cmd)" ;;
    *) usage; exit 2 ;;
  esac

  bash -ic "$cmd"
}

terminal_run_command() {
  local step="$1"
  local cmd

  case "$step" in
    cleanup) cmd="$(cleanup_cmd)" ;;
    agent) cmd="$(agent_cmd)" ;;
    sim) cmd="$(sim_cmd)" ;;
    ros) cmd="$(ros_cmd)" ;;
    *) usage; exit 2 ;;
  esac

  printf '%s\nstatus=$?\necho\necho "[multilift] %s exited with status ${status}"\nexec bash -i\n' "$cmd" "$step"
}

open_terminal() {
  local title="$1"
  local step="$2"
  local cmd
  cmd="$(terminal_run_command "$step")"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title "$title" -- bash -ic "$cmd"
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "$title" -e bash -ic "$cmd"
  elif command -v konsole >/dev/null 2>&1; then
    konsole --new-tab --title "$title" -e bash -ic "$cmd"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title "$title" --command "bash -ic '$cmd'"
  else
    echo "No supported terminal emulator found. Run these manually:"
    echo "  ./scripts/launch_takeoff.sh agent"
    echo "  ./scripts/launch_takeoff.sh sim"
    echo "  ./scripts/launch_takeoff.sh ros"
    exit 1
  fi
}

run_all() {
  run_interactive cleanup
  sleep 0.5
  open_terminal "multilift-agent" agent
  sleep "${START_SIM_DELAY:-1}"
  open_terminal "multilift-sim" sim
  sleep "${START_ROS_DELAY:-1}"
  open_terminal "multilift-ros" ros
}

case "${1:-all}" in
  cleanup) run_interactive cleanup ;;
  agent) run_interactive agent ;;
  sim) run_interactive sim ;;
  ros) run_interactive ros ;;
  all) run_all ;;
  help|-h|--help) usage ;;
  *) usage; exit 2 ;;
esac

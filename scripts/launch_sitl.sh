#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${MULTILIFT_TMUX_SESSION:-sitl}"
ISAACSIM_PYTHON="${ISAACSIM_PYTHON:-$HOME/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh}"

usage() {
  cat <<'USAGE'
Usage: scripts/launch_sitl.sh [all|help]

Starts the legacy geometric-control SITL flow in a tmux session.
USAGE
}

run_all() {
  command -v tmux >/dev/null
  tmux kill-session -t "$SESSION" 2>/dev/null || true

  tmux new -d -s "$SESSION" -n sitl \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && ros2 run px4_tf tf_convert'"
  tmux split-window -h -t "$SESSION":0 \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && MicroXRCEAgent udp4 -p 8888'"
  tmux split-window -v -t "$SESSION":0.0 \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && \"$ISAACSIM_PYTHON\" \"$ROOT_DIR/src/sitl_sim/sitl_sim/sitl_stable.py\"'"
  tmux split-window -v -t "$SESSION":0.1 \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && ros2 launch px4_offboard geom_multi.launch.py'"

  tmux select-layout -t "$SESSION":0 tiled
  tmux attach -t "$SESSION"
}

case "${1:-all}" in
  all) run_all ;;
  help|-h|--help) usage ;;
  *)
    usage
    exit 2
    ;;
esac

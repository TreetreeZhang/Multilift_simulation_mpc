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

  tf_pane="$(tmux new -d -s "$SESSION" -n sitl -P -F '#{pane_id}' \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && ros2 run px4_tf tf_convert'"
  )"
  agent_pane="$(tmux split-window -h -t "$tf_pane" -P -F '#{pane_id}' \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && MicroXRCEAgent udp4 -p 8888'"
  )"
  tmux split-window -v -t "$tf_pane" \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && \"$ISAACSIM_PYTHON\" \"$ROOT_DIR/src/sitl_sim/sitl_sim/sitl_stable.py\"'"
  tmux split-window -v -t "$agent_pane" \
    "bash -lc 'source \"$ROOT_DIR/install/setup.bash\" && ros2 launch px4_offboard geom_multi.launch.py'"

  tmux select-layout -t "$SESSION":sitl tiled
  tmux attach -t "$SESSION":sitl
}

case "${1:-all}" in
  all) run_all ;;
  help|-h|--help) usage ;;
  *)
    usage
    exit 2
    ;;
esac

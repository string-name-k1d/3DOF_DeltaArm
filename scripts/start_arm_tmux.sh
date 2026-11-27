#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Local miniPC arm session:
  ./scripts/start_arm_tmux.sh                     local session "arm"
  ./scripts/start_arm_tmux.sh control:=false      feedback only
  ./scripts/start_arm_tmux.sh --session fyp --no-attach

Options:
  --session NAME   tmux session name
  --window NAME    first window name
  --no-attach      do not attach after creating the session
  --               remaining arguments go to hardware.launch.py
EOF
}

die() { echo "start_arm_tmux.sh: $*" >&2; exit 2; }

SESSION=""
WINDOW="arm"
ATTACH=1
LAUNCH_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)   usage; exit 0 ;;
    --session)   [ $# -ge 2 ] || die "missing value for $1"; SESSION="$2"; shift 2 ;;
    --window)    [ $# -ge 2 ] || die "missing value for $1"; WINDOW="$2"; shift 2 ;;
    --no-attach) ATTACH=0; shift ;;
    --)          shift; LAUNCH_ARGS+=("$@"); break ;;
    -*)          die "unknown option: $1" ;;
    *)           LAUNCH_ARGS+=("$1"); shift ;;
  esac
done

command -v tmux >/dev/null 2>&1 || die "tmux not installed (sudo apt install tmux)"

PARENT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [ -d "$PARENT/arm" ]; then
  REPO_DIR="$PARENT"
  ARM_DIR="$PARENT/arm"
else
  ARM_DIR="$PARENT"
  REPO_DIR="$(dirname "$PARENT")"
fi

ENV_FILES=()
[ -f "$REPO_DIR/.env" ] && ENV_FILES+=("$REPO_DIR/.env")
[ -f "$ARM_DIR/.env" ]  && ENV_FILES+=("$ARM_DIR/.env")

set -a
for env_file in "${ENV_FILES[@]}"; do
  . "$env_file"
done
set +a

[ -n "$SESSION" ] || SESSION="${ARM_TMUX_SESSION:-arm}"

ARM_WS="${ARM_WS:-}"
if [ -z "$ARM_WS" ]; then
  src_dir="$(dirname "$REPO_DIR")"
  if [ "$(basename "$src_dir")" = src ]; then
    ARM_WS="$(dirname "$src_dir")"
  else
    ARM_WS="$HOME/arm_ws"
  fi
fi
SETUP_PATH="$ARM_WS/install/setup.bash"
[ -f "$SETUP_PATH" ] || die "workspace setup not found: $SETUP_PATH (set ARM_WS in .env)"

PARAMS=""
if [ -n "${ARM_PARAMS_FILE:-}" ]; then
  case "$ARM_PARAMS_FILE" in
    /*) PARAMS="$ARM_PARAMS_FILE" ;;
    *)  PARAMS="$ARM_DIR/$ARM_PARAMS_FILE" ;;
  esac
  [ -f "$PARAMS" ] || die "parameter file not found: $PARAMS (fix ARM_PARAMS_FILE in .env)"
fi

if [ -x "$REPO_DIR/scripts/stop_sim.sh" ]; then
  STOP_SIM="$REPO_DIR/scripts/stop_sim.sh"
else
  STOP_SIM="$ARM_DIR/stop_sim.sh"
fi

q() { printf '%q' "$1"; }

ENV_STEP=""
for env_file in "${ENV_FILES[@]}"; do
  ENV_STEP+=". $(q "$env_file") && "
done

PORT_STEP=""
[ -n "${SERIAL_DEVICE:-}" ] && PORT_STEP="export SERIAL_DEVICE=$(q "$SERIAL_DEVICE") && "

CLEAN_ENV="unset BASH_ENV AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; "
PREFIX="cd $(q "$ARM_DIR") && ${CLEAN_ENV}${ENV_STEP}${PORT_STEP}. $(q "$SETUP_PATH") && "

motion=1
for arg in "${LAUNCH_ARGS[@]}"; do
  case "$arg" in
    control:=false|control:=0) motion=0 ;;
  esac
done

WAIT_ACTION="timeout 60 bash -c 'until ros2 action list 2>/dev/null | grep -qx /arm/set_pos; do sleep 1; done'"

HW_LINE="${PREFIX}ros2 launch arm hardware.launch.py"
[ -n "$PARAMS" ] && HW_LINE+=" params_file:=$(q "$PARAMS")"
for arg in "${LAUNCH_ARGS[@]}"; do
  HW_LINE+=" $(q "$arg")"
done

MANUAL_LINE="${PREFIX}"
[ "$motion" = 1 ] && MANUAL_LINE+="${WAIT_ACTION} && "
MANUAL_LINE+="ros2 run arm arm_manual"
[ -n "$PARAMS" ] && MANUAL_LINE+=" --ros-args --params-file $(q "$PARAMS")"

POS_LINE="${PREFIX}ros2 topic echo /arm/pos arm/msg/ArmPosition"
FEEDBACK_LINE="${PREFIX}ros2 topic echo /arm/MotorFeedback arm/msg/MotorFeedback"

session_exists=0
tmux has-session -t "$SESSION" 2>/dev/null && session_exists=1

window_exists() {
  tmux list-windows -t "$1" -F '#{window_name}' 2>/dev/null | grep -qx "$2"
}

if [ "$session_exists" = 0 ]; then
  echo "start_arm_tmux.sh: creating local session '$SESSION'"
  echo "start_arm_tmux.sh: arm    = $ARM_DIR"
  echo "start_arm_tmux.sh: env    = ${ENV_FILES[*]:-(none)}"
  echo "start_arm_tmux.sh: setup  = $SETUP_PATH"
  if [ -n "${SERIAL_DEVICE:-}" ]; then
    echo "start_arm_tmux.sh: port   = $SERIAL_DEVICE (SERIAL_DEVICE)"
  else
    echo "start_arm_tmux.sh: port   = (unset; driver default /dev/ttyUSB0)"
  fi
  if [ -n "$PARAMS" ]; then
    echo "start_arm_tmux.sh: params = $PARAMS"
  else
    echo "start_arm_tmux.sh: params = (launch defaults)"
  fi
  tmux new-session -d -s "$SESSION" -n "$WINDOW"
fi

if [ "$session_exists" = 1 ] && window_exists "$SESSION" "$WINDOW"; then
  echo "start_arm_tmux.sh: window $WINDOW already in $SESSION; skipping."
else
  if [ "$session_exists" = 1 ]; then
    tmux new-window -t "$SESSION:" -n "$WINDOW"
  fi
  target="$SESSION:$WINDOW"
  tmux set-option -w -t "$target" pane-border-status top
  tmux set-option -w -t "$target" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$target.0" -T 'hardware (ros2 launch arm)'
  tmux send-keys -t "$target.0" "$HW_LINE" C-m
  tmux split-window -h -t "$target"
  tmux select-pane -t "$target.1" -T 'manual (WASD + Z/X, Q quits)'
  tmux send-keys -t "$target.1" "$MANUAL_LINE" C-m
fi

if window_exists "$SESSION" feedback; then
  echo "start_arm_tmux.sh: window feedback already in $SESSION; skipping."
else
  tmux new-window -t "$SESSION:" -n feedback
  target="$SESSION:feedback"
  tmux set-option -w -t "$target" pane-border-status top
  tmux set-option -w -t "$target" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$target.0" -T 'arm/pos'
  tmux send-keys -t "$target.0" "$POS_LINE" C-m
  tmux split-window -h -t "$target"
  tmux select-pane -t "$target.1" -T 'arm/MotorFeedback'
  tmux send-keys -t "$target.1" "$FEEDBACK_LINE" C-m
fi

echo "start_arm_tmux.sh: stop with tmux kill-session -t $SESSION, then:"
echo "  $STOP_SIM"

[ "$ATTACH" = 1 ] || exit 0

tmux select-window -t "$SESSION:$WINDOW"
tmux select-pane -t "$SESSION:$WINDOW.1"
if [ -n "${TMUX:-}" ]; then
  exec tmux switch-client -t "$SESSION"
else
  exec tmux attach -t "$SESSION"
fi

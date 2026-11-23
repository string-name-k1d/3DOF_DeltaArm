#!/usr/bin/env bash
# Real-hardware delta arm in a tmux session that runs on THIS machine; every
# arm process itself runs on the Jetson, reached over SSH.
#
#   ./scripts/start_arm_tmux.sh                        # local session "arm"
#   ./scripts/start_arm_tmux.sh control:=false         # feedback only (no motion)
#   ./scripts/start_arm_tmux.sh --host nvidia@jetson   # override SSH target
#   ./scripts/start_arm_tmux.sh --dir ~/arm_ws/src/fyp
#   ./scripts/start_arm_tmux.sh --session fyp --no-attach   # used by start_tmux.sh
#
# Local tmux windows (each pane SSHes into the Jetson):
#   arm       pane 0: ros2 launch arm hardware.launch.py
#             pane 1: arm_manual      (WASD + Z/X, Q quits; uses ssh -t)
#   feedback  pane 0: ros2 topic echo /arm/pos
#             pane 1: ros2 topic echo /arm/MotorFeedback
#
# Only the SSH target is read from the local environment (ARM_REMOTE,
# ARM_REMOTE_DIR). All arm runtime configuration (SERIAL_DEVICE,
# ARM_PARAMS_FILE, X11, ...) comes from the Jetson's own .env, sourced by each
# pane inside its SSH shell. Nothing is copied from this PC's .env. Each pane
# clears inherited ROS environment first, then sources only that .env and
# ~/arm_ws/install/setup.bash (which already chains the ROS underlay).
#
# The manual pane uses `ssh -t` and the node switches its terminal to
# single-keypress mode, so keys work without Enter. `ros2 launch` is never
# used for it because launch does not forward stdin.
#
# Stop:  tmux kill-session -t <session>
#        ssh <host> 'cd <dir> && . ./.env && ./scripts/stop_sim.sh'
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  sed -n '2,/^set -euo pipefail$/p' "$BASH_SOURCE" \
    | sed -e '$d' -e 's/^# \{0,1\}//'
}

die() { echo "start_arm_tmux.sh: $*" >&2; exit 2; }

# --- options ----------------------------------------------------------------
REMOTE="${ARM_REMOTE:-tegra-ubuntu}"
REMOTE_DIR="${ARM_REMOTE_DIR:-arm_ws/src/fyp}"
SESSION="${ARM_TMUX_SESSION:-arm}"
WINDOW="arm"
ATTACH=1
LAUNCH_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --host)    [ $# -ge 2 ] || die "missing value for $1"; REMOTE="$2"; shift 2 ;;
    --dir)     [ $# -ge 2 ] || die "missing value for $1"; REMOTE_DIR="$2"; shift 2 ;;
    --session) [ $# -ge 2 ] || die "missing value for $1"; SESSION="$2"; shift 2 ;;
    --window)  [ $# -ge 2 ] || die "missing value for $1"; WINDOW="$2"; shift 2 ;;
    --no-attach) ATTACH=0; shift ;;
    --) shift; LAUNCH_ARGS+=("$@"); break ;;
    -*) die "unknown option: $1" ;;
    *) LAUNCH_ARGS+=("$1"); shift ;;
  esac
done

command -v tmux >/dev/null 2>&1 || die "tmux not installed (sudo apt install tmux)"
command -v ssh >/dev/null 2>&1 || die "ssh not found"
[ -n "$REMOTE" ] || die "empty SSH host (use --host or ARM_REMOTE)"

# An SSH session starts in the remote home dir, so reduce ~/$HOME prefixes to
# relative paths rather than letting our quoting escape them on the Jetson.
case "$REMOTE_DIR" in
  '~/'*)       REMOTE_DIR="${REMOTE_DIR#\~/}" ;;
  '$HOME/'*)   REMOTE_DIR="${REMOTE_DIR#\$HOME/}" ;;
esac

# Word-safe quoting for the remote shell, then for the local shell.
rq() { printf '%q' "$1"; }

DIR_Q="$(rq "$REMOTE_DIR")"

# The repo lives at <workspace>/src/fyp on the Jetson. Source exactly two
# things there: its .env (SERIAL_DEVICE, ARM_PARAMS_FILE, X11, ...) and the
# workspace-root install/setup.bash, which already chains the ROS underlay.
# Calling run_arm.sh as a wrapper would re-source ROS and walk overlays again,
# producing "not found: .../local_setup.bash" warnings from stale colcon hooks.
REMOTE_WS="${ARM_REMOTE_WS:-}"
if [ -z "$REMOTE_WS" ]; then
  case "$REMOTE_DIR" in
    */src/*) REMOTE_WS="${REMOTE_DIR%/src/*}" ;;
    *)       REMOTE_WS="$REMOTE_DIR" ;;
  esac
fi
case "$REMOTE_WS" in
  /*) SETUP_PATH="$REMOTE_WS/install/setup.bash" ;;
  *)  SETUP_PATH="\$HOME/$REMOTE_WS/install/setup.bash" ;;
esac

# Discard any ROS environment inherited from the ssh login shell (a ros2_ws
# source in .bashrc/BASH_ENV is the usual culprit). Otherwise colcon chains
# those stale prefixes and emits "not found: .../local_setup.bash" while never
# adding arm_ws to AMENT_PREFIX_PATH. .env and the workspace setup restore
# everything that is actually needed.
CLEAN_ENV="unset BASH_ENV AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; "
PREFIX="cd $DIR_Q && ${CLEAN_ENV}. ./.env && . $SETUP_PATH && "

# control:=false disables the action server; there is nothing to wait for.
motion=1
for arg in "${LAUNCH_ARGS[@]}"; do
  case "$arg" in
    control:=false|control:=0) motion=0 ;;
  esac
done

# The manual pane waits for the controller itself so this script never blocks
# (important when it is called from start_tmux.sh). Only pass a params file
# when the Jetson's .env actually sets ARM_PARAMS_FILE: the launch file and the
# manual node both have working defaults, while a guessed relative path (e.g.
# config/arm_params.yaml) does not exist from the repo root. The ${VAR:+...}
# expansion is left literal here for the REMOTE shell to evaluate after .env.
WAIT_ACTION="timeout 60 bash -c 'until ros2 action list 2>/dev/null | grep -qx /arm/set_pos; do sleep 1; done'"
MANUAL_LINE="${PREFIX}"
[ "$motion" = 1 ] && MANUAL_LINE+="${WAIT_ACTION} && "
MANUAL_LINE+="ros2 run arm arm_manual \${ARM_PARAMS_FILE:+--ros-args --params-file \"\$ARM_PARAMS_FILE\"}"

HW_LINE="${PREFIX}ros2 launch arm hardware.launch.py \${ARM_PARAMS_FILE:+params_file:=\"\$ARM_PARAMS_FILE\"}"
for arg in "${LAUNCH_ARGS[@]}"; do
  HW_LINE+=" $(rq "$arg")"
done
POS_LINE="${PREFIX}ros2 topic echo /arm/pos arm/msg/ArmPosition"
FEEDBACK_LINE="${PREFIX}ros2 topic echo /arm/MotorFeedback arm/msg/MotorFeedback"

# Build a local shell command for a tmux pane. $1=1 -> ssh -t (interactive).
pane_cmd() {
  local tty="$1" line="$2" inner wrapper
  # ssh runs its command through the remote login shell first. Wrap ours in a
  # clean bash so nothing else (e.g. a ros2_ws source in ~/.bashrc or BASH_ENV)
  # is sourced before .env and <ws>/install/setup.bash.
  inner="$(printf '%q' "$line")"
  wrapper="env -u BASH_ENV bash --noprofile --norc -c $inner"
  if [ "$tty" = 1 ]; then
    printf 'ssh -t %q %q' "$REMOTE" "$wrapper"
  else
    printf 'ssh %q %q' "$REMOTE" "$wrapper"
  fi
}
HW_PANE="$(pane_cmd 0 "$HW_LINE")"
MANUAL_PANE="$(pane_cmd 1 "$MANUAL_LINE")"
POS_PANE="$(pane_cmd 0 "$POS_LINE")"
FEEDBACK_PANE="$(pane_cmd 0 "$FEEDBACK_LINE")"

# --- session / windows ------------------------------------------------------
session_exists=0
tmux has-session -t "$SESSION" 2>/dev/null && session_exists=1

window_exists() {
  tmux list-windows -t "$1" -F '#{window_name}' 2>/dev/null | grep -qx "$2"
}

if [ "$session_exists" = 0 ]; then
  echo "start_arm_tmux.sh: creating local session '$SESSION' (host $REMOTE)"
  tmux new-session -d -s "$SESSION" -n "$WINDOW"
fi

# A freshly created session already owns a window named $WINDOW (index 0);
# populate it. Only skip when the session pre-existed with that window.
if [ "$session_exists" = 1 ] && window_exists "$SESSION" "$WINDOW"; then
  echo "start_arm_tmux.sh: window $WINDOW already in $SESSION; skipping."
else
  if [ "$session_exists" = 1 ]; then
    tmux new-window -t "$SESSION:" -n "$WINDOW"
  fi
  target="$SESSION:$WINDOW"
  tmux set-option -w -t "$target" pane-border-status top
  tmux set-option -w -t "$target" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$target.0" -T 'hardware (SSH: ros2 launch arm)'
  tmux send-keys -t "$target.0" "$HW_PANE" C-m
  tmux split-window -h -t "$target"
  tmux select-pane -t "$target.1" -T 'manual (SSH -t: WASD + Z/X, Q quits)'
  tmux send-keys -t "$target.1" "$MANUAL_PANE" C-m
fi

if window_exists "$SESSION" feedback; then
  echo "start_arm_tmux.sh: window feedback already in $SESSION; skipping."
else
  tmux new-window -t "$SESSION:" -n feedback
  target="$SESSION:feedback"
  tmux set-option -w -t "$target" pane-border-status top
  tmux set-option -w -t "$target" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$target.0" -T 'arm/pos (SSH)'
  tmux send-keys -t "$target.0" "$POS_PANE" C-m
  tmux split-window -h -t "$target"
  tmux select-pane -t "$target.1" -T 'arm/MotorFeedback (SSH)'
  tmux send-keys -t "$target.1" "$FEEDBACK_PANE" C-m
fi

echo "start_arm_tmux.sh: remote config = ssh $REMOTE 'cd $REMOTE_DIR && . ./.env'"
if [ "$ATTACH" = 1 ]; then
  echo "start_arm_tmux.sh: stop with tmux kill-session -t $SESSION, then:"
else
  echo "start_arm_tmux.sh: stop with tmux kill-window -t $SESSION:$WINDOW and"
  echo "                   tmux kill-window -t $SESSION:feedback, then:"
fi
echo "  ssh $REMOTE 'cd $REMOTE_DIR && . ./.env && ./scripts/stop_sim.sh'"

[ "$ATTACH" = 1 ] || exit 0

if [ -n "${TMUX:-}" ]; then
  tmux select-window -t "$SESSION:$WINDOW"
  tmux select-pane -t "$SESSION:$WINDOW.1"
  exec tmux switch-client -t "$SESSION"
else
  tmux select-window -t "$SESSION:$WINDOW"
  tmux select-pane -t "$SESSION:$WINDOW.1"
  exec tmux attach -t "$SESSION"
fi

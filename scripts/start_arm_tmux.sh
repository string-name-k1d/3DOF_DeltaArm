#!/usr/bin/env bash
# Real-hardware delta arm with keyboard (manual) control, in a tmux session.
#
#   ./scripts/start_arm_tmux.sh
#   ./scripts/start_arm_tmux.sh control:=false     # feedback only (no motion)
#   ./scripts/start_arm_tmux.sh driver:=false      # dry run without servos
#
# Windows:
#   arm       pane 0: run_arm.sh hardware (real motor driver, motion on)
#             pane 1: ros2 run arm arm_manual (WASD + Z/X, Q quits)
#   feedback  pane 0: ros2 topic echo /arm/pos
#             pane 1: ros2 topic echo /arm/MotorFeedback
#
# The manual node is started with `ros2 run` because `ros2 launch` does not
# forward terminal stdin to child nodes. It switches its own terminal to
# single-keypress mode, so keys work over a local TTY or `ssh -t` without any
# `stty` setup. No display is needed.
#
# Configuration is local: the .env beside run_arm.sh (sourced again inside
# each pane because tmux panes do not reliably inherit the caller's
# environment) and config/arm_params.yaml, or ARM_PARAMS_FILE when set.
#
# Stop:  tmux kill-session -t arm
#        ./scripts/stop_sim.sh        # clear any surviving nodes
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  sed -n '2,/^set -euo pipefail$/p' "$BASH_SOURCE" \
    | sed -e '$d' -e 's/^# \{0,1\}//'
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
esac

command -v tmux >/dev/null 2>&1 || {
  echo "start_arm_tmux.sh: tmux not installed (sudo apt install tmux)." >&2
  exit 1
}

# --- locate the arm checkout and its runner --------------------------------
if [ -f "$SCRIPT_DIR/../arm/run_arm.sh" ]; then
  # fyp workspace: scripts/start_arm_tmux.sh next to the mirrored runners.
  LAYOUT=fyp
  ARM_DIR="$(cd "$SCRIPT_DIR/../arm" && pwd)"
  RUN_ARM="$SCRIPT_DIR/run_arm.sh"
  STOP_SIM="$SCRIPT_DIR/stop_sim.sh"
  if [ ! -x "$RUN_ARM" ]; then
    echo "start_arm_tmux.sh: $RUN_ARM is missing; run ./scripts/setup.sh first." >&2
    exit 1
  fi
elif [ -f "$SCRIPT_DIR/../run_arm.sh" ]; then
  # arm package checkout: scripts/start_arm_tmux.sh next to run_arm.sh.
  LAYOUT=arm
  ARM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
  RUN_ARM="$SCRIPT_DIR/../run_arm.sh"
  STOP_SIM="$SCRIPT_DIR/../stop_sim.sh"
else
  echo "start_arm_tmux.sh: cannot find run_arm.sh for $SCRIPT_DIR." >&2
  exit 1
fi

# --- local configuration ---------------------------------------------------
ENV_FILES=()
if [ "$LAYOUT" = fyp ] && [ -f "$SCRIPT_DIR/../.env" ]; then
  ENV_FILES+=("$(cd "$SCRIPT_DIR/.." && pwd)/.env")
fi
if [ -f "$ARM_DIR/.env" ]; then
  ENV_FILES+=("$ARM_DIR/.env")
fi
for env_file in "${ENV_FILES[@]}"; do
  # shellcheck disable=SC1090
  . "$env_file"
done

# Read after .env so ARM_TMUX_SESSION there takes effect.
SESSION="${ARM_TMUX_SESSION:-arm}"

if [ -n "${ARM_PARAMS_FILE:-}" ]; then
  case "$ARM_PARAMS_FILE" in
    /*) PARAMS="$ARM_PARAMS_FILE" ;;
    *) PARAMS="$ARM_DIR/$ARM_PARAMS_FILE" ;;
  esac
else
  PARAMS="$ARM_DIR/config/arm_params.yaml"
fi
if [ ! -f "$PARAMS" ]; then
  echo "start_arm_tmux.sh: parameter file '$PARAMS' not found." >&2
  echo "  Set ARM_PARAMS_FILE in .env or provide config/arm_params.yaml." >&2
  exit 1
fi

# --- pane commands ---------------------------------------------------------
# %q makes each value safe to paste into the pane's shell.
q() { printf '%q' "$1"; }

ENV_STEP=""
for env_file in "${ENV_FILES[@]}"; do
  ENV_STEP+=". $(q "$env_file") && "
done

# Real hardware (driver:=true, control:=true); extra args are launch args.
HW_CMD="cd $(q "$ARM_DIR") && ${ENV_STEP}export ARM_PARAMS_FILE=$(q "$PARAMS") && $(q "$RUN_ARM") hardware"
for arg in "$@"; do
  HW_CMD+=" $(q "$arg")"
done

# Terminal mode is handled inside the node; no shell `stty` needed.
MANUAL_CMD="${ENV_STEP}$(q "$RUN_ARM") ros2 run arm arm_manual --ros-args --params-file $(q "$PARAMS")"

# Explicit message types let the echoes subscribe before the arm stack starts.
POS_CMD="${ENV_STEP}$(q "$RUN_ARM") ros2 topic echo /arm/pos arm/msg/ArmPosition"
FEEDBACK_CMD="${ENV_STEP}$(q "$RUN_ARM") ros2 topic echo /arm/MotorFeedback arm/msg/MotorFeedback"

# --- session ---------------------------------------------------------------
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "start_arm_tmux.sh: session = $SESSION"
  echo "start_arm_tmux.sh: arm     = $ARM_DIR"
  if [ ${#ENV_FILES[@]} -gt 0 ]; then
    echo "start_arm_tmux.sh: env     = ${ENV_FILES[*]}"
  else
    echo "start_arm_tmux.sh: env     = (none; using defaults)"
  fi
  echo "start_arm_tmux.sh: params  = $PARAMS"

  tmux new-session -d -s "$SESSION" -n arm
  tmux set-option -w -t "$SESSION:arm" pane-border-status top
  tmux set-option -w -t "$SESSION:arm" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$SESSION:arm.0" -T 'hardware (run_arm.sh hardware)'
  tmux send-keys -t "$SESSION:arm.0" "$HW_CMD" C-m

  # control:=false disables the action server, so there is nothing to wait for
  # (and the manual node cannot move the arm).
  motion=1
  for arg in "$@"; do
    case "$arg" in
      control:=false|control:=0) motion=0 ;;
    esac
  done

  if [ "$motion" = 1 ]; then
    # Wait for the controller before starting the manual node, so its first
    # target is based on a real arm/pos sample instead of the (0,0,0) fallback.
    echo "start_arm_tmux.sh: waiting for /arm/set_pos..."
    if "$RUN_ARM" bash -c '
      for _ in $(seq 1 60); do
        ros2 action list 2>/dev/null | grep -qx "/arm/set_pos" && exit 0
        sleep 1
      done
      exit 1
    '; then
      echo "start_arm_tmux.sh: controller ready."
    else
      echo "start_arm_tmux.sh: WARNING: /arm/set_pos not seen after 60s;" >&2
      echo "                   starting manual control anyway." >&2
    fi
  else
    echo "start_arm_tmux.sh: control:=false (feedback only) - skipping wait;"
    echo "                   manual control cannot move the arm in this mode."
  fi

  tmux split-window -h -t "$SESSION:arm"
  tmux select-pane -t "$SESSION:arm.1" -T 'manual (WASD + Z/X, Q quits)'
  tmux send-keys -t "$SESSION:arm.1" "$MANUAL_CMD" C-m

  # Telemetry window. The explicit topic types make `ros2 topic echo` subscribe
  # immediately; it simply stays silent until the arm stack publishes.
  tmux new-window -t "$SESSION" -n feedback
  tmux set-option -w -t "$SESSION:feedback" pane-border-status top
  tmux set-option -w -t "$SESSION:feedback" pane-border-format ' #{pane_title} '
  tmux select-pane -t "$SESSION:feedback.0" -T 'arm/pos'
  tmux send-keys -t "$SESSION:feedback.0" "$POS_CMD" C-m
  tmux split-window -h -t "$SESSION:feedback"
  tmux select-pane -t "$SESSION:feedback.1" -T 'arm/MotorFeedback'
  tmux send-keys -t "$SESSION:feedback.1" "$FEEDBACK_CMD" C-m

  # Leave the user on the manual pane so keys go to the node.
  tmux select-window -t "$SESSION:arm"
  tmux select-pane -t "$SESSION:arm.1"
else
  echo "start_arm_tmux.sh: attaching to existing session '$SESSION'"
fi

echo "start_arm_tmux.sh: stop with: tmux kill-session -t $SESSION && $(q "$STOP_SIM")"

if [ -n "${TMUX:-}" ]; then
  exec tmux switch-client -t "$SESSION"
else
  exec tmux attach -t "$SESSION"
fi

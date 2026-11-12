#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# launch_container.sh
#
# Host-side helper: launch the arm Docker container, open an interactive bash
# inside it, and start the repo's run_arm.sh.
#
# Run from the host (outside the container), from anywhere:
#
#   scripts/launch_container.sh                 # run_arm.sh simulation + bash
#   scripts/launch_container.sh --hardware      # hardware stack (USB) + bash
#   scripts/launch_container.sh --foreground    # run_arm.sh in front; bash afterwards
#   scripts/launch_container.sh -- <run_arm.sh args...>
#
# By default run_arm.sh is started in the background (logs in
# /tmp/run_arm.log) so the bash prompt stays usable. Ctrl-D / exit in the
# shell stops the container and the background launch.
#
# The host X11 display is forwarded (DISPLAY, /tmp/.X11-unix, XAUTHORITY) when
# available, so `ros2 launch arm arm_sim_2d.launch.py` opens its window from
# inside the container. Headless hosts still work: only the window is affected.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

SERVICE=arm
FOREGROUND=0
ARGS_GIVEN=0
RUN_ARGS=()

usage() {
  sed -n '3,21p' "$BASH_SOURCE" | sed -e 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--hardware)   SERVICE=arm-hardware; shift ;;
    -f|--foreground) FOREGROUND=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    --)              shift; RUN_ARGS+=("$@"); ARGS_GIVEN=1; break ;;
    *)               echo "[launch] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "[launch] ERROR: docker (or the podman docker shim) is not available." >&2
  exit 1
fi

# Defaults when no explicit run_arm.sh arguments were given: the plain `arm`
# service runs the controller-only simulation (safe without the servo
# adapter); `--hardware` runs the normal hardware launch.
if [ "$ARGS_GIVEN" = "0" ]; then
  if [ "$SERVICE" = "arm-hardware" ]; then
    RUN_ARGS=()
  else
    RUN_ARGS=(simulation)
  fi
fi

if [ "$SERVICE" = "arm-hardware" ]; then
  serial="${SERIAL_DEVICE:-/dev/ttyUSB0}"
  if [ ! -e "$serial" ]; then
    echo "[launch] ERROR: $serial not found. Plug in the adapter or set SERIAL_DEVICE=..." >&2
    exit 1
  fi
fi

# ---------------------------------------------------------------------------
# X11: forward the host display into the container. Mounts are only added when
# the source exists, because podman-backed compose errors on a missing bind
# source; headless runs still start (arm_sim_sfml then reports the missing
# display and exits instead of hanging the launch).
# ---------------------------------------------------------------------------
X11_ARGS=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  X11_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  xauth="${XAUTHORITY:-$HOME/.Xauthority}"
  if [ -f "$xauth" ] && [ -r "$xauth" ]; then
    X11_ARGS+=(-v "$xauth:/tmp/.Xauthority:ro")
  else
    # Access-control-disabled X servers (e.g. WSLg) need no cookie. Clear the
    # compose default so Xlib does not look for a file that is not mounted.
    X11_ARGS+=(-e XAUTHORITY=)
  fi
elif [ -n "${DISPLAY:-}" ]; then
  echo "[launch] WARNING: DISPLAY=$DISPLAY but /tmp/.X11-unix is missing -" >&2
  echo "[launch]          the SFML window cannot connect; use a desktop session." >&2
fi

if ! docker image inspect arm:latest >/dev/null 2>&1; then
  echo "[launch] image arm:latest not found - building..."
  docker compose build arm
fi

echo "[launch] container service: $SERVICE"
if [ ${#RUN_ARGS[@]} -eq 0 ]; then
  echo "[launch] run_arm.sh (hardware launch)"
else
  echo "[launch] run_arm.sh ${RUN_ARGS[*]}"
fi

# Inside the container: cd to the mounted repo, launch run_arm.sh, then keep
# an interactive shell. Positional args: $1 = foreground flag, rest = run args.
read -r -d '' INNER <<'INNER_EOF' || true
set -u
fg="$1"
shift
cd /root/ws 2>/dev/null || cd /root/ros2_ws/src/arm
if [ "$fg" = "1" ]; then
  ./run_arm.sh "$@"
  echo "[launch] run_arm.sh exited - dropping to bash"
  exec bash
fi
./run_arm.sh "$@" > /tmp/run_arm.log 2>&1 &
pid=$!
echo "[launch] run_arm.sh started in the background (pid $pid)"
echo "[launch] log: tail -f /tmp/run_arm.log"
exec bash
INNER_EOF

docker compose run --rm ${X11_ARGS[@]+"${X11_ARGS[@]}"} "$SERVICE" \
  bash -lc "$INNER" bash "$FOREGROUND" \
  ${RUN_ARGS[@]+"${RUN_ARGS[@]}"}

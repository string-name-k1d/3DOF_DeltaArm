#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

SERVICE=arm-hardware
FOREGROUND=0
ARGS_GIVEN=0
RUN_ARGS=()

usage() {
  cat <<'EOF'
Container helper:
  -H | --hardware   real hardware service (default)
  -s | --sim        simulation service
  -f | --foreground stay in the container shell
  --                pass remaining args to run_arm.sh
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--hardware)   SERVICE=arm-hardware; shift ;;
    -s|--sim)        SERVICE=arm; shift ;;
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

if [ "$ARGS_GIVEN" = "0" ]; then
  if [ "$SERVICE" = "arm-hardware" ]; then
    RUN_ARGS=()
  else
    RUN_ARGS=(2d driver:=false)
  fi
fi

if [ "$SERVICE" = "arm-hardware" ]; then
  serial="${SERIAL_DEVICE:-/dev/ttyUSB0}"
  if [ ! -e "$serial" ]; then
    echo "[launch] ERROR: $serial not found. Plug in the adapter or set SERIAL_DEVICE=..." >&2
    exit 1
  fi
fi

X11_ARGS=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  X11_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  xauth="${XAUTHORITY:-$HOME/.Xauthority}"
  if [ -f "$xauth" ] && [ -r "$xauth" ]; then
    X11_ARGS+=(-v "$xauth:/tmp/.Xauthority:ro")
  else
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
  echo "[launch] run_arm.sh (real hardware + 2-D reference, control on)"
else
  echo "[launch] run_arm.sh ${RUN_ARGS[*]}"
fi

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

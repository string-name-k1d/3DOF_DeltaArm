#!/usr/bin/env bash
# Build the arm package with per-machine defaults from .env.
#
# Usage (from anywhere):
#   ./scripts/build.sh                  # build with .env values
#   ./scripts/build.sh --print-config   # show resolved settings, no build
#   ./scripts/build.sh --event-handlers console_direct+   # extra colcon args
#
# Reads <repo>/.env when present (falling back to ./.env). Shell environment
# variables take precedence over .env, matching Docker Compose.
#
# Relevant .env keys:
#   ROS_DISTRO           ROS 2 install to source (default: humble)
#   ARM_WS               workspace root containing src/ (default: auto-detect)
#   ARM_BUILD_SFML_SIM   ON/OFF for the optional 2-D viewer (default: ON)
#   ARM_BUILD_PACKAGES   packages for --packages-select (default: arm)
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=scripts/dotenv.sh
source "$SCRIPT_DIR/dotenv.sh"
DOTENV_FILE=""
load_dotenv_first "$REPO_ROOT/.env" "$PWD/.env" || true
ENV_FILE="$DOTENV_FILE"

# ROS 2 environment.
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
elif ! command -v colcon >/dev/null 2>&1; then
  echo "build.sh: ROS 2 not found (looked for $ROS_SETUP) and colcon is not on PATH." >&2
  echo "          Install ROS 2 or source its setup.bash first." >&2
  exit 1
fi

# Workspace discovery: walk up from this package until the repo is under
# <candidate>/src/ (arm_ws/src/fyp/arm -> arm_ws). A standalone checkout (repo
# root IS the package) falls back to the repo root, which colcon accepts as a
# base path.
if [ -n "${ARM_WS:-}" ]; then
  if [ ! -d "$ARM_WS" ]; then
    echo "build.sh: ARM_WS='$ARM_WS' is not a directory." >&2
    exit 1
  fi
  WS="$(cd "$ARM_WS" && pwd)"
else
  WS="$REPO_ROOT"
  candidate="$(dirname "$REPO_ROOT")"
  while [ "$candidate" != "/" ]; do
    case "$REPO_ROOT/" in
      "$candidate/src/"*) WS="$candidate"; break ;;
    esac
    candidate="$(dirname "$candidate")"
  done
fi

SFML_SIM="${ARM_BUILD_SFML_SIM:-ON}"
PACKAGES="${ARM_BUILD_PACKAGES:-arm}"

if [ "${1:-}" = "--print-config" ]; then
  echo "env_file=${ENV_FILE:-<none>}"
  echo "repo_root=$REPO_ROOT"
  echo "workspace=$WS"
  echo "ros_distro=${ROS_DISTRO:-humble}"
  echo "ARM_BUILD_SFML_SIM=$SFML_SIM"
  echo "ARM_BUILD_PACKAGES=$PACKAGES"
  echo "SERIAL_DEVICE=${SERIAL_DEVICE:-<unset>}"
  echo "DISPLAY=${DISPLAY:-<unset>}"
  exit 0
fi

echo "build.sh: workspace = $WS"
echo "build.sh: .env = ${ENV_FILE:-<none>}"
echo "build.sh: ARM_BUILD_SFML_SIM = $SFML_SIM"
echo "build.sh: packages = $PACKAGES"

cd "$WS"

# Extra arguments come before the package/CMake options so they are parsed as
# colcon options rather than being consumed by --cmake-args.
# shellcheck disable=SC2086  # PACKAGES is intentionally word-split.
colcon build "$@" --symlink-install --packages-select $PACKAGES \
  --cmake-args "-DARM_BUILD_SFML_SIM=$SFML_SIM"

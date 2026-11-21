#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="soccer_sim_debug"
EXECUTABLE_NAME="soccer_sim_node"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HOST_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=scripts/soccer_config.sh
source "$SCRIPT_DIR/soccer_config.sh"
load_soccer_config "$HOST_ROOT"

CONTAINER="${SOCCER_DOCKER_CONTAINER:-$SOCCER_DEFAULT_DOCKER_CONTAINER}"
WORKDIR="${SOCCER_DOCKER_WORKDIR:-/root/workspace/SoccerSim}"
ROS_SETUP="${SOCCER_DOCKER_ROS_SETUP:-/opt/ros/humble/setup.bash}"
EXTRA_SETUP="${SOCCER_DOCKER_EXTRA_SETUP:-}"
FASTDDS_PROFILE="${SOCCER_FASTDDS_PROFILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"

docker_tty_args=("-i")
if [ -t 1 ]; then
  docker_tty_args=("-it")
fi

if [ "$#" -eq 0 ]; then
  ros_args=(
    "--ros-args"
    "-p"
    "robot_names:=default"
    "-p"
    "ready_slots:=center"
    "-p"
    "control_hz:=10"
  )
else
  ros_args=("$@")
fi

quoted_ros_args=()
for arg in "${ros_args[@]}"; do
  quoted_ros_args+=("$(printf "%q" "$arg")")
done

read -r -d '' inner_cmd <<EOF || true
set -euo pipefail
cd "$WORKDIR"
source "$ROS_SETUP"
if [ -n "$EXTRA_SETUP" ]; then
  source "$EXTRA_SETUP"
fi
if [ ! -f install/setup.bash ]; then
  echo "install/setup.bash not found. Run scripts/debug_build first." >&2
  exit 1
fi
source install/setup.bash
if [ -f "$FASTDDS_PROFILE" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="\${FASTRTPS_DEFAULT_PROFILES_FILE:-$FASTDDS_PROFILE}"
fi
exec ros2 run "$PACKAGE_NAME" "$EXECUTABLE_NAME" ${quoted_ros_args[*]}
EOF

docker exec "${docker_tty_args[@]}" "$CONTAINER" bash -lc "$inner_cmd"

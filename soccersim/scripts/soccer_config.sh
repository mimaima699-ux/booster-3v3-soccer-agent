#!/usr/bin/env bash

SOCCER_DEFAULT_DOCKER_CONTAINER="c972daf7bf19"

# Load the ros_debug auto-debug deployment config. Conventions:
#  - Default path: <host_root>/ros_debug/.soccer_sim.env
#  - Explicit override: SOCCER_SIM_CONFIG_FILE
# The file should only contain SOCCER_DOCKER_*, SOCCER_FASTDDS_PROFILE, and
# SOCCER_GAME_CONTROL_* values. They are read only by deployment scripts under
# scripts/ and do not affect runtime processes inside the container.
load_soccer_config() {
  local host_root="$1"
  local config_file="${SOCCER_SIM_CONFIG_FILE:-$host_root/ros_debug/.soccer_sim.env}"

  if [ ! -f "$config_file" ]; then
    return 0
  fi

  local had_container=0
  local had_workdir=0
  local had_ros_setup=0
  local had_extra_setup=0
  local had_fastdds_profile=0
  local had_control_url=0
  local had_game_control_artifact_dir=0

  local existing_container="${SOCCER_DOCKER_CONTAINER-}"
  local existing_workdir="${SOCCER_DOCKER_WORKDIR-}"
  local existing_ros_setup="${SOCCER_DOCKER_ROS_SETUP-}"
  local existing_extra_setup="${SOCCER_DOCKER_EXTRA_SETUP-}"
  local existing_fastdds_profile="${SOCCER_FASTDDS_PROFILE-}"
  local existing_control_url="${SOCCER_GAME_CONTROL_URL-}"
  local existing_game_control_artifact_dir="${SOCCER_GAME_CONTROL_ARTIFACT_DIR-}"

  if [ "${SOCCER_DOCKER_CONTAINER+x}" = x ]; then had_container=1; fi
  if [ "${SOCCER_DOCKER_WORKDIR+x}" = x ]; then had_workdir=1; fi
  if [ "${SOCCER_DOCKER_ROS_SETUP+x}" = x ]; then had_ros_setup=1; fi
  if [ "${SOCCER_DOCKER_EXTRA_SETUP+x}" = x ]; then had_extra_setup=1; fi
  if [ "${SOCCER_FASTDDS_PROFILE+x}" = x ]; then had_fastdds_profile=1; fi
  if [ "${SOCCER_GAME_CONTROL_URL+x}" = x ]; then had_control_url=1; fi
  if [ "${SOCCER_GAME_CONTROL_ARTIFACT_DIR+x}" = x ]; then had_game_control_artifact_dir=1; fi

  set -a
  # shellcheck disable=SC1090
  source "$config_file"
  set +a

  if [ "$had_container" -eq 1 ]; then SOCCER_DOCKER_CONTAINER="$existing_container"; fi
  if [ "$had_workdir" -eq 1 ]; then SOCCER_DOCKER_WORKDIR="$existing_workdir"; fi
  if [ "$had_ros_setup" -eq 1 ]; then SOCCER_DOCKER_ROS_SETUP="$existing_ros_setup"; fi
  if [ "$had_extra_setup" -eq 1 ]; then SOCCER_DOCKER_EXTRA_SETUP="$existing_extra_setup"; fi
  if [ "$had_fastdds_profile" -eq 1 ]; then SOCCER_FASTDDS_PROFILE="$existing_fastdds_profile"; fi
  if [ "$had_control_url" -eq 1 ]; then SOCCER_GAME_CONTROL_URL="$existing_control_url"; fi
  if [ "$had_game_control_artifact_dir" -eq 1 ]; then SOCCER_GAME_CONTROL_ARTIFACT_DIR="$existing_game_control_artifact_dir"; fi
}

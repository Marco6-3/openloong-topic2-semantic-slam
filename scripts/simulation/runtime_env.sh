#!/usr/bin/env bash

# Sourced by runtime commands after Pixi has activated the isolated ROS Noetic environment.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
simulation_workspace="$repo_root/data/simulation/catkin_ws"
official_model_path="$repo_root/data/simulation/official/gazebo_models"
runtime_state="$repo_root/data/simulation/runtime"

[[ "${ROS_DISTRO:-}" == "noetic" ]] || { echo "expected isolated ROS Noetic environment" >&2; return 1; }
[[ -f "$simulation_workspace/devel/setup.bash" ]] || { echo "simulation is not built; run ./scripts/simulation/sim.sh prepare" >&2; return 1; }

set +u
source "$simulation_workspace/devel/setup.bash"
set -u

mkdir -p "$runtime_state/ros-home" "$runtime_state/ros-logs" "$runtime_state/gazebo-logs"
export ROS_HOME="$runtime_state/ros-home"
export ROS_LOG_DIR="$runtime_state/ros-logs"
export ROS_TEST_RESULTS_DIR="$runtime_state/ros-test-results"
export GAZEBO_LOG_PATH="$runtime_state/gazebo-logs"
export GAZEBO_MODEL_PATH="$official_model_path${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$simulation_workspace/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
# Every required model is local and verified, so Gazebo must not contact a public model database.
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
# Keep the ROS Noetic end-of-life dialog out of the recording without changing shell startup files.
export DISABLE_ROS1_EOL_WARNINGS=1

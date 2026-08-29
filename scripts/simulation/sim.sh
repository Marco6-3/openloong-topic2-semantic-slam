#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
manifest="$repo_root/simulation/pixi.toml"
command_name="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

run_isolated() {
  env \
    -u AMENT_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH \
    -u ROS_DISTRO \
    -u ROS_ETC_DIR \
    -u ROS_PACKAGE_PATH \
    -u ROS_PYTHON_VERSION \
    -u ROS_ROOT \
    -u ROS_VERSION \
    pixi run -m "$manifest" "$@"
}

case "$command_name" in
  prepare)
    run_isolated check
    ;;
  build)
    run_isolated build
    ;;
  demo)
    run_isolated demo "$@"
    ;;
  headless)
    run_isolated demo gazebo_gui:=false headless:=true rviz:=false "$@"
    ;;
  monitor)
    run_isolated monitor
    ;;
  semantic-log)
    run_isolated semantic-log
    ;;
  validate)
    run_isolated validate "$@"
    ;;
  model)
    bash "$repo_root/scripts/simulation/prepare_semantic_model.sh"
    ;;
  help|-h|--help)
    cat <<'EOF'
Usage: ./scripts/simulation/sim.sh COMMAND [roslaunch arguments]

  prepare       download, verify, patch, build, and check the isolated simulator
  model         download and export the pinned YOLOv8n-seg 320 ONNX model
  demo          start Gazebo GUI, semantic SLAM, automatic route, and RViz
  headless      start the same pipeline without Gazebo GUI or RViz
  monitor       print combined SLAM, semantic, and real-time-factor status
  semantic-log  print raw /semantic/status messages
  validate      validate a running pipeline for 60 seconds
EOF
    ;;
  *)
    echo "unknown command: $command_name" >&2
    exit 2
    ;;
esac

#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="$repo_root/data/simulation/catkin_ws"
model_path="$repo_root/data/simulation/official/gazebo_models"
semantic_model="$repo_root/data/models/yolov8n-seg-320.onnx"

[[ "${ROS_DISTRO:-}" == "noetic" ]] || { echo "expected ROS_DISTRO=noetic" >&2; exit 1; }
[[ -f "$workspace/devel/setup.bash" ]] || { echo "simulation is not built" >&2; exit 1; }
[[ -d "$model_path" ]] || { echo "missing official Gazebo models" >&2; exit 1; }
[[ -f "$workspace/devel/lib/libgazebo_ros_velodyne_gpu_laser.so" ]] || { echo "missing VLP-16 GPU ray plugin" >&2; exit 1; }
grep -q 'gpu="true"' "$workspace/src/vehicle_simulator/urdf/lidar.urdf.xacro"
grep -q '<real_time_update_rate>400</real_time_update_rate>' "$workspace/src/vehicle_simulator/world/house.world"
semantic_nodes="$repo_root/simulation/src/openloong_semantic_slam/nodes"
grep -q 'ApproximateTimeSynchronizer' "$semantic_nodes/semantic_mapper_init.py"
grep -q 'lookup_transform' "$semantic_nodes/semantic_mapper_pipeline.py"
if grep -qE 'camera_up = xyz|xyz\[:, 2\] \+ 0\.0377' "$semantic_nodes"/semantic_mapper*.py; then
  echo "semantic mapper still contains the removed hard-coded camera-height projection" >&2
  exit 1
fi

set +u
source "$workspace/devel/setup.bash"
set -u
export GAZEBO_MODEL_PATH="$model_path${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

for package in vehicle_simulator velodyne_gazebo_plugins openloong_semantic_slam slam_toolbox; do
  rospack find "$package" >/dev/null
done

python - <<'PY'
import cv2
import message_filters
import numpy
import rospy
print(
    f"[check] ROS={rospy.get_param.__module__.split('.')[0]} "
    f"OpenCV={cv2.__version__} NumPy={numpy.__version__} "
    f"message_filters={message_filters.__name__}"
)
PY

if [[ -f "$semantic_model" ]]; then
  SEMANTIC_MODEL="$semantic_model" python - <<'PY'
import os
import cv2
network = cv2.dnn.readNetFromONNX(os.environ["SEMANTIC_MODEL"])
print(f"[check] semantic ONNX loaded, output layers={network.getUnconnectedOutLayersNames()}")
PY
else
  echo "[check] semantic model not prepared yet; run ./scripts/simulation/sim.sh model"
fi

set +e
gazebo_version="$(gazebo --version 2>&1)"
set -e
grep -q '^Gazebo multi-robot simulator, version 11\.' <<<"$gazebo_version" || {
  printf '%s\n' "$gazebo_version" >&2
  echo "unexpected Gazebo version output" >&2
  exit 1
}
printf '%s\n' "${gazebo_version%%$'\n'*}"
echo "[check] model path: $GAZEBO_MODEL_PATH"

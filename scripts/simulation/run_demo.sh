#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
model_path="${OPENLOONG_SEMANTIC_MODEL:-$repo_root/data/models/yolov8n-seg-320.onnx}"

source "$script_dir/runtime_env.sh"
[[ -f "$model_path" ]] || {
  echo "missing semantic model: $model_path" >&2
  echo "run ./scripts/simulation/sim.sh model first" >&2
  exit 1
}

exec roslaunch openloong_semantic_slam slam.launch semantic_model:="$model_path" "$@"
